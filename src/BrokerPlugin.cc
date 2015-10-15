/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <algorithm>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Rand.hh>
#include <sdf/sdf.hh>

#include "msgs/datagram.pb.h"
#include "swarm/CommsModel.hh"
#include "swarm/BrokerPlugin.hh"

using namespace swarm;

GZ_REGISTER_WORLD_PLUGIN(BrokerPlugin)

//////////////////////////////////////////////////
BrokerPlugin::~BrokerPlugin()
{
  gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->logger->Unregister("broker");
}

//////////////////////////////////////////////////
void BrokerPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "BrokerPlugin::Load() error: _world pointer is NULL");
  GZ_ASSERT(_sdf, "BrokerPlugin::Load() error: _sdf pointer is NULL");

  this->world = _world;
  this->swarm = std::make_shared<SwarmMembership_M>();

  this->rndEngine = std::default_random_engine(ignition::math::Rand::Seed());

  // Get the addresses of the swarm.
  this->ReadSwarmFromSDF(_sdf);

  this->commsModel.reset(new CommsModel(this->swarm, this->world, _sdf));

  this->maxDataRatePerCycle = this->commsModel->MaxDataRate() *
      this->world->GetPhysicsEngine()->GetMaxStepSize();

  // Register in the logger.
  this->logger->Register("broker", this);

  // Listen to the update event broadcasted every simulation iteration.
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&BrokerPlugin::Update, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void BrokerPlugin::ReadSwarmFromSDF(sdf::ElementPtr _sdf)
{
  sdf::ElementPtr worldSDF = _sdf->GetParent();
  if (!worldSDF)
  {
    gzerr << "BrokerPlugin::ReadSwarmFromSDF() Unable to read world SDF\n";
    gazebo::shutdown();
  }

  // Iterate over all the models looking for <address> inside <plugin>.
  auto modelElem = worldSDF->GetElement("model");
  while (modelElem)
  {
    if (modelElem->HasElement("plugin") && modelElem->HasAttribute("name"))
    {
      auto const &pluginElem = modelElem->GetElement("plugin");
      if (pluginElem->HasElement("address"))
      {
        std::string address = pluginElem->Get<std::string>("address");
        std::string name = modelElem->GetAttribute("name")->GetAsString();
        auto const &model = this->world->GetModel(name);
        if (model)
        {
          // Create a new SwarmMember for storing the vehicle's information.
          auto newMember = std::make_shared<SwarmMember>();
          newMember->address = address;
          newMember->name = name;
          newMember->model = model;
          (*this->swarm)[address] = newMember;
        }
        else
        {
          gzerr << "BrokerPlugin::ReadSwarmFromSDF(): Error getting a model"
                << "pointer for robot [" << name << "]" << std::endl;
        }
      }
    }

    modelElem = modelElem->GetNextElement("model");
  }

  if (this->swarm->empty())
    gzerr << "BrokerPlugin::ReadSwarmFromSDF: No members found" << std::endl;
  else
    gzmsg << "BrokerPlugin::ReadSwarmFromSDF: " << this->swarm->size()
          << " swarm members found" << std::endl;
}

//////////////////////////////////////////////////
void BrokerPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);

    // Update the state of the communication model.
    this->commsModel->Update();

    // Send a message to each swarm member with its updated neighbors list.
    this->NotifyNeighbors();
  }

  // Dispatch all the incoming messages, deciding whether the destination gets
  // the message according to the communication model.
  // Mutex handling is done inside DispatchMessages().
  this->DispatchMessages();

  // Log the current iteration.
  this->logger->Update(_info.simTime.Double());
}

//////////////////////////////////////////////////
void BrokerPlugin::NotifyNeighbors()
{
  const auto &clients = this->broker->Clients();

  // Send neighbors update to each member of the swarm.
  for (auto const &robot : (*this->swarm))
  {
    std::vector<std::string> v;
    auto address = robot.first;
    auto swarmMember = (*this->swarm)[address];

    // This address is not registered as a broker client.
    if (clients.find(address) == clients.end())
      continue;

    for (auto const &neighbor : swarmMember->neighbors)
      v.push_back(neighbor.first);

    // Notify the node with its updated list of neighbors.
    clients.at(address)->OnNeighborsReceived(v);
  }
}

//////////////////////////////////////////////////
void BrokerPlugin::DispatchMessages()
{
  // Create a copy of the incoming message queue, then release the mutex, to
  // avoid the potential for a deadlock later if a robot calls SendTo() inside
  // its message callback.
  std::deque<msgs::Datagram> incomingMsgsBuffer;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    std::swap(incomingMsgsBuffer, this->broker->Messages());
  }

  this->logIncomingMsgs.Clear();

  // Shuffle the messages.
  std::shuffle(incomingMsgsBuffer.begin(), incomingMsgsBuffer.end(),
    this->rndEngine);

  // Get the current list of endpoints and clients bound.
  const auto &endpoints = this->broker->EndPoints();

  // Clear the data rate usage for each robot.
  for (const auto &member : (*this->swarm))
    member.second->dataRateUsage = 0;

  while (!incomingMsgsBuffer.empty())
  {
    // Get the next message to dispatch.
    const auto msg = incomingMsgsBuffer.front();
    incomingMsgsBuffer.pop_front();

    // Sanity check: Make sure that the sender is a member of the swarm.
    if (this->swarm->find(msg.src_address()) == this->swarm->end())
    {
      gzerr << "BrokerPlugin::DispatchMessages(): Discarding message. Robot ["
            << msg.src_address() << "] is not registered as a member of the "
            << "swarm" << std::endl;
      continue;
    }

    // For logging purposes, we store the request for communication.
    auto logMsg = this->logIncomingMsgs.add_message();
    logMsg->set_src_address(msg.src_address());
    logMsg->set_dst_address(msg.dst_address());
    logMsg->set_dst_port(msg.dst_port());
    logMsg->set_size(msg.data().size());

    // Get the list of neighbors of the sender.
    const auto &neighbors = (*this->swarm)[msg.src_address()]->neighbors;

    // Update the data rate usage.
    for (const auto &neighbor : neighbors)
    {
      // We account the overhead caused by the UDP/IP/Ethernet headers + the
      // payload. We convert the total amount of bytes to bits.
      (*this->swarm)[neighbor.first]->dataRateUsage +=
        (msg.data().size() + this->commsModel->UdpOverhead()) * 8;
    }

    auto dstEndPoint = msg.dst_address() + ":" + std::to_string(msg.dst_port());
    if (endpoints.find(dstEndPoint) != endpoints.end())
    {
      auto clientsV = endpoints.at(dstEndPoint);
      for (const auto &client : clientsV)
      {
        // Make sure that we're sending the message to a valid neighbor.
        if (neighbors.find(client.address) == neighbors.end())
          continue;

        auto neighborEntryLog = logMsg->add_neighbor();
        neighborEntryLog->set_dst(client.address);

        // Check if the maximum data rate has been reached in the destination.
        if ((*this->swarm)[client.address]->dataRateUsage >
            this->maxDataRatePerCycle)
        {
          neighborEntryLog->set_status(msgs::CommsStatus::DATARATE);
          // Debug output
          // gzdbg << "Dropping message (max data rate) from "
          //       << msg.src_address() << " to " << client.address
          //       << " (addressed to " << msg.dst_address()
          //       << ")" << std::endl;
          continue;
        }

        // Decide whether this neighbor gets this message, according to the
        // probability of communication between them right now.
        const auto &neighborProb = neighbors.at(client.address);
        if (ignition::math::Rand::DblUniform(0.0, 1.0) < neighborProb)
        {
          // Debug output
          // gzdbg << "Sending message from " << msg.src_address() << " to "
          //       << client.address << " (addressed to " << msg.dst_address()
          //       << ")" << std::endl;
          client.handler->OnMsgReceived(msg);
          neighborEntryLog->set_status(msgs::CommsStatus::DELIVERED);
        }
        else
        {
          neighborEntryLog->set_status(msgs::CommsStatus::DROPPED);
          // Debug output.
          // gzdbg << "Dropping message from " << msg.src_address() << " to "
          //       << client.address << " (addressed to " << msg.dst_address()
          //       << ")" << std::endl;
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void BrokerPlugin::OnLog(msgs::LogEntry &_logEntry) const
{
  // Our logging contribution:
  //   * Visibility information of all the nodes.
  //   * Incoming messages.
  _logEntry.mutable_visibility()->CopyFrom(this->commsModel->VisibilityMap());
  _logEntry.mutable_incoming_msgs()->CopyFrom(this->logIncomingMsgs);
}
