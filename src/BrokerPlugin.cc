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

#include <memory>
#include <string>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/transport.hh>
#include <sdf/sdf.hh>

#include "msgs/datagram.pb.h"
#include "msgs/neighbor_v.pb.h"
#include "swarm/CommsModel.hh"
#include "swarm/BrokerPlugin.hh"

using namespace swarm;

GZ_REGISTER_WORLD_PLUGIN(BrokerPlugin)

//////////////////////////////////////////////////
BrokerPlugin::~BrokerPlugin()
{
  gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

//////////////////////////////////////////////////
void BrokerPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "BrokerPlugin::Load() error: _world pointer is NULL");
  GZ_ASSERT(_sdf, "BrokerPlugin::Load() error: _sdf pointer is NULL");

  this->world = _world;
  this->swarm = std::make_shared<SwarmMembership_M>();

  // This is the subscription that will allow us to receive incoming messages.
  const std::string kBrokerIncomingTopic = "/swarm/broker/incoming";
  if (!this->node.Subscribe(kBrokerIncomingTopic,
      &BrokerPlugin::OnMsgReceived, this))
  {
    gzerr << "BrokerPlugin::Load(): Error trying to subscribe"
          << " on topic " << kBrokerIncomingTopic << std::endl;
  }

  // Get the addresses of the swarm.
  this->ReadSwarmFromSDF(_sdf);

  this->commsModel.reset(new CommsModel(this->swarm, this->world, _sdf));

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

          // Advertise the topic for future neighbor updates for this vehicle.
          std::string topic = "/swarm/" + address + "/neighbors";
          this->node.Advertise(topic);
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

    this->logEntryComms.Clear();

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
  // Send neighbors update to each member of the swarm.
  for (auto const &robot : (*this->swarm))
  {
    auto address = robot.first;
    auto swarmMember = (*this->swarm)[address];
    auto topic = "/swarm/" + swarmMember->address + "/neighbors";

    swarm::msgs::Neighbor_V msg;
    for (auto const &neighbor : swarmMember->neighbors)
      msg.add_neighbors(neighbor.first);

    // Notify the node with its updated list of neighbors.
    this->node.Publish(topic, msg);
  }
}

//////////////////////////////////////////////////
void BrokerPlugin::DispatchMessages()
{
  // Create a copy of the incoming message queue, then release the mutex, to
  // avoid the potential for a deadlock later if a robot calls SendTo() inside
  // its message callback.
  std::queue<msgs::Datagram> incomingMsgsBuffer;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    std::swap(incomingMsgsBuffer, this->incomingMsgs);
  }

  auto commsLog = new msgs::Comms();

  while (!incomingMsgsBuffer.empty())
  {
    // Get the next message to dispatch.
    auto msg = incomingMsgsBuffer.front();
    incomingMsgsBuffer.pop();

    // Debug output.
    // gzdbg << "Processing message from " << msg.src_address()
    //       << " addressed to " << msg.dst_address() << std::endl;

    // Sanity check: Make sure that the sender is a member of the swarm.
    if (this->swarm->find(msg.src_address()) == this->swarm->end())
    {
      gzerr << "BrokerPlugin::Update(): Discarding message. Robot ["
            << msg.src_address() << "] is not registered as a member of the "
            << "swarm" << std::endl;
      continue;
    }

    auto msgLog = commsLog->add_message();
    msgLog->set_src_address(msg.src_address());
    msgLog->set_dst_address(msg.dst_address());
    msgLog->set_dst_port(msg.dst_port());
    msgLog->set_size(msg.data().size());

    // Add the list of neighbors of the sender to the outgoing message.
    for (auto const &neighborKv : (*this->swarm)[msg.src_address()]->neighbors)
    {
      auto neighborId = neighborKv.first;
      auto neighborProb = neighborKv.second;
      msgs::Neighbor::CommsResult commsResult;

      // Decide whether this neighbor gets this message, according to the
      // probability of communication between them right now.
      if (ignition::math::Rand::DblUniform(0.0, 1.0) < neighborProb)
      {
        // Debug output
        // gzdbg << "Sending message from " << msg.src_address() << " to " <<
        //   neighbor.first << " (addressed to " << msg.dst_address() << ")" <<
        //   std::endl;
        msg.add_recipients(neighborId);

        commsResult = msgs::Neighbor::SUCCESS;
      }
      else
      {
        commsResult = msgs::Neighbor::FAIL_DROP;
      //   Debug output.
      //   gzdbg << "Dropping message from " << msg.src_address() << " to " <<
      //     neighbor.first << " (addressed to " << msg.dst_address() << ")" <<
      //     std::endl;
      }

      if ((msg.dst_address() == "broadcast") ||
          (msg.dst_address() != "multicast" && msg.dst_address() == neighborId))
      {
        auto neighborLog = msgLog->add_to_address();
        neighborLog->set_id(neighborId);
        neighborLog->set_result(commsResult);
      }
    }

    // Create the topic name for the message destination.
    const std::string topic =
        "/swarm/" + msg.dst_address() + "/" + std::to_string(msg.dst_port());

    // Forward the message to the destination.
    this->node.Advertise(topic);
    this->node.Publish(topic, msg);
  }

  this->logEntryComms.set_allocated_comms(commsLog);
}

//////////////////////////////////////////////////
void BrokerPlugin::OnMsgReceived(const std::string &/*_topic*/,
    const msgs::Datagram &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Queue the new message.
  this->incomingMsgs.push(_msg);
}

//////////////////////////////////////////////////
void BrokerPlugin::OnLog(msgs::LogEntry &_logEntry) const
{
  _logEntry.mutable_comms()->CopyFrom(this->logEntryComms.comms());
}
