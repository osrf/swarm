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
}

//////////////////////////////////////////////////
void BrokerPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "BrokerPlugin::Load() error: _world pointer is NULL");
  GZ_ASSERT(_sdf, "BrokerPlugin::Load() error: _sdf pointer is NULL");

  this->world = _world;
  this->swarm = std::make_shared<SwarmMembership_M>();

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
  auto t1 = std::chrono::steady_clock::now();
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

  auto t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = t2 - t1;
  //  std::cout << "Broker: " << std::chrono::duration_cast<std::chrono::microseconds>
  //      (elapsed).count() << std::endl;
}

//////////////////////////////////////////////////
void BrokerPlugin::NotifyNeighbors()
{
  // Send neighbors update to each member of the swarm.
  for (auto const &robot : (*this->swarm))
  {
    std::vector<std::string> v;
    auto address = robot.first;
    auto swarmMember = (*this->swarm)[address];
    //auto topic = "/swarm/" + swarmMember->address + "/neighbors";

    //swarm::msgs::Neighbor_V msg;
    for (auto const &neighbor : swarmMember->neighbors)
      v.push_back(neighbor.first);

    // Notify the node with its updated list of neighbors.
    this->broker->clients[address]->OnNeighborsReceived(v);
  }
}

//////////////////////////////////////////////////
void BrokerPlugin::DispatchMessages()
{
  //auto t1 = std::chrono::steady_clock::now();
  // Create a copy of the incoming message queue, then release the mutex, to
  // avoid the potential for a deadlock later if a robot calls SendTo() inside
  // its message callback.
  std::queue<msgs::Datagram> incomingMsgsBuffer;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    std::swap(incomingMsgsBuffer, this->broker->incomingMsgs);
  }

  //this->logIncomingMsgs.Clear();

  while (!incomingMsgsBuffer.empty())
  {
    // Get the next message to dispatch.
    const auto msg = incomingMsgsBuffer.front();
    incomingMsgsBuffer.pop();

    // For logging purposes, we store the request for communication.
    //auto logMsg = this->logIncomingMsgs.add_message();
    //logMsg->set_src_address(msg.src_address());
    //logMsg->set_dst_address(msg.dst_address());
    //logMsg->set_dst_port(msg.dst_port());
    //logMsg->set_size(msg.data().size());

    auto dstEndPoint = msg.dst_address() + ":" + std::to_string(msg.dst_port());
    if (this->broker->listeners.find(dstEndPoint) != this->broker->listeners.end())
    {
      auto clientsV = this->broker->listeners[dstEndPoint];
      for (const auto &client : clientsV)
        client.client->OnMsgReceived(msg);
    }
  }
  //auto t2 = std::chrono::steady_clock::now();
  //std::chrono::duration<double> elapsed = t2 - t1;
  //  std::cout << "Dispatch: " << std::chrono::duration_cast<std::chrono::microseconds>
  //      (elapsed).count() << std::endl;
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
