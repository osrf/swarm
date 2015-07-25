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

#include <string>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <ignition/transport.hh>
#include <sdf/sdf.hh>
#include "msgs/datagram.pb.h"
#include "msgs/neighbor_v.pb.h"
#include "swarm/BrokerPlugin.hh"

using namespace swarm;

GZ_REGISTER_WORLD_PLUGIN(BrokerPlugin)

//////////////////////////////////////////////////
void BrokerPlugin::Load(gazebo::physics::WorldPtr _world,
                             sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "BrokerPlugin::Load() error: _world pointer is NULL");
  GZ_ASSERT(_sdf, "BrokerPlugin::Load() error: _sdf pointer is NULL");

  this->world = _world;

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
    return;
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
          this->swarm[address] = newMember;

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

  if (this->swarm.empty())
    gzerr << "BrokerPlugin::ReadSwarmFromSDF: No members found" << std::endl;
  else
    gzmsg << "BrokerPlugin::ReadSwarmFromSDF: " << this->swarm.size()
          << " swarm members found" << std::endl;
}

//////////////////////////////////////////////////
void BrokerPlugin::Update(const gazebo::common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Update the list of neighbors for each robot.
  for (auto const &robot : this->swarm)
    this->UpdateNeighborList(robot.first);

  // Dispatch all incoming messages.
  while (!this->incomingMsgs.empty())
  {
    // Get the next message to dispatch.
    auto msg = this->incomingMsgs.front();
    this->incomingMsgs.pop();

    // Sanity check: Make sure that the sender is a member of the swarm.
    if (this->swarm.find(msg.src_address()) == this->swarm.end())
    {
      gzerr << "BrokerPlugin::Update(): Discarding message. Robot ["
            << msg.src_address() << "] is not registered as a member of the "
            << "swarm" << std::endl;
      continue;
    }

    // Add the list of neighbors of the sender to the outgoing message.
    for (auto const &neighbor : this->swarm[msg.src_address()]->neighbors)
      msg.add_neighbors(neighbor);

    // Create the topic name for the message destination.
    const std::string topic =
        "/swarm/" + msg.dst_address() + "/" + std::to_string(msg.dst_port());

    // Forward the message to the destination.
    this->node.Advertise(topic);
    this->node.Publish(topic, msg);
  }
}

//////////////////////////////////////////////////
void BrokerPlugin::UpdateNeighborList(const std::string &_address)
{
  GZ_ASSERT(this->swarm.find(_address) != this->swarm.end(),
            "_address not found in the swarm.");

  auto swarmMember = this->swarm[_address];

  // Update the neighbor list for this robot.
  // ToDo: For now we include all the robots as neighbors.
  //       In the future we should do something smarter.
  swarmMember->neighbors.clear();
  for (auto const &member : this->swarm)
    swarmMember->neighbors.push_back(member.first);

  // Fill the message with the new neighbor list.
  swarm::msgs::Neighbor_V msg;
  for (auto const &neighbor : swarmMember->neighbors)
    msg.add_neighbors(neighbor);

  // Notify the node with its updated list of neighbors.
  std::string topic = "/swarm/" + swarmMember->address + "/neighbors";
  this->node.Publish(topic, msg);
}

//////////////////////////////////////////////////
void BrokerPlugin::OnMsgReceived(const std::string &/*_topic*/,
    const msgs::Datagram &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Queue the new message.
  this->incomingMsgs.push(_msg);
}
