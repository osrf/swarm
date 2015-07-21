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
#include <ignition/transport.hh>
#include <sdf/sdf.hh>
#include "msgs/datagram.pb.h"
#include "swarm/SwarmBrokerPlugin.hh"

using namespace gazebo;
using namespace swarm;

GZ_REGISTER_WORLD_PLUGIN(SwarmBrokerPlugin)

//////////////////////////////////////////////////
void SwarmBrokerPlugin::Load(physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "SwarmBrokerPlugin::Load() error: _world pointer is NULL");
  GZ_ASSERT(_sdf, "SwarmBrokerPlugin::Load() error: _sdf pointer is NULL");

  // This is the subscription that will allow us to receive incoming messages.
  const std::string kBrokerIncomingTopic = "/swarm/broker/incoming";
  if (!this->node.Subscribe(kBrokerIncomingTopic,
      &SwarmBrokerPlugin::OnMsgReceived, this))
  {
    gzerr << "SwarmBrokerPlugin::Load(): Error trying to subscribe"
          << " on topic " << kBrokerIncomingTopic << std::endl;
  }

  // Listen to the update event broadcasted every simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&SwarmBrokerPlugin::Update, this, _1));
}

//////////////////////////////////////////////////
void SwarmBrokerPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Dispatch all incoming messages.
  while (!this->incomingMsgs.empty())
  {
    // Get the next message to dispatch.
    auto msg = this->incomingMsgs.front();
    this->incomingMsgs.pop();

    // ToDo: Get the list of neighbors of the source node and include it in the
    // outgoing message.

    // Create the topic name for the message destination.
    const std::string topic =
        "/swarm/" + msg.dst_address() + "/" +
        std::to_string(msg.dst_port());

    // Forward the message to the destination.
    this->node.Advertise(topic);
    this->node.Publish(topic, msg);
  }
}

//////////////////////////////////////////////////
void SwarmBrokerPlugin::OnMsgReceived(const std::string &/*_topic*/,
    const msgs::Datagram &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Queue the new message.
  this->incomingMsgs.push(_msg);
}
