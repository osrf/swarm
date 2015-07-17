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

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>
#include "msgs/datagram.pb.h"
#include "swarm/SwarmBrokerPlugin.hh"

using namespace gazebo;
using namespace swarm;

GZ_REGISTER_WORLD_PLUGIN(SwarmBrokerPlugin)

//////////////////////////////////////////////////
SwarmBrokerPlugin::SwarmBrokerPlugin()
{
}

//////////////////////////////////////////////////
SwarmBrokerPlugin::~SwarmBrokerPlugin()
{
}

//////////////////////////////////////////////////
void SwarmBrokerPlugin::Load(physics::WorldPtr /*_world*/,
    sdf::ElementPtr /*_sdf*/)
{
  // Initialize transport.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  const std::string kBrokerIncomingTopic = "~/swarm/broker/incoming";

  this->brokerSub = this->node->Subscribe(kBrokerIncomingTopic,
      &SwarmBrokerPlugin::OnMsgReceived, this);
}

//////////////////////////////////////////////////
void SwarmBrokerPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Dispatch all incoming messages.
  while (!this->incomingMsgs.empty())
  {
    auto incomingMsg = this->incomingMsgs.front();
    this->incomingMsgs.pop();

    // ToDo: Get the list of neighbors of the source node and include it in the
    // outgoing message.

    msgs::Datagram outgoingMsg;
    outgoingMsg.mutable_socket()->set_address(incomingMsg.socket().address());
    outgoingMsg.mutable_socket()->set_port(incomingMsg.socket().port());
    outgoingMsg.set_data(incomingMsg.data());

    transport::PublisherPtr pub =
      this->node->Advertise<msgs::Datagram>(incomingMsg.socket().address());
    pub->Publish(outgoingMsg);
  }
}

//////////////////////////////////////////////////
void SwarmBrokerPlugin::OnMsgReceived(ConstDatagramPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Queue the new message.
  this->incomingMsgs.push(*_msg);
}
