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

#include <gazebo/common/Events.hh>
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

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
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
    auto incomingMsg = this->incomingMsgs.front();
    this->incomingMsgs.pop();

    // ToDo: Get the list of neighbors of the source node and include it in the
    // outgoing message.

    msgs::Datagram outgoingMsg;
    outgoingMsg.mutable_socket()->set_dst_address(
      incomingMsg.socket().dst_address());
    outgoingMsg.mutable_socket()->set_port(incomingMsg.socket().port());
    outgoingMsg.set_src_address(incomingMsg.src_address());
    outgoingMsg.set_data(incomingMsg.data());

    const std::string topic =
        "~/swarm/" + incomingMsg.socket().dst_address() + "/" +
        std::to_string(incomingMsg.socket().port());
    transport::PublisherPtr pub =
      this->node->Advertise<msgs::Datagram>(topic);
    pub->Publish(outgoingMsg);

    std::cout << "Broker: Sending new message to " << topic << std::endl;
  }
}

//////////////////////////////////////////////////
void SwarmBrokerPlugin::OnMsgReceived(ConstDatagramPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  std::cout << "Broker: New message received from " << _msg->src_address()
            << std::endl;

  // Queue the new message.
  this->incomingMsgs.push(*_msg);
}
