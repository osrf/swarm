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
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include "msgs/datagram.pb.h"
#include "swarm/SwarmRobotPlugin.hh"

using namespace gazebo;
using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(SwarmRobotPlugin)

//////////////////////////////////////////////////
SwarmRobotPlugin::~SwarmRobotPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

//////////////////////////////////////////////////
void SwarmRobotPlugin::Load(sdf::ElementPtr /*_sdf*/)
{
}

//////////////////////////////////////////////////
bool SwarmRobotPlugin::SendTo(const std::string &_data,
    const std::string &_dstAddress, const uint32_t _port)
{
  msgs::Datagram msg;
  msg.set_src_address(this->Host());
  msg.set_dst_address(_dstAddress);
  msg.set_dst_port(_port);
  msg.set_data(_data);

  // ToDo: Include here the neighbors list.

  // Send the message from the agent to the broker.
  const std::string kBrokerIncomingTopic = "/swarm/broker/incoming";
  if (!this->node.Publish(kBrokerIncomingTopic, msg))
  {
    gzerr << "[" << this->Host() << "] SwarmRobotPlugin::SendTo(): Error "
          << "trying to publish on topic [" << kBrokerIncomingTopic << "]"
          << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
std::string SwarmRobotPlugin::Host() const
{
  return this->address;
}

//////////////////////////////////////////////////
void SwarmRobotPlugin::Update(const common::UpdateInfo &_info)
{
  this->Update(_info);
}

//////////////////////////////////////////////////
void SwarmRobotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "SwarmRobotPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "SwarmRobotPlugin _sdf pointer is NULL");
  this->model = _model;

  // Read the robot address.
  if (!_sdf->HasElement("address"))
  {
    gzerr << "SwarmRobotPlugin::Load(): Unable to find the <address> parameter"
          << std::endl;
    return;
  }

  this->address = _sdf->Get<std::string>("address");

  const std::string kBrokerIncomingTopic = "/swarm/broker/incoming";
  if (!this->node.Advertise(kBrokerIncomingTopic))
  {
    gzerr << "[" << this->Host() << "] SwarmRobotPlugin::Load(): Error "
          << "trying to advertise topic [" << kBrokerIncomingTopic << "]"
          << std::endl;
  }

  // Call the Load() method from the derived plugin.
  this->Load(_sdf);

  // Listen to the update event broadcasted every simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SwarmRobotPlugin::Update, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void SwarmRobotPlugin::OnMsgReceived(const std::string &/*_topic*/,
    const msgs::Datagram &_msg)
{
  const std::string topic = "/swarm/" + _msg.dst_address() + "/" +
      std::to_string(_msg.dst_port());

  if (this->callbacks.find(topic) == this->callbacks.end())
  {
    gzerr << "[" << this->Host() << "] SwarmRobotPlugin::OnMsgReceived(): "
          << "Address [" << topic << "] not found" << std::endl;
    return;
  }

  // There's visibility between source and destination: run the user callback.
  auto const &userCallback = this->callbacks[topic];
  userCallback(_msg.src_address(), _msg.data());
}
