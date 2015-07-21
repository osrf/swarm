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

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include "msgs/datagram.pb.h"
#include "msgs/socket.pb.h"
#include "swarm/SwarmRobotPlugin.hh"

using namespace gazebo;
using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(SwarmRobotPlugin)

//////////////////////////////////////////////////
void SwarmRobotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "SwarmRobotPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "SwarmRobotPlugin _sdf pointer is NULL");
  this->model = _model;

  std::cout << "SwarmRobotPlugin::Load()" << std::endl;

  // Read the robot address.
  if (!_sdf->HasElement("address"))
  {
    std::cerr << "Unable to find the [address] parameter" << std::endl;
    return;
  }

  this->address = _sdf->Get<std::string>("address");

  // Initialize transport.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->pub = this->node->Advertise<msgs::Datagram>("~/swarm/broker/incoming");
}

//////////////////////////////////////////////////
bool SwarmRobotPlugin::SendTo(const std::string &_data,
    const msgs::Socket &_socket) const
{
  msgs::Datagram msg;

  // ToDo: Include here the source address.
  msg.mutable_socket()->set_dst_address(_socket.dst_address());
  msg.mutable_socket()->set_port(_socket.port());

  // ToDo: Include here the neighbors list.

  msg.set_src_address(this->GetHost());

  msg.set_data(_data);

  std::cerr << "[" << this->GetHost() << "] Sending data to the broker"
            << std::endl;

  this->pub->Publish(msg);

  return true;
}

//////////////////////////////////////////////////
std::string SwarmRobotPlugin::GetHost() const
{
  return this->address;
}

//////////////////////////////////////////////////
void SwarmRobotPlugin::OnMsgReceived(ConstDatagramPtr &_msg)
{
  const std::string topic =
      "~/swarm/" + _msg->socket().dst_address() + "/" +
      std::to_string(_msg->socket().port());

  if (this->cb.find(topic) == this->cb.end())
  {
    std::cerr << "Address not found" << std::endl;
    return;
  }

  msgs::Socket socket;
  socket.set_dst_address(_msg->src_address());
  socket.set_port(_msg->socket().port());

  // ToDo: Check if the destination node is in the neighbor list of the source
  // node.

  std::cerr << "[" << this->GetHost()
            << "] SwarmRobotPlugin::New message received" << std::endl;

  // There's visibility between source and destination: run the user callback.
  auto &userCallback = this->cb[topic].cb;
  userCallback(socket, _msg->data());
}
