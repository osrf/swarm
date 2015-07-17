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

const std::string kBroadcast = "broadcast";

//////////////////////////////////////////////////
void SwarmRobotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "SwarmRobotPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "SwarmRobotPlugin _sdf pointer is NULL");
  this->model = _model;

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
}

//////////////////////////////////////////////////
bool SwarmRobotPlugin::SendTo(const msgs::Socket &/*_socket*/,
    const std::string &/*_data*/) const
{
  std::cout << "Sending data" << std::endl;
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
  if (this->cb.find(_msg->socket().address()) == this->cb.end())
  {
    std::cerr << "Address not found" << std::endl;
    return;
  }

  // ToDo: Check if the destination node is in the neighbor list of the source
  // node.

  // There's visibility between source and destination: run the user callback.
  auto &userCallback = this->cb[_msg->socket().address()].cb;
  userCallback(_msg->socket(), _msg->data());
}
