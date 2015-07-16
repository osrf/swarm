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
#include "SwarmRobotPlugin.hh"
#include <swarm/msgs/socket.pb.h>

using namespace gazebo;
using namespace swarm;

//////////////////////////////////////////////////
SwarmRobotPlugin::SwarmRobotPlugin()
{
  // Create a unicast socket.
  this->socket = new swarm::Socket("192.1.68.1.2", 4000);
  this->socket->Bind(&SwarmRobotPlugin::OnDataReceived, this);

  // Create a broadcast socket.
  this->bcastSocket = new swarm::Socket(Socket::Broadcast, 4100);
}

//////////////////////////////////////////////////
SwarmRobotPlugin::~SwarmRobotPlugin()
{
  delete this->socket;
  delete this->bcastSocket;
}

//////////////////////////////////////////////////
void SwarmRobotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
}

//////////////////////////////////////////////////
std::string SwarmRobotPlugin::GetHost() const
{
  return "local IP";
}

//////////////////////////////////////////////////
void SwarmRobotPlugin::Update(const common::UpdateInfo &_info)
{
  // Send some data via the unicast socket.
  auto res = this->socket->SendTo("some data");

  // Send some data via the broadcast socket.
  res = this->bcastSocket->SendTo("more data");
}

//////////////////////////////////////////////////
void SwarmRobotPlugin::OnDataReceived(const msgs::Socket &_socket,
    const std::string &_data)
{
  std::cout << "\n---\n" << std::endl;
  std::cout << "\tFrom: [" << _socket.address() << "]" << std::endl;
  std::cout << "\tData: [" << _data << "]" << std::endl;
}
