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
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>
#include <swarm/msgs/socket.pb.h>
#include "TeamControllerPlugin.hh"


using namespace gazebo;
using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(TeamControllerPlugin)

//////////////////////////////////////////////////
TeamControllerPlugin::TeamControllerPlugin()
  : SwarmRobotPlugin()
{
}

//////////////////////////////////////////////////
TeamControllerPlugin::~TeamControllerPlugin()
{
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Call Load() on the base Swarm plugin.
  SwarmRobotPlugin::Load(_model, _sdf);

  // Create a unicast socket and bind our local address.
  this->socket.set_address(this->GetHost());
  this->socket.set_port(4000);
  this->Bind(this->socket, &TeamControllerPlugin::OnDataReceived, this);

  // Create a broadcast socket.
  this->bcastSocket.set_address(swarm::kBroadcast);
  this->bcastSocket.set_port(4100);
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Update(const common::UpdateInfo &_info)
{
  // Send some data via the unicast socket.
  auto res = this->SendTo(this->socket, "some data on the unicast socket" );

  // Send some data via the broadcast socket.
  res = this->SendTo(this->bcastSocket, "more data on the broadcast socket");
}

//////////////////////////////////////////////////
void TeamControllerPlugin::OnDataReceived(const msgs::Socket &_socket,
    const std::string &_data)
{
  std::cout << "\n---\n" << std::endl;
  std::cout << "\tFrom: [" << _socket.address() << "]" << std::endl;
  std::cout << "\tData: [" << _data << "]" << std::endl;
}
