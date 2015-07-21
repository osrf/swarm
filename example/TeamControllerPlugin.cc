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
#include <gazebo/common/Events.hh>
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
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Init()
{
  // Bind on the default port.
  this->Bind(&TeamControllerPlugin::OnDataReceived, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&TeamControllerPlugin::Update, this, _1));
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Update(const common::UpdateInfo &_info)
{
  if (this->counter == 0)
  {
    // Create a unicast socket.
    msgs::Socket socket;
    if (this->GetHost() == "192.168.2.1")
      socket.set_dst_address("192.168.2.2");
    else if (this->GetHost() == "192.168.2.2")
      socket.set_dst_address("192.168.2.1");
    else
    {
      std::cout << this->GetHost() << ": Nothing to do" << std::endl;
      return;
    }

    std::cout << "[" << this->GetHost() <<
      "] TeamControllerPlugin::Update() Sending unicast data" << std::endl;
    // Send some data via the unicast socket.
    auto res = this->SendTo("some data on the unicast socket", socket);


    // Create a broadcast socket.
    std::cout << "[" << this->GetHost() <<
      "] TeamControllerPlugin::Update() Sending broadcast data" << std::endl;
    // Send some data via the broadcast socket.
    res = this->SendTo("more data on the broadcast socket");

    this->counter++;
  }
}

//////////////////////////////////////////////////
void TeamControllerPlugin::OnDataReceived(const msgs::Socket &_socket,
    const std::string &_data)
{
  std::cout << "\n---\n" << std::endl;
  std::cout << "\tNew Message [" << this->GetHost() << "]" << std::endl;
  std::cout << "\tFrom: [" << _socket.dst_address() << "]" << std::endl;
  std::cout << "\tData: [" << _data << "]" << std::endl;
}
