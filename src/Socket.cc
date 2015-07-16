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
#include "msgs/socket.pb.h"
#include "swarm/Socket.hh"

using namespace gazebo;
using namespace swarm;

const std::string Socket::Broadcast = "broadcast";

//////////////////////////////////////////////////
Socket::Socket(const std::string &_addr, const int _port)
{
  this->internalSocket = "";
  std::cout << "New socket [" << _addr << ":" << _port << "]" << std::endl;
}

//////////////////////////////////////////////////
bool Socket::SendTo(const std::string &/*_data*/) const
{
  std::cout << "Sending data" << std::endl;
  return true;
}