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
#include "swarm/Broker.hh"

using namespace swarm;

//////////////////////////////////////////////////
Broker *Broker::Instance()
{
  static Broker instance;
  return &instance;
}

//////////////////////////////////////////////////
Broker::Broker()
{
}

//////////////////////////////////////////////////
bool Broker::Register(const std::string &_id, const BrokerClient *_client)
{
  if (this->clients.find(_id) != this->clients.end())
  {
    std::cerr << "Broker::Register() error: ID [" << _id << "] already exists"
              << std::endl;
    return false;
  }

  this->clients[_id] = _client;
  return true;
}
