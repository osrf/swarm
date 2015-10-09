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
#include "msgs/datagram.pb.h"
#include "swarm/Broker.hh"

using namespace swarm;

//////////////////////////////////////////////////
Broker *Broker::Instance()
{
  static Broker instance;
  return &instance;
}

//////////////////////////////////////////////////
bool Broker::Bind(const std::string &_clientAddress,
  BrokerClient *_client, const std::string &_endpoint)
{
  // Make sure that the same client didn't bind the same end point before.
  if (this->receivers.find(_endpoint) != this->receivers.end())
  {
    const auto &clientsV = this->receivers[_endpoint];
    for (const auto &client : clientsV)
    {
      if (client.address == _clientAddress)
      {
        std::cerr << "Broker::Bind() error: Address [" << _clientAddress
                  << "] already used in a previous Bind()" << std::endl;
        return false;
      }
    }
  }

  BrokerClientInfo clientInfo;
  clientInfo.address = _clientAddress;
  clientInfo.handler = _client;
  this->receivers[_endpoint].push_back(clientInfo);
  return true;
}

//////////////////////////////////////////////////
void Broker::Push(const msgs::Datagram _msg)
{
  // Queue the new message.
  this->incomingMsgs.push(_msg);
}

//////////////////////////////////////////////////
bool Broker::Register(const std::string &_id, BrokerClient *_client)
{
  if (this->clients.find(_id) != this->clients.end())
  {
    std::cerr << "Logger::Register() error: ID [" << _id << "] already exists"
              << std::endl;
    return false;
  }

  this->clients[_id] = _client;
  return true;
}

//////////////////////////////////////////////////
bool Broker::Unregister(const std::string &_id)
{
  if (this->clients.erase(_id) != 1)
  {
    std::cerr << "Broker::Unregister() error: ID [" << _id << "] doesn't exist"
              << std::endl;
    return false;
  }

  // Unbind.
  for (auto &receiversKv : this->receivers)
  {
    auto &v = receiversKv.second;

    auto i = std::begin(v);
    while (i != std::end(v))
    {
      if (i->address == _id)
        i = v.erase(i);
      else
        ++i;
    }
  }

  return true;
}
