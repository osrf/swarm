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

/// \file Broker.hh
/// \brief

#include <map>
#include <queue>
#include <string>
#include <vector>
#include "msgs/datagram.pb.h"
#include "swarm/Helpers.hh"

#ifndef __SWARM_BROKER_HH__
#define __SWARM_BROKER_HH__

namespace swarm
{
  /// \brief
  class IGNITION_VISIBLE BrokerClient
  {
    /// \brief
    public: virtual void OnMsgReceived(const msgs::Datagram &/*_msg*/)
    {
    };

    // \brief
    public: virtual void OnNeighborsReceived(
                                       const std::vector<std::string> &/*_msg*/)
    {
    };
  };

  /// \brief
  class IGNITION_VISIBLE BrokerClientInfo
  {
    /// \brief
    public: std::string address;

    /// \brief.
    public: BrokerClient* client;
  };

  /// \brief
  class IGNITION_VISIBLE Broker
  {
    /// \brief
    /// \return Pointer to the current Broker instance.
    public: static Broker *Instance();

    /// \brief
    public: bool Bind(const std::string &_clientAddress,
                      BrokerClient *_client,
                      const std::string &_endpoint);

    /// \brief
    public: void Push(const msgs::Datagram _msg);

    /// \brief Register a new client for message handling.
    /// \param[in] _id Unique ID of the client.
    /// \param[in] _client Pointer to the client.
    public: bool Register(const std::string &_id,
                          BrokerClient *_client);

    /// \brief Constructor.
    protected: Broker();

    /// \brief Destructor.
    protected: virtual ~Broker() = default;

    /// \brief Queue to store the incoming messages received from the clients.
    public: std::queue<msgs::Datagram> incomingMsgs;

    /// \brief List of clients. The key is the ID of the client and the value
    /// is a pointer to each client.
    public: std::map<std::string, BrokerClient*> clients;

    /// \brief List of clients. The key is the ID of the client and the value
    /// is a pointer to each client.
    public: std::map<std::string, std::vector<BrokerClientInfo>> listeners;
  };
}  // namespace
#endif
