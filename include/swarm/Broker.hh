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
/// \brief Broker for handling message delivery among robots.

#include <cstdint>
#include <map>
#include <deque>
#include <vector>
#include "msgs/datagram.pb.h"
#include "swarm/Helpers.hh"

#ifndef __SWARM_BROKER_HH__
#define __SWARM_BROKER_HH__

namespace swarm
{
  class RobotPlugin;

  /// \brief Stores information about a client broker.
  class IGNITION_VISIBLE BrokerClientInfo
  {
    /// \brief Address of the client. E.g.: 1
    public: uint32_t address;

    /// \brief Pointer to the client.
    public: const RobotPlugin *handler;
  };

  /// \def EndPoints_M
  /// \brief Map of endpoints
  using EndPoints_M = std::map<uint32_t, std::vector<BrokerClientInfo>>;

  /// \brief Store messages, and exposes an API for registering new clients,
  /// bind to a particular address, push new messages or get the list of
  /// messages already stored in the queue.
  class IGNITION_VISIBLE Broker
  {
    /// \brief Broker is a singleton. This method gets the Broker instance
    /// shared between all the clients.
    /// \return Pointer to the current Broker instance.
    public: static Broker *Instance();

    /// \brief This method associates an endpoint with a broker client and its
    /// address. An endpoint is constructed as a numeric address.
    /// \param[in] _clientAddress Address of the broker client.
    /// \param[in] _client Pointer to the robot plugin.
    /// \param[in] _endpoint End point requested to bind.
    /// \return True if the operation succeed or false otherwise (if the client
    /// was already bound to the same endpoint).
    public: bool Bind(const uint32_t _clientAddress,
                      const RobotPlugin *_client,
                      const uint32_t _endpoint);

    /// \brief Queue a new message.
    /// \param[in] _msg A new message.
    public: void Push(const msgs::Datagram &_msg);

    /// \brief Register a new client for message handling.
    /// \param[in] _id Unique ID of the client.
    /// \param[in] _client Pointer to the robot plugin.
    /// \return True if the operation succeed of false otherwise (if the same
    /// id was already registered).
    public: bool Register(const uint32_t _id,
                          RobotPlugin *_client);

    /// \brief Get the list of registered clients.
    /// \return Map of registered clients. The key is the client ID and the
    /// value is a pointer to the client.
    public: const std::map<uint32_t, RobotPlugin*> &Clients() const;

    /// \brief Get the list of endpoints bound.
    /// \return Map of endpoints. The key is the endpoint and the value is a
    /// vector containing the information of all the clients bound to this
    /// endpoint.
    /// \sa BrokerClientInfo.
    public: const EndPoints_M &EndPoints() const;

    /// \brief Get the current message queue.
    /// \return Reference to the queue of messages.
    public: std::deque<msgs::Datagram> &Messages();

    /// \brief Unregister a client and unbind from all the endpoints.
    /// \param[in] _id Unique ID of the client.
    /// \return True if the operation succeed or false otherwise (if there is
    /// no client registered for this ID).
    public: bool Unregister(const uint32_t _id);

    /// \brief Handle reset.
    public: void Reset();

    /// \brief Constructor.
    protected: Broker() = default;

    /// \brief Destructor.
    protected: virtual ~Broker() = default;

    /// \brief Queue to store the incoming messages received from the clients.
    protected: std::deque<msgs::Datagram> incomingMsgs;

    /// \brief List of clients. The key is the ID of the client and the value
    /// is a pointer to each client.
    protected: std::map<uint32_t, RobotPlugin*> clients;

    /// \brief List of bound endpoints. The key is an endpoint and the
    /// value is the vector of clients bounded on that endpoint.
    protected: EndPoints_M endpoints;
  };
}  // namespace
#endif
