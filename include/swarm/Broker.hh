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
  /// \brief Interface that classes should implement for using the broker.
  /// These methods will be automatically executed in the client for delivering
  /// new messages or new neighbors updates.
  class IGNITION_VISIBLE BrokerClient
  {
    /// \brief Executed when a new message is delivered to the client.
    /// \param[in] _msg New message.
    public: virtual void OnMsgReceived(const msgs::Datagram &_msg) const = 0;

    // \brief Executed when a new neighbor update is received in the client.
    public: virtual void OnNeighborsReceived(
      const std::vector<std::string> &_msg) = 0;
  };

  /// \brief Stores information about a client broker.
  class IGNITION_VISIBLE BrokerClientInfo
  {
    /// \brief Address of the client. E.g.: 192.168.2.2
    public: std::string address;

    /// \brief Pointer to the client.
    public: const BrokerClient* handler;
  };

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
    /// address. An endpoint is constructed as an address followed by ':',
    /// followed by the port. E.g.: "192.168.1.5:8000" is a valid endpoint.
    /// \param[in] _clientAddress Address of the broker client.
    /// \param[in] _client Pointer to the broker client.
    /// \param[in] _endpoint End point requested to bind.
    /// \return True if the operation succeed or false otherwise (if the client
    /// was already bound to the same endpoint).
    public: bool Bind(const std::string &_clientAddress,
                      const BrokerClient *_client,
                      const std::string &_endpoint);

    /// \brief Queue a new message.
    /// \param[in] _msg A new message.
    public: void Push(const msgs::Datagram _msg);

    /// \brief Register a new client for message handling.
    /// \param[in] _id Unique ID of the client.
    /// \param[in] _client Pointer to the client.
    /// \return True if the operation succeed of false otherwise (if the same
    /// id was already registered).
    public: bool Register(const std::string &_id,
                          BrokerClient *_client);

    /// \brief Get the list of registered clients.
    /// \return Map of registered clients. The key is the client ID and the
    /// value is a pointer to the client.
    public: const std::map<std::string, BrokerClient*> &Clients() const;

    /// \brief Get the list of endpoints bound.
    /// \return Map of endpoints. The key is the endpoint and the value is a
    /// vector containing the information of all the clients bound to this
    /// endpoint.
    /// \sa BrokerClientInfo.
    public: const std::map<std::string, std::vector<BrokerClientInfo>>
      &EndPoints() const;

    /// \brief Get the current message queue.
    /// \return Reference to the queue of messages.
    public: std::queue<msgs::Datagram> &Messages();

    /// \brief Unregister a client and unbind from all the endpoints.
    /// \param[in] _id Unique ID of the client.
    /// \return True if the operation succeed or false otherwise (if there is
    /// no client registered for this ID).
    public: bool Unregister(const std::string &_id);

    /// \brief Constructor.
    protected: Broker() = default;

    /// \brief Destructor.
    protected: virtual ~Broker() = default;

    /// \brief Queue to store the incoming messages received from the clients.
    protected: std::queue<msgs::Datagram> incomingMsgs;

    /// \brief List of clients. The key is the ID of the client and the value
    /// is a pointer to each client.
    protected: std::map<std::string, BrokerClient*> clients;

    /// \brief List of bound endpoints. The key is an endpoint and the
    /// value is the vector of clients bounded on that endpoint.
    protected: std::map<std::string, std::vector<BrokerClientInfo>> endpoints;
  };
}  // namespace
#endif
