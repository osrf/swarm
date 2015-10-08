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
#include <string>
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
    public: virtual void OnMsgReceived(const std::string &/*_topic*/,
                                       const msgs::Datagram &/*_msg*/)
    {
    };
  };

  /// \brief
  class IGNITION_VISIBLE Broker
  {
    /// \brief
    /// \return Pointer to the current Broker instance.
    public: static Broker *Instance();

    /// \brief
    public: bool Register(const std::string &_id,
                          const BrokerClient *_client);

    /// \brief Constructor.
    protected: Broker();

    /// \brief Destructor.
    protected: virtual ~Broker() = default;

    /// \brief List of clients. The key is the ID of the client and the value
    /// is a pointer to each client.
    private: std::map<std::string, const BrokerClient*> clients;
  };
}  // namespace
#endif
