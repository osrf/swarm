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

/// \file SwarmBrokerPlugin.hh
/// \brief Internal broker to dispatch messages inside the Swarm project.

#ifndef __SWARM_BROKER_PLUGIN_HH__
#define __SWARM_BROKER_PLUGIN_HH__

#include <mutex>
#include <queue>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ignition/transport.hh>
#include <sdf/sdf.hh>
#include "msgs/datagram.pb.h"

namespace gazebo
{
  namespace swarm
  {
    /// \brief This is a world plugin designed to centralize all the messages
    /// sent by the members of the swarm. This plugin subscribes to the
    /// "/swarm/broker/incoming" topic, on which all the agents publish their
    /// messages. This plugin receives and queues the messages. The broker
    /// uses its Update() function to dispatch the queued messages.
    /// Dispatch of a message may directly deliver it to
    /// the destination/s node/s or it may forward the message to a
    /// network simulator (ns-3).
    class IGNITION_VISIBLE SwarmBrokerPlugin : public WorldPlugin
    {
      /// \brief Class constructor.
      public: SwarmBrokerPlugin() = default;

      /// \brief Class destructor.
      public: virtual ~SwarmBrokerPlugin() = default;

      // Documentation Inherited.
      public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

      /// \brief Update callback for the plugin.
      /// \param[in] _info Update information provided by the server.
      private: void Update(const common::UpdateInfo &_info);

      /// \brief Callback executed when a new message is received.
      /// \param[in] _topic Topic name associated to the new message received.
      /// \param[in] _msg The new message received.
      private: void OnMsgReceived(const std::string &_topic,
                                  const msgs::Datagram &_msg);

      /// \brief World pointer.
      private: physics::WorldPtr world;

      /// \brief SDF for this plugin.
      private: sdf::ElementPtr sdf;

      /// \brief Pointer to the update event connection.
      private: event::ConnectionPtr updateConnection;

      /// \brief Pointer to a node for communication.
      private: ignition::transport::Node node;

      /// \brief Queue to store the incoming messages received from the agents.
      private: std::queue<msgs::Datagram> incomingMsgs;

      /// \brief Mutex for protecting the queue.
      private: std::mutex mutex;
    };
  }
}

#endif
