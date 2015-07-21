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
/// \brief Structures and functions for the SWARM API.

#ifndef __SWARM_BROKER_PLUGIN_HH__
#define __SWARM_BROKER_PLUGIN_HH__

#include <mutex>
#include <queue>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>
#include "msgs/datagram.pb.h"

namespace gazebo
{
  namespace swarm
  {
    /// \brief
    class IGNITION_VISIBLE SwarmBrokerPlugin : public WorldPlugin
    {
      /// \brief
      public: SwarmBrokerPlugin();

      /// \brief
      public: virtual ~SwarmBrokerPlugin();

      // Documentation Inherited.
      public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

      /// \brief Update the plugin.
      /// \param[in] _info Update information provided by the server.
      private: void Update(const common::UpdateInfo &_info);

      private: void OnMsgReceived(ConstDatagramPtr &_msg);

      /// \brief World pointer.
      private: physics::WorldPtr world;

      /// \brief SDF for this plugin.
      private: sdf::ElementPtr sdf;

      /// \brief Pointer to the update event connection.
      private: event::ConnectionPtr updateConnection;

      /// \brief Pointer to a node for communication.
      private: transport::NodePtr node;

      /// \brief
      private: transport::SubscriberPtr brokerSub;

      /// \brief
      private: std::queue<msgs::Datagram> incomingMsgs;

      /// \brief
      private: std::mutex mutex;
    };
  }
}

#endif
