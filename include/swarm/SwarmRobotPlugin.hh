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

/// \file SwarmRobotPlugin.hh
/// \brief Structures and functions for the SWARM API.

#ifndef __SWARM_ROBOT_PLUGIN_HH__
#define __SWARM_ROBOT_PLUGIN_HH__

#include <functional>
#include <map>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>
#include "msgs/datagram.pb.h"
#include "msgs/socket.pb.h"

namespace gazebo
{
  namespace swarm
  {
    static const std::string kBroadcast;

    using Callback_t =
      std::function<void(const msgs::Socket &, const std::string &_data)>;

    /// \brief
    class IGNITION_VISIBLE NodeHandler
    {
      /// \brief
      public: NodeHandler() = default;

      /// \brief
      public: ~NodeHandler() = default;

      /// \brief
      public: Callback_t cb;

      /// \brief
      public: transport::SubscriberPtr socketSub;
    };

    /// \brief
    class IGNITION_VISIBLE SwarmRobotPlugin : public gazebo::ModelPlugin
    {
      /// \brief
      public: SwarmRobotPlugin() = default;

      /// \brief
      public: virtual ~SwarmRobotPlugin() = default;

      // Documentation Inherited.
      public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

      /// \brief
      public: template<typename C>
      bool Bind(const msgs::Socket &_socket,
                void(C::*_cb)(const msgs::Socket &_socket,
                              const std::string &_data),
                C *_obj)
      {
        // Sanity check: Only allow to bind on your local address.
        if (_socket.address() != this->address)
        {
          std::cerr << "Bind() error: Your local address is [" << this->address
                    << "] but you're trying to bind using ["
                    << _socket.address() << "]" << std::endl;
        }

        const std::string topic =
          "~/swarm/" + _socket.address() + "/" + std::to_string(_socket.port());

        NodeHandler nodeHandler;
        nodeHandler.cb = std::bind(_cb, _obj,
            std::placeholders::_1, std::placeholders::_2);
        nodeHandler.socketSub = this->node->Subscribe(topic,
            &SwarmRobotPlugin::OnMsgReceived, this);

        this->cb[topic] = nodeHandler;

        return true;
      }

      /// \brief
      public: bool SendTo(const msgs::Socket &_socket,
                          const std::string &_data) const;

      /// \brief
      public: std::string GetHost() const;

      /// \brief
      private: void OnMsgReceived(ConstDatagramPtr &_msg);

      /// \brief Node used for using Gazebo communications.
      private: transport::NodePtr node;

      /// \brief
      private: std::map<std::string, NodeHandler> cb;

      /// \brief Pointer to the model;
      private: physics::ModelPtr model;

      /// \brief
      private: std::string address;
    };
  }
}

#endif
