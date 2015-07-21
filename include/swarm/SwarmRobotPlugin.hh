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
/// \brief Structures and functions for the Swarm API.

#ifndef __SWARM_ROBOT_PLUGIN_HH__
#define __SWARM_ROBOT_PLUGIN_HH__

#include <functional>
#include <map>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ignition/transport.hh>
#include <sdf/sdf.hh>
#include "msgs/datagram.pb.h"
#include "msgs/socket.pb.h"

namespace gazebo
{
  namespace swarm
  {
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
      bool Bind(void(C::*_cb)(const msgs::Socket &_socket,
                              const std::string &_data),
                C *_obj,
                const int _port = 4100)
      {
        const std::string topic =
          "/swarm/" + this->GetHost() + "/" + std::to_string(_port);

        const std::string bcastTopic =
          "/swarm/broadcast/" + std::to_string(_port);

        std::cout << "[" << this->GetHost() << "] Bind to ["
                  << topic << "]" << std::endl;

        std::cout << "[" << this->GetHost() << "] Bind to ["
                  << bcastTopic << "]" << std::endl;

        this->node.Subscribe(topic, &SwarmRobotPlugin::OnMsgReceived, this);
        this->cb[topic] = std::bind(_cb, _obj,
            std::placeholders::_1, std::placeholders::_2);

        this->node.Subscribe(bcastTopic,
            &SwarmRobotPlugin::OnMsgReceived, this);
        this->cb[bcastTopic] = std::bind(_cb, _obj,
            std::placeholders::_1, std::placeholders::_2);

        return true;
      }

      /// \brief
      public: bool SendTo(const std::string &_data,
                          const msgs::Socket &_socket = msgs::Socket());

      /// \brief
      public: std::string GetHost() const;

      /// \brief
      private: void OnMsgReceived(const std::string &_topic,
                                  const msgs::Datagram &_msg);

      using Callback_t =
      std::function<void(const msgs::Socket &, const std::string &_data)>;

      protected: const std::string kBroadcast = "broadcast";

      /// \brief
      public: ignition::transport::Node node;

      /// \brief
      private: std::map<std::string, Callback_t> cb;

      /// \brief Pointer to the model;
      private: physics::ModelPtr model;

      /// \brief
      private: std::string address;
    };
  }
}

#endif
