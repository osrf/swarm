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
/// \brief An example of a Gazebo plugin for controlling a member of the swarm.

#ifndef __SWARM_TEAM_CONTROLLER_PLUGIN_HH__
#define __SWARM_TEAM_CONTROLLER_PLUGIN_HH__

#include <memory>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <swarm/SwarmRobotPlugin.hh>
#include <swarm/msgs/socket.pb.h>

namespace gazebo
{
  namespace swarm
  {
    /// \brief
    class TeamControllerPlugin : public gazebo::swarm::SwarmRobotPlugin
    {
      /// \brief
      public: TeamControllerPlugin();

      /// \brief
      public: virtual ~TeamControllerPlugin();

      // Documentation Inherited.
      public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

      /// \brief Update the robot controller.
      /// \param[in] _info Update information provided by the server.
      private: void Update(const common::UpdateInfo &_info);

      /// \brief
      private: void OnDataReceived(const msgs::Socket &_socket,
                                   const std::string &_data);

      /// \brief Pointer to the update event connection.
      private: event::ConnectionPtr updateConnection;

      /// \brief Pointer to the model;
      private: physics::ModelPtr model;

      /// \brief
      private: msgs::Socket socket;

      /// \brief
      private: msgs::Socket bcastSocket;
    };
  }
}

#endif