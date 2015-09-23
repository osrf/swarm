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

/// \file TeamPlugin.hh
/// \brief An example of a Gazebo plugin for controlling a member of the swarm.

#ifndef __SWARM_TEAM_CONTROLLER_PLUGIN_HH__
#define __SWARM_TEAM_CONTROLLER_PLUGIN_HH__

#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <swarm/RobotPlugin.hh>

namespace swarm
{
  /// \brief Class that shows a potential agent controller using the Swarm API
  class TeamControllerPlugin : public swarm::RobotPlugin
  {
    /// \brief Class constructor.
    public: TeamControllerPlugin();

    /// \brief Class destructor.
    public: virtual ~TeamControllerPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(sdf::ElementPtr _sdf);

    // Documentation inherited.
    private: virtual void Update(const gazebo::common::UpdateInfo &_info);

    /// \brief Callback executed when a new message is received.
    /// \param[in] _srcAddress Source address of the message.
    /// \param[in] _dstAddress Destination address of the message.
    /// \param[in] _dstPort Destination port.
    /// \param[in] _data Message payload.
    private: void OnDataReceived(const std::string &_srcAddress,
                                 const std::string &_dstAddress,
                                 const uint32_t _dstPort,
                                 const std::string &_data);
  };
}

#endif