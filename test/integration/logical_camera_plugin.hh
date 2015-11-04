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

#ifndef __SWARM_BATTERY_PLUGIN_HH__
#define __SWARM_BATTERY_PLUGIN_HH__

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <sdf/sdf.hh>
#include <gazebo/physics/physics.hh>
#include <swarm/RobotPlugin.hh>

namespace swarm
{
  /// \brief Class to test battery functionality
  class LogicalCameraPlugin : public swarm::RobotPlugin
  {
    /// \brief Class constructor.
    public: LogicalCameraPlugin();

    /// \brief Class destructor.
    public: virtual ~LogicalCameraPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(sdf::ElementPtr _sdf);

    // Documentation inherited.
    private: virtual void Update(const gazebo::common::UpdateInfo &_info);

    /// \brief Update for test case 0
    private: void Update0();

    /// \brief Update for test case 1
    private: void Update1();

    /// \brief Update for test case 2
    private: void Update2();

    /// \brief Pointer to the world
    private: gazebo::physics::WorldPtr world;

    /// \brief The current test case
    private: int testCase = 0;
  };
}
#endif
