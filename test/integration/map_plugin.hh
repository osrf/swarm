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

#ifndef __SWARM_MAP_PLUGIN_HH__
#define __SWARM_MAP_PLUGIN_HH__

#include <gazebo/common/UpdateInfo.hh>
#include <sdf/sdf.hh>
#include <swarm/RobotPlugin.hh>

namespace swarm
{
  /// \brief Class to test the communication with the base of operations.
  class MapPlugin : public swarm::RobotPlugin
  {
    /// \brief Class constructor.
    public: MapPlugin();

    // Documentation inherited.
    public: virtual void Load(sdf::ElementPtr _sdf);

    // Documentation inherited.
    private: virtual void Update(const gazebo::common::UpdateInfo &_info);

    /// \brief Every test in test/integration/boo.cc has a unique number and
    /// different expectations. We read this test number from the SDF to
    /// be able to know which test is executing.
    private: int testCase = -1;

    private: int iteration = 0;
  };
}
#endif
