/*
 *
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

/// \file LostPersonControllerPlugin.hh
/// \brief An example of a Gazebo plugin for controlling a lost person.

#ifndef __SWARM_LOST_PERSON_CONTROLLER_PLUGIN_HH__
#define __SWARM_LOST_PERSON_CONTROLLER_PLUGIN_HH__

#include <gazebo/common/UpdateInfo.hh>
#include <sdf/sdf.hh>
#include <swarm/LostPersonPlugin.hh>

namespace swarm
{
  /// \brief Example implementation of a lost person model
  class LostPersonControllerPlugin : public swarm::LostPersonPlugin
  {
    /// \brief Class constructor.
    public: LostPersonControllerPlugin();

    /// \brief Class destructor.
    public: virtual ~LostPersonControllerPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(sdf::ElementPtr _sdf);

    // Documentation inherited.
    private: virtual void Update(const gazebo::common::UpdateInfo &_info);

    // Speed of the lost person
    private: double speed;
  };
}
#endif
