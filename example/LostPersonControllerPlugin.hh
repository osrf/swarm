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

#include <gazebo/common/Time.hh>
#include <ignition/math/Vector3.hh>
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

    /// \brief Handle reset
    protected: virtual void Reset();

    // Documentation inherited.
    private: virtual void Update(const gazebo::common::UpdateInfo &_info);

    /// \brief Velocity scaling factor.
    public: double velocityFactor = 1.0;

    /// \brief Time between recomputing a new velocity vector.
    public: gazebo::common::Time updatePeriod = 5.0;

    /// \brief Time the of the last update.
    public: gazebo::common::Time prevUpdate = 0.0;

    /// \brief Velocity to apply.
    public: ignition::math::Vector3d velocity;
  };
}
#endif
