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

/// \file Boo.hh
/// \brief Plugin that drives the base of operations.

#ifndef __SWARM_BOO_PLUGIN_HH__
#define __SWARM_BOO_PLUGIN_HH__

#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include "swarm/RobotPlugin.hh"

namespace swarm
{
  /// \brief Class that shows a potential agent controller using the Swarm API
  class BooPlugin : public swarm::RobotPlugin
  {
    /// \brief Class constructor.
    public: BooPlugin();

    /// \brief Class destructor.
    public: virtual ~BooPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(gazebo::physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    /// \brief Callback executed at the end of each world update.
    private: virtual void OnUpdateEnd();

    /// \brief Callback executed when a new message is received.
    /// \param[in] _srcAddress Source address of the message.
    /// \param[in] _data Message payload.
    private: void OnDataReceived(const std::string &_srcAddress,
                                 const std::string &_data);

    /// \brief Pointer to the lost person's model.
    private: gazebo::physics::ModelPtr lostPerson;

    /// \brief Location of the lost person in the previous iteration.
    /// Messages take one cycle to arrive to the destination, so when the BOO
    /// receives a message we have to checked with the lost person's position
    /// in the pevious cycle.
    private: ignition::math::Vector3d prevLostPersonPose;

    /// \brief True when the lost person has been found.
    private: bool found = false;

    /// \brief Pointer to the OnUpdateEnd event connection.
    private: gazebo::event::ConnectionPtr updateEndConnection;
  };
}
#endif
