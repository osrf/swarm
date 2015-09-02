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

#include <mutex>
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
  /// \brief Class that drives the behavior of the base of operations (BOO).
  /// The BOO binds on its own address (kBoo) and port (kBooPort).
  /// It accepts messages of the format: <cmd> [args]
  ///
  /// List of supported commands:
  /// FOUND <x> <y> <z> <t> : Person found in [x,y,z] at time t, where:
  ///
  /// x: X coordinate (meters).
  /// y: Y coordinate (meters).
  /// z: Z coordinate (meters).
  /// t: Time when the person was seen (double).
  ///
  /// The BOO also verifies that the position reported by a user match the
  /// position of the lost person. If the person is found, a message of type
  /// PersonFound is published in the topic /swarm/found and the simulation is
  /// paused.
  class BooPlugin : public swarm::RobotPlugin
  {
    /// \brief Class constructor.
    public: BooPlugin();

    /// \brief Class destructor.
    public: virtual ~BooPlugin();

    /// \brief Callback executed at the end of each world update.
    protected: virtual void OnUpdateEnd();

    // Documentation inherited.
    private: virtual void Load(gazebo::physics::ModelPtr _model,
                               sdf::ElementPtr _sdf);

    /// \brief Callback executed when a new message is received.
    /// \param[in] _srcAddress Source address of the message.
    /// \param[in] _data Message payload.
    private: void OnDataReceived(const std::string &_srcAddress,
                                 const std::string &_data);

    /// \brief Error tolerance allowed between the reported position and the
    /// real position of the lost person. This is interpreted as an Euclidean
    /// distance (m).
    protected: const double kTolerance = 0.25;

    /// \brief True when the lost person has been found.
    protected: bool found = false;

    /// \brief Pointer to the lost person's model.
    private: gazebo::physics::ModelPtr lostPerson;

    /// \brief Location of the lost person in the previous iteration.
    /// Messages take one cycle to arrive to the destination, so when the BOO
    /// receives a message we have to checked with the lost person's position
    /// in the pevious cycle.
    private: ignition::math::Vector3d lostPersonPose;

    /// \brief Pointer to the OnUpdateEnd event connection.
    private: gazebo::event::ConnectionPtr updateEndConnection;

    // \brief Mutex to avoid race conditions.
    private: std::mutex mutex;
  };
}
#endif
