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

/// \file CommsModel.hh
/// \brief Manages how the communication behave between the members of the team.

#ifndef __SWARM_COMMS_MODEL_HH__
#define __SWARM_COMMS_MODEL_HH__

#include <string>
#include <gazebo/common/Time.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>

#include "swarm/SwarmTypes.hh"

namespace swarm
{
  /// \brief Class used to store information about the communication model.
  class IGNITION_VISIBLE CommsModel
  {
    /// \brief Class constructor.
    ///
    /// \param[in] _swarm Pointer to the swarm.
    /// \param[in] _world Pointer to the Gazebo world.
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: CommsModel(SwarmMembershipPtr _swarm,
                       gazebo::physics::WorldPtr _world,
                       sdf::ElementPtr _sdf);

    /// \brief Class destructor.
    public: virtual ~CommsModel() = default;

    /// \brief Decide if each member of the swarm enters into a comms outage.
    public: void UpdateOutages();

    /// \brief Update the neighbors list of each member of the swarm.
    public: void UpdateNeighbors();

    /// \brief Update the neighbor list for a single robot and notifies the
    /// robot with the updated list.
    ///
    /// \param[in] _address Address of the robot to be updated.
    private: void UpdateNeighborList(const std::string &_address);

    /// \brief Get the number of walls between two points in the world.
    ///
    /// \param[in] _p1 A 3D point.
    /// \param[in] _p2 Another 3D point.
    /// \return Number of walls between the points.
    private: unsigned int NumWallsBetweenPoses(const gazebo::math::Pose& _p1,
                                               const gazebo::math::Pose& _p2);

    /// \brief Get the number of tree lines between two points in the world.
    ///
    /// \param[in] _p1 A 3D point.
    /// \param[in] _p2 Another 3D point.
    /// \return Number of tree lines between the points.
    private: unsigned int NumTreesBetweenPoses(const gazebo::math::Pose& _p1,
                                               const gazebo::math::Pose& _p2);

    /// \brief Check if a "comms_model" block exists in the SDF element of the
    /// plugin. If so, update the value of the default parameters with the one
    /// read from the world file.
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    private: void LoadParameters(sdf::ElementPtr _sdf);

    /// \brief Minimum free-space distance (m) between two nodes to be
    /// neighbors. Set to <0 for no limit.
    private: double neighborDistanceMin = -1.0;

    /// \brief Maximum free-space distance (m) between two nodes to be
    /// neighbors. Set to <0 for no limit.
    private: double neighborDistanceMax = -1.0;

    /// \brief Equivalent free space distance (m) that is "consumed" by an
    /// intervening solid obstacle (wall, terrain, etc.).
    /// Set to <0 for infinite penalty (i.e., one obstacle blocks neighbors).
    private: double neighborDistancePenaltyWall = 0.0;

    /// \brief Equivalent free space distance (m) that is "consumed" by an
    /// intervening non-solid obstacle (forest, etc.).
    /// Set to <0 for infinite penalty (i.e., one obstacle blocks neighbors).
    private: double neighborDistancePenaltyTree = 0.0;

    /// \brief Minimum free-space distance (m) between two nodes to communicate
    /// (must also be neighbors). Set to <0 for no limit.
    private: double commsDistanceMin = -1.0;

    /// \brief Maximum free-space distance (m) between two nodes to communicate
    /// (must also be neighbors). Set to <0 for no limit.
    private: double commsDistanceMax = -1.0;

    /// \brief Equivalent free space distance (m) that is "consumed" by an
    /// intervening solid obstacle (wall, terrain, etc.).
    /// Set to <0 for infinite penalty (i.e., one obstacle blocks comms).
    private: double commsDistancePenaltyWall = 0.0;

    /// \brief Equivalent free space distance (m) that is "consumed" by an
    /// intervening non-solid obstacle (forest, etc.).
    /// Set to <0 for infinite penalty (i.e., one obstacle blocks comms).
    private: double commsDistancePenaltyTree = 0.0;

    /// \brief Minimum probability of dropping a given packet.
    /// Used with uniform drop probability model.
    private: double commsDropProbabilityMin = 0.0;

    /// \brief Maximum probability of dropping a given packet.
    /// Used with uniform drop probability model.
    private: double commsDropProbabilityMax = 0.0;

    /// \brief Probability of going into a comms outage at each second.
    private: double commsOutageProbability = 0.0;

    /// \brief Minimum length of comms outage (secs). Set to <0 for no limit.
    /// Used with uniform outage duration probability model.
    private: double commsOutageDurationMin = -1.0;

    /// \brief Maximum length of comms outage (secs). Set to <0 for no limit.
    /// Used with uniform outage duration probability model.
    private: double commsOutageDurationMax = -1.0;

    /// \brief Pointer to the swarm.
    private: SwarmMembershipPtr swarm;

    /// \brief Pointer to the Gazebo world.
    private: gazebo::physics::WorldPtr world;

    /// \brief Keep track of update sim-time.
    private: gazebo::common::Time lastUpdateTime;

    // \brief Ray used to test for line of sight between vehicles.
    private: gazebo::physics::RayShapePtr ray;
  };
}  // namespace
#endif
