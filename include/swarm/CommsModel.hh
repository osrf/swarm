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
/// \brief

#ifndef __SWARM_COMMS_MODEL_HH__
#define __SWARM_COMMS_MODEL_HH__

#include <string>
#include <gazebo/common/Time.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

#include "swarm/SwarmTypes.hh"

namespace swarm
{
  /// \brief Class used to store information about the communication model.
  class IGNITION_VISIBLE CommsModel
  {
    /// \brief Class constructor.
    public: CommsModel(SwarmMembershipPtr _swarm,
                       gazebo::physics::WorldPtr _world,
                       sdf::ElementPtr _sdf);

    /// \brief Class destructor.
    public: virtual ~CommsModel() = default;

    /// \brief For each member of the team, decide its outage state.
    public: void UpdateOutages();

    /// \brief ToDo.
    public: void UpdateNeighbors();

    /// \brief ToDo.
    private: void LoadParameters(sdf::ElementPtr _sdf);

    /// \brief Update the neighbor list for a single robot and notifies the
    /// robot with the updated list.
    ///
    /// \param[in] _address Address of the robot to be updated.
    private: void UpdateNeighborList(const std::string &_address);

    /// \brief ToDo.
    private: unsigned int NumWallsBetweenPoses(const gazebo::math::Pose& _p1,
                                               const gazebo::math::Pose& _p2);

    /// \brief ToDo.
    private: unsigned int NumTreesBetweenPoses(const gazebo::math::Pose& _p1,
                                               const gazebo::math::Pose& _p2);

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
    /// (must also be neighbors).  Set to <0 for no limit.
    private: double commsDistanceMin = -1.0;

    /// \brief Maximum free-space distance (m) between two nodes to communicate
    /// (must also be neighbors).  Set to <0 for no limit.
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

    /// \brief ToDo
    private: SwarmMembershipPtr swarm;

    /// \brief ToDo
    private: gazebo::physics::WorldPtr world;

    /// \brief Keep track of update sim-time.
    private: gazebo::common::Time lastUpdateTime;
  };
}  // namespace
#endif