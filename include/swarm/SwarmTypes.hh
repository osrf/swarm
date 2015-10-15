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

#ifndef __SWARM_TYPES_HH__
#define __SWARM_TYPES_HH__

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <gazebo/common/Time.hh>

namespace swarm
{
  /// \brief Class used to store information about a member of the Swarm.
  class IGNITION_VISIBLE SwarmMember
  {
    /// \brief Gazebo name used for this model.
    public: std::string name;

    /// \brief Address of the robot. E.g.: 192.168.1.2
    public: std::string address;

    /// \brief Model pointer.
    public: gazebo::physics::ModelPtr model;

    /// \brief List of neighbors and comms probabilities for this robot.
    public: std::map<std::string, double> neighbors;

    /// \brief Is this robot on outage?
    public: bool onOutage;

    /// \brief When will the last outage finish?
    public: gazebo::common::Time onOutageUntil;

    /// \brief Current data rate usage (bits).
    public: uint32_t dataRateUsage;
  };

  /// \def SwarmMembership_M
  /// \brief Map containing information about the members of the swarm.
  /// The key is the robot address. The value is a pointer to a SwarmMember
  /// object that contains multiple information about the robot.
  using SwarmMembership_M = std::map<std::string, std::shared_ptr<SwarmMember>>;

  /// \def SwarmMembershipPtr
  /// \brief A shared pointer to the membership data structure.
  /// \sa SwarmMembership_M
  using SwarmMembershipPtr = std::shared_ptr<SwarmMembership_M>;
}
#endif
