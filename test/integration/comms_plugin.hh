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

#ifndef __SWARM_COMMS_PLUGIN_HH__
#define __SWARM_COMMS_PLUGIN_HH__

#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <sdf/sdf.hh>
#include <gazebo/physics/physics.hh>
#include <swarm/RobotPlugin.hh>

namespace swarm
{
  /// \brief Class to test battery functionality
  class CommsPlugin : public swarm::RobotPlugin
  {
    /// \brief Class constructor.
    public: CommsPlugin();

    /// \brief Class destructor.
    public: virtual ~CommsPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(sdf::ElementPtr _sdf);

    // Documentation inherited.
    private: virtual void Update(const gazebo::common::UpdateInfo &_info);

    private: void OnDataReceived(const std::string &_srcAddress,
                                 const std::string &_data);

    private: int iterations = 0;

    private: int numUnicastSent = 0;
    private: int numMulticastSent = 0;
    private: int numBroadcastSent = 0;

    private: int numMsgsRecv = 0;

    /// \brief Minimum free-space distance (m) between two nodes to be
    /// neighbors. Set to <0 for no limit.
    private: double neighborDistanceMin = -1.0;

    /// \brief Maximum free-space distance (m) between two nodes to be
    /// neighbors. Set to <0 for no limit.
    private: double neighborDistanceMax = -1.0;

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
  };
}
#endif
