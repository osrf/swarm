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

    /// \brief Callback executed when a new message is received.
    /// \param[in] _srcAddress Source address of the message.
    /// \param[in] _data Message payload.
    private: void OnDataReceived(const std::string &_srcAddress,
                                 const std::string &_data);

    // Number of server iterations executed.
    private: int iterations = 0;

    // Number of unicast messages sent.
    private: int numUnicastSent = 0;

    // Number of multicast messages sent.
    private: int numMulticastSent = 0;

    // Number of broadcast messages sent.
    private: int numBroadcastSent = 0;

    // Number of messages received.
    private: int numMsgsRecv = 0;

    // Every test in test/integration/test.cc has a unique number and
    // different expectations. We read this test number from the SDF to
    // be able to know which test is executing.
    private: int testCase = -1;
  };
}
#endif
