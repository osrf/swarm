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

#ifndef __SWARM_BOO_FINDER_PLUGIN_HH__
#define __SWARM_BOO_FINDER_PLUGIN_HH__

#include <string>
#include <gazebo/common/UpdateInfo.hh>
#include <sdf/sdf.hh>
#include <swarm/RobotPlugin.hh>

namespace swarm
{
  /// \brief Class to test the communication with the base of operations.
  class BooFinderPlugin : public swarm::RobotPlugin
  {
    /// \brief Class constructor.
    public: BooFinderPlugin();

    // Documentation inherited.
    public: virtual void Load(sdf::ElementPtr _sdf);

    // Documentation inherited.
    private: virtual void Update(const gazebo::common::UpdateInfo &_info);

    /// \brief Callback executed when a new message is received.
    /// \param[in] _srcAddress Source address of the message.
    /// \param[in] _dstAddress Destination address of the message.
    /// \param[in] _dstPort Destination port.
    /// \param[in] _data Message payload.
    private: void OnDataReceived(const std::string &_srcAddress,
                                 const std::string &_dstAddress,
                                 const uint32_t _dstPort,
                                 const std::string &_data);

    /// \brief Validate the ACK received.
    private: void ValidateACK();

    /// \brief Number of server iterations executed.
    private: int iterations = -1;

    /// \brief Every test in test/integration/boo.cc has a unique number and
    /// different expectations. We read this test number from the SDF to
    /// be able to know which test is executing.
    private: int testCase = -1;

    /// \brief Maximum time difference allowed (seconds) between the current
    /// time and the reported lost person messages to the BOO.
    /// This parameter is read from SDF.
    private: double maxDt = 5.0;

    /// \brief Last ACK received from the BOO.
    private: std::string ack = "";
  };
}
#endif
