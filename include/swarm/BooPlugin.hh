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

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include "msgs/log_entry.pb.h"
#include "swarm/Logger.hh"
#include "swarm/RobotPlugin.hh"
#include "swarm/SwarmTypes.hh"

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
  /// E.g.: FOUND 100.0 50.0 1.0 10.4
  ///
  /// The BOO also verifies that the position reported by a user match the
  /// position of the lost person. If the person is found, a message of type
  /// PersonFound is published in the topic /swarm/found and the simulation is
  /// paused.
  class BooPlugin : public RobotPlugin
  {
    /// \brief Class constructor.
    public: BooPlugin();

    /// \brief Class destructor.
    public: virtual ~BooPlugin();

    /// \brief Callback executed when a new message is received, and the
    /// message is not a FOUND message.
    /// \param[in] _srcAddress Source address of the message.
    /// \param[in] _dstAddress Destination address of the message.
    /// \param[in] _dstPort Destination port.
    /// \param[in] _data Message payload.
    public: virtual void OnData(const std::string &_srcAddress,
                                const std::string &_dstAddress,
                                const uint32_t _dstPort,
                                const std::string &_data);

    /// \brief Call this function to indicate that the lost person was found
    /// at a particular location and time.
    /// \param[in] _pos Position of the lost person.
    /// \param[in] _time Time associated with the _pos.
    /// \return True if the location and time are correct.
    public: bool Found(const ignition::math::Vector3d &_pos,
                       const double _time);

    /// \brief Send an ACK message to the vehicle after processing a request.
    /// The format of the message is: ACK <code>
    /// \param[in] _dstAddress Destination address.
    /// \param[in] _code Reply code:
    ///   0. Found. The person has been found on the reported position.
    ///   1. Not found. Incorrect position.
    ///   2. Not found. The reported time is too old.
    ///   3. Error: Unable to parse request.
    ///   4. Error: Incorrect number of arguments in request.
    ///   5. Error: Parsing arguments.
    ///   6. Error: Negative time reported.
    ///   7. Error: Future time reported.
    ///   8. Error: Unrecognized command.
    protected: void SendAck(const std::string &_dstAddress,
                            const int _code);

    /// \brief Handle reset
    protected: virtual void Reset();

    /// \brief Function that indicates the lost person was found
    /// at a particular location and time.
    /// \param[in] _pos Position of the lost person.
    /// \param[in] _time Time associated with the _pos.
    /// \param[in] _srcAddress Address which should receive acknowledgments.
    /// This is used by the OnDataReceived function.
    /// \return True if the location and time are correct.
    private: bool FoundHelper(const ignition::math::Vector3d &_pos,
                              const gazebo::common::Time &_time,
                              const std::string &_srcAddress="");

    /// \brief Callback executed at the end of each world update.
    private: virtual void OnUpdateEnd();

    // Documentation inherited.
    private: virtual void Load(gazebo::physics::ModelPtr _model,
                               sdf::ElementPtr _sdf);

    /// \brief Callback executed when a new message is received.
    /// \param[in] _srcAddress Source address of the message.
    /// \param[in] _dstAddress Destination address of the message.
    /// \param[in] _dstPort Destination port.
    /// \param[in] _data Message payload.
    private: void OnDataReceived(const std::string &_srcAddress,
                                 const std::string &_dstAddress,
                                 const uint32_t _dstPort,
                                 const std::string &_data);

    /// \brief Converts from a world position to a cell in the 3D grid.
    /// \param[in] _pos Position in world coordinates.
    /// \return The coordinates of a cell in the 3D grid.
    private: ignition::math::Vector3i PosToGrid(ignition::math::Vector3d _pos);

    // Documentation inherited.
    private: virtual void OnLog(msgs::LogEntry &_logEntry) const;

    /// \brief True when the lost person has been found.
    private: bool found = false;

    /// \brief Pointer to the lost person's model.
    private: gazebo::physics::ModelPtr lostPerson;

    /// \brief Buffer of registered lost person's positions.
    /// The key is the time at which the lost person changed position.
    /// The value contains the coordinates of a cell in a 3D grid.
    /// E.g.: {
    ///         {time 0.0, 10, 0, 0 },  => At t=0.0, the person was at [10,0,0]
    ///         {time 1.0, 20, 20, 0}   => At t=1.0, the person was at [20,20,0]
    ///       }
    private: std::map<gazebo::common::Time,
                      ignition::math::Vector3i> lostPersonBuffer;

    /// \brief Last known location of the lost person.
    private: ignition::math::Vector3i lastPersonPosInGrid;

    /// \brief Pointer to the OnUpdateEnd event connection.
    private: gazebo::event::ConnectionPtr updateEndConnection;

    // \brief Mutex to avoid race conditions.
    private: std::mutex mutex;

    /// \brief We discretize the world in a 3D grid of square cells.
    /// This constant expresses the size of each cell (m). We update the cell
    /// in which the lost person is located every cycle. Each reported position
    /// is discretized to a cell too. We consider success when both cells match.
    private: double cellSize = 10;

    /// \brief Maximum time difference allowed (seconds) between the current
    /// time and the reported lost person messages to the BOO.
    private: gazebo::common::Time maxDt = 1000.0;

    /// \brief Logger instance.
    private: Logger *logger = Logger::Instance();

    /// \brief Last report received
    private: mutable std::vector<msgs::BooReport> lastReports;
  };
}
#endif
