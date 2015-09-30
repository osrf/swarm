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

/// \file Logger.hh
/// \brief Query log information from clients and create a log in disk.

#include <boost/filesystem.hpp>
#include <fstream>
#include <map>
#include <string>
#include "msgs/log_entry.pb.h"
#include "swarm/Helpers.hh"

#ifndef __SWARM_LOGGER_HH__
#define __SWARM_LOGGER_HH__

namespace swarm
{
  /// \brief Interface that a client must implement to be able to log data.
  class IGNITION_VISIBLE Loggable
  {
    /// \brief Collect the logging information from a client.
    /// \param[out] _logEntry Protobuf data structure that the client should
    /// partially fill.
    public: virtual void OnLog(msgs::LogEntry &/*_logEntry*/) const
    {
    };
  };

  /// \brief A logger is an object that stores a list of loggable clients.
  /// In each Update(), the logger collects the log information from each client
  /// and saves it into disk. We use a protobuf variable as the exchange unit
  /// between the log clients and the logger (see msgs/log_entry.proto).
  /// A "msgs::LogEntry" message contains the simulation time, the ID of the
  /// robot and all the information that we want to log for the given robot at
  /// this specific simulation time.
  /// E.g.: We want to log all the observed sensor measurements, the battery
  /// capacity, the actions sent to the robot and the messages sent/received.
  /// A client partially fills a LogEntry and the rest of the fields
  /// will be filled by other clients or the logger itself.
  /// E.g.: A "RobotPlugin" client can fill its sensor observations and the
  /// actions sent to a robot but doesn't have enough information to fill
  /// the communication information. The "BrokerPlugin" will fill that part of
  /// the LogEntry.
  ///
  /// Once the LogEntry is ready, the logger serialize the data and stores it
  /// into disk. The format of the log in disk is:
  /// <size0><log_entry0><size1><log_entry1>...<sizeN><log_entry>.
  ///
  /// Each time a log is saved into disk, it's stored under
  /// ~/.swarm/logs/<timstamp>/swarm.log
  ///
  /// It's possible to introspect a log file using the provided tool "swarmlog".
  /// \sa LogParser
  class IGNITION_VISIBLE Logger
  {
    /// \brief Logger is a singleton. This method gets the Logger instance
    /// shared between all the clients.
    /// \return Pointer to the current Logger instance.
    public: static Logger *GetInstance();

    /// \brief Get the full path of the log file.
    /// \return Full path to the log.
    public: std::string FilePath() const;

    /// \brief Collect a new round of log information from the clients.
    /// \param[in] _simTime Current simulation time.
    public: void Update(const double _simTime);

    /// \brief Register a new client for logging.
    /// \param[in] _id Unique ID of the client.
    /// \param[in] _client Pointer to the client.
    public: bool Register(const std::string &_id,
                          const Loggable *_client);

    /// \brief Constructor.
    protected: Logger();

    /// \brief Destructor.
    protected: virtual ~Logger() = default;

    /// \brief List of clients. The key is the ID of the client and the value
    /// is a pointer to each client.
    private: std::map<std::string, const Loggable*> clients;

    /// \brief Current list of log entries stored in memory. The key is the
    /// client ID and the value is the last logEntry stored for this client.
    private: std::map<std::string, msgs::LogEntry> log;

    /// \brief Stream object to operate on a log file.
    private: std::fstream output;

    /// \brief The complete pathname for the log file.
    private: boost::filesystem::path logCompletePath;

    /// \brief Whether the logging is enabled or not.
    private: bool enabled = false;
  };
}  // namespace
#endif
