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
/// \brief Query log information from the RobotPlugins and the BrokerPlugin and
/// write it into disk.

#include <map>
#include <string>

#include "msgs/log_entry.pb.h"
#include "swarm/Helpers.hh"

#ifndef __SWARM_LOGGER_HH__
#define __SWARM_LOGGER_HH__

namespace swarm
{
  /// \brief ToDo
  class IGNITION_VISIBLE Loggable
  {
    // Gazebo plugins complain if we declare this method pure virtual.
    public: virtual bool OnLog(msgs::LogEntry &/*_logEntry*/) const
    {
      return false;
    };
  };

  /// \brief ToDo
  class IGNITION_VISIBLE Logger
  {
    /// \brief Logger is a singleton. This method gets the
    /// Logger instance shared between all the clients.
    /// \return Pointer to the current Logger instance.
    public: static Logger *GetInstance();

    /// \brief ToDo.
    public: void Update(const double _simTime);

    /// \brief ToDo.
    public: bool Register(const std::string &_id, const Loggable *_client);

    /// \brief Constructor.
    protected: Logger();

    /// \brief Destructor.
    protected: virtual ~Logger();

    /// \brief Full path to the log file on disk.
    private: std::string filename;

    /// \brief ToDo. The key is the ID of the client.
    private: std::map<std::string, const Loggable*> clients;

    /// \brief ToDo. The key is the ID of the client. The value is the last
    /// log entry.
    private: std::map<std::string, msgs::LogEntry> log;
  };
}  // namespace
#endif
