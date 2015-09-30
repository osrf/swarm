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

#include <cstdint>
#include <cstdlib>  // getenv
#include <iostream>
#include <string>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include "msgs/log_entry.pb.h"
#include "swarm/Logger.hh"

using namespace swarm;

//////////////////////////////////////////////////
Logger *Logger::GetInstance()
{
  static Logger instance;
  return &instance;
}

//////////////////////////////////////////////////
Logger::Logger()
{
  // Did the user set SWARM_LOG?
  char *logEnableEnv = std::getenv("SWARM_LOG");
  if ((logEnableEnv) && (std::string(logEnableEnv) == "1"))
  {
    this->enabled = true;

    // The base pathname for all the logs.
    const char *homePath = gazebo::common::getEnv("HOME");
    boost::filesystem::path logBasePath = boost::filesystem::path(homePath);
    logBasePath /= "/.swarm/log/";

    std::string logTimeDir = gazebo::common::Time::GetWallTimeAsISOString();
    this->logCompletePath = logBasePath / logTimeDir;

    // Create the log directory if necessary
    if (!boost::filesystem::exists(this->logCompletePath))
      boost::filesystem::create_directories(this->logCompletePath);

    this->logCompletePath = logBasePath / logTimeDir / "swarm.log";

    // Create the log file.
    this->output.open(this->logCompletePath.string(),
      std::ios::out | std::ios::binary);

    gzmsg << "Logging enabled [" << this->logCompletePath.string()
          << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
std::string Logger::FilePath() const
{
  return this->logCompletePath.string();
}

//////////////////////////////////////////////////
bool Logger::Register(const std::string &_id, const Loggable *_client)
{
  if (this->clients.find(_id) != this->clients.end())
  {
    std::cerr << "Logger::Register() error: ID [" << _id << "] already exists"
              << std::endl;
    return false;
  }

  this->clients[_id] = _client;
  return true;
}

//////////////////////////////////////////////////
void Logger::Update(const double _simTime)
{
  if (!this->output.is_open() || !this->enabled)
    return;

  // Fill the simulation time of the log entry.
  for (const auto &client : this->clients)
  {
    msgs::LogEntry logEntryMsg;

    // The logger sets some fields.
    logEntryMsg.set_id(client.first);
    logEntryMsg.set_time(_simTime);

    if (!client.second)
    {
      std::cerr << "Logger::Update() error: Client [" << client.first
                << "] has been destroyed. Removing client." << std::endl;
      this->log.erase(client.first);
      continue;
    }

    // The client sets some fields.
    client.second->OnLog(logEntryMsg);

    // We save the logEntry in memory.
    // ToDo: Allow adding information to an existing entry without replacing it.
    this->log[client.first] = logEntryMsg;
  }

  // Flush the log into disk.
  for (const auto &logPair : this->log)
  {
    // Write the length of the message to be serialized.
    int32_t size = logPair.second.ByteSize();
    output.write(reinterpret_cast<char*>(&size), sizeof(size));

    if (!logPair.second.SerializeToOstream(&output))
    {
      std::cerr << "Failed to write log into disk." << std::endl;
      return;
    }
  }

  output.flush();
}
