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

#include <gazebo/gazebo_config.h>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include <ignition/math/Rand.hh>
#include "msgs/log_entry.pb.h"
#include "msgs/log_header.pb.h"
#include "swarm/config.hh"
#include "swarm/Logger.hh"

using namespace swarm;

//////////////////////////////////////////////////
Logger *Logger::Instance()
{
  static Logger instance;
  return &instance;
}

//////////////////////////////////////////////////
Logger::Logger()
{
  // Did the user set SWARM_LOG?
  char *logEnableEnv = std::getenv("SWARM_LOG");
  this->enabled = ((logEnableEnv) && (std::string(logEnableEnv) == "1"));
}

/////////////////////////////////////////////////
void Logger::CreateLogFile(sdf::ElementPtr _sdf)
{
  if (this->enabled)
  {
    this->enabled = true;

    gzmsg << "Logging enabled [" << this->logCompletePath.string()
          << "]" << std::endl;

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

    // Close an open stream
    if (this->output.is_open())
      this->output.close();

    // Create the log file.
    this->output.open(this->logCompletePath.string(),
      std::ios::out | std::ios::binary);

    // Fill the header.
    this->FillHeader(_sdf);

    // Write the length of the header to be serialized.
    int32_t size = this->header.ByteSize();
    this->output.write(reinterpret_cast<char*>(&size), sizeof(size));

    if (!this->header.SerializeToOstream(&this->output))
    {
      std::cerr << "Failed to write header into disk." << std::endl;
      return;
    }
    this->output.flush();
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
bool Logger::Unregister(const std::string &_id)
{
  if (this->clients.erase(_id) != 1)
  {
    std::cerr << "Logger::Unregister() error: ID [" << _id << "] doesn't exist"
              << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void Logger::Update(const double _simTime)
{
  if (!this->output.is_open() || !this->enabled)
    return;

  // Fill the simulation time of the log entry.
  for (const auto &kv : this->clients)
  {
    auto id = kv.first;
    auto client = kv.second;
    msgs::LogEntry logEntryMsg;

    // The logger sets some fields.
    logEntryMsg.set_id(id);
    logEntryMsg.set_time(_simTime);

    if (!client)
    {
      std::cerr << "Logger::Update() error: Client [" << id
                << "] has been destroyed. Removing client." << std::endl;
      this->log.erase(id);
      continue;
    }

    // The client sets some fields.
    client->OnLog(logEntryMsg);

    // We save the logEntry in memory.
    this->log[id] = logEntryMsg;
  }

  // Flush the log into disk.
  for (const auto &logPair : this->log)
  {
    // Write the length of the message to be serialized.
    int32_t size = logPair.second.ByteSize();
    this->output.write(reinterpret_cast<char*>(&size), sizeof(size));

    if (!logPair.second.SerializeToOstream(&this->output))
    {
      std::cerr << "Failed to write log into disk." << std::endl;
      return;
    }
  }

  this->output.flush();
}

/////////////////////////////////////////////////
void Logger::Reset()
{
}

/////////////////////////////////////////////////
void Logger::FillHeader(sdf::ElementPtr _sdf)
{
  this->header.set_swarm_version(SWARM_HASH_VERSION);
  this->header.set_gazebo_version(GAZEBO_VERSION_FULL);
  this->header.set_seed(ignition::math::Rand::Seed());

  if (_sdf && _sdf->HasElement("log_info"))
  {
    auto const &logElem = _sdf->GetElement("log_info");

    if (logElem->HasElement("num_ground_vehicles"))
    {
      this->header.set_num_ground_vehicles(
        logElem->Get<int>("num_ground_vehicles"));
    }
    if (logElem->HasElement("num_fixed_vehicles"))
    {
      this->header.set_num_fixed_vehicles(
        logElem->Get<int>("num_fixed_vehicles"));
    }
    if (logElem->HasElement("num_rotor_vehicles"))
    {
      this->header.set_num_rotor_vehicles(
        logElem->Get<int>("num_rotor_vehicles"));
    }
    if (logElem->HasElement("terrain_name"))
    {
      this->header.set_terrain_name(
        logElem->Get<std::string>("terrain_name"));
    }
    if (logElem->HasElement("vegetation_name"))
    {
      this->header.set_vegetation_name(
        logElem->Get<std::string>("vegetation_name"));
    }
    if (logElem->HasElement("search_area"))
    {
      this->header.set_search_area(
        logElem->Get<std::string>("search_area"));
    }
    if (logElem->HasElement("max_time_allowed"))
    {
      this->header.set_max_time_allowed(
        logElem->Get<double>("max_time_allowed"));
    }
    if (logElem->HasElement("max_wrong_reports"))
    {
      this->header.set_max_wrong_reports(
        logElem->Get<int>("max_wrong_reports"));
    }
  }

  // Did the user set SWARM_TEAMNAME?
  char *teamNameEnv = std::getenv("SWARM_TEAMNAME");
  if (teamNameEnv)
    this->header.set_team_name(std::string(teamNameEnv));
}
