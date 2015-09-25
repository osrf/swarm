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

#include <fstream>
#include <iostream>
#include <string>
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
}

//////////////////////////////////////////////////
Logger::~Logger()
{
  std::cout << "Logger destructor" << std::endl;
}

//////////////////////////////////////////////////
bool Logger::Register(const std::string _id, const Loggable *_client)
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
  // Fill the simulation time of the log entry.
  for (const auto &client : this->clients)
  {
    //std::cout << client.first << std::endl;
    msgs::LogEntry logEntryMsg;

    // Set the simulation time of the log entry.
    logEntryMsg.set_time(_simTime);

    client.second->OnLog(logEntryMsg);
    std::cout << logEntryMsg.DebugString() << std::endl;

    this->log[client.first] = logEntryMsg;
  }

  std::fstream output("/tmp/test.log", std::ios::out | std::ios::trunc | std::ios::binary);
  // Flush the log into disk.
  for (const auto &robotLog : this->log)
  {
    if (!robotLog.second.SerializeToOstream(&output))
    {
      std::cerr << "Failed to write log into disk." << std::endl;
      return;
    }
  }

}
