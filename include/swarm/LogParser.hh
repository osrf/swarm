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

/// \file LogParser.hh
/// \brief Provide functions for parsing a log file.

#include <cstdint>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "msgs/log_entry.pb.h"

#ifndef __SWARM_LOGPARSER_HH__
#define __SWARM_LOGPARSER_HH__

namespace swarm
{
  /// \brief ToDo
  class IGNITION_VISIBLE LogParser
  {
    /// \brief Todo.
    public: LogParser(const std::string &_filename)
      : filename(_filename),
        isOpen(false),
        input(filename, std::ios::in | std::ios::binary)
    {
      if (!this->input)
      {
        std::cerr << this->filename << ": File not found" << std::endl;
        return;
      }

      this->isOpen = true;
    };

    /// \brief ToDo.
    public: bool Next(msgs::LogEntry &_entry)
    {
      if (!this->isOpen)
      {
        std::cerr << "LogParser::Next() error: File [" << this->filename
                  << "] is not open" << std::endl;
        return false;
      }

      // Read the size of the next protobuf message.
      int32_t size = 0;
      if (!this->input.read(reinterpret_cast<char*>(&size), sizeof(size)))
        return false;

      std::vector<char> buffer;
      buffer.resize(size);
      this->input.read(reinterpret_cast<char*>(&buffer[0]), size);
      if (!_entry.ParseFromArray(&buffer[0], size))
      {
        std::cerr << "Failed to parse log file" << std::endl;
        return false;
      }
      return true;
    }

    /// \brief ToDo.
    public: virtual ~LogParser() = default;

    /// \brief ToDo.
    private: std::string filename;

    /// \brief ToDo.
    private: bool isOpen;

    /// \brief ToDo.
    private: std::fstream input;
  };
}  // namespace
#endif
