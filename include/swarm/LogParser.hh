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
/// \brief Provide functions for parsing a Swarm log file.

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "msgs/log_entry.pb.h"

#ifndef __SWARM_LOGPARSER_HH__
#define __SWARM_LOGPARSER_HH__

namespace swarm
{
  /// \brief Parse a Swarm log file. Once you specify the full path to the log
  /// file in the constructor, you can call Next().
  /// Each Next() call returns the next LogEntry stored in the log file.
  /// When the log file reaches the end, Next() will return false.
  class IGNITION_VISIBLE LogParser
  {
    /// \brief Class constructor.
    /// \param[in] _filename Full path to the log file.
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

    /// \brief Get the next entry of the log.
    /// \param[out] _entry Next entry parsed from the log.
    /// \return True when the next entry has been succesfully parsed or false
    /// otherwise (e.g.: when there are no more entries in the log).
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

    /// \brief Destructor.
    public: virtual ~LogParser() = default;

    /// \brief Full path to the log.
    private: std::string filename;

    /// \brief True if the file is currently opened.
    private: bool isOpen;

    /// \brief Stream object to operate on a log file.
    private: std::fstream input;
  };
}  // namespace
#endif
