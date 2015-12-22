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
#include "msgs/log_header.pb.h"

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
    public: LogParser()
            : isOpen(false)
    {
    }

    /// \brief Class constructor.
    /// \param[in] _filename Full path to the log file.
    public: LogParser(const std::string &_filename)
        : isOpen(false)
    {
      this->Load(_filename);
    };

    /// \brief Load a log file
    public: bool Load(const std::string &_filename)
    {
      this->filename = _filename;
      this->input.open(this->filename, std::ios::in | std::ios::binary);

      if (!this->input)
      {
        std::cerr << this->filename << ": File not found" << std::endl;
        return false;
      }

      // Read the header size.
      int32_t size = 0;
      if (!this->input.read(reinterpret_cast<char*>(&size), sizeof(size)))
        return false;

      // Read the header.
      std::vector<char> buffer;
      buffer.resize(size);
      this->input.read(reinterpret_cast<char*>(&buffer[0]), size);
      if (!this->header.ParseFromArray(&buffer[0], size))
      {
        std::cerr << "Failed to parse header log file" << std::endl;
        return false;
      }

      this->isOpen = true;
      return true;
    }
    /// \brief Get the header of the log file.
    /// \param[out] _header Copy of the header.
    /// \return True if the operation succeed or false otherwise. E.g.: The
    /// file was not opened correctly.
    public: bool Header(msgs::LogHeader &_header) const
    {
      if (!this->isOpen)
      {
        std::cerr << "The log file has not been opened properly" << std::endl;
        return false;
      }
      _header.CopyFrom(this->header);
      return true;
    }

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

    /// \brief Log header.
    private: msgs::LogHeader header;
  };
}  // namespace
#endif
