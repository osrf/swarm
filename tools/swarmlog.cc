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
#include <termios.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <boost/filesystem/operations.hpp>
#include <boost/program_options.hpp>
#include "swarm/LogParser.hh"
#include "msgs/log_entry.pb.h"
#include "msgs/log_header.pb.h"

namespace po = boost::program_options;

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Introspect and manipulate a Swarm log file.\n\n"
            << " swarmlog [options]\n\n"
            << "Options:\n"
            << " -h, --help             Show this help message.\n"
            << " -e, --echo             Output the content of a log file to"
            <<                          " screen.\n"
            << " -i, --info             Output information about a log file."
            <<                          " Log filename\n"
            << "                        should be specified using the --file "
            <<                          "option.\n"
            << " -s, --step             Step through the content of a log "
            <<                          "file.\n"
            << " -f, --file   <input>   Path to a Swarm log file.\n"
            << "     --filter <output>  Filter only broker and BOO entries."
            << std::endl;
}

/////////////////////////////////////////////////
int getChar()
{
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

/////////////////////////////////////////////////
std::string fileSizeStr(const boost::uintmax_t &_bytesize)
{
  std::ostringstream size;

  // Generate a human friendly string
  if (_bytesize < 1000)
    size << _bytesize << " B";
  else if (_bytesize < 1000000)
    size << _bytesize / 1.0e3 << " KB";
  else if (_bytesize < 1000000000)
    size << _bytesize / 1.0e6 << " MB";
  else
    size << _bytesize / 1.0e9 << " GB";

  return size.str();
}

//////////////////////////////////////////////////
bool parseArguments(int argc, char **argv, po::variables_map &_vm)
{
  // Define and parse the program options.
  po::options_description desc("Options");
  desc.add_options()
    ("help,h" , "Show this help message.")
    ("echo,e" , "Output the content of a log file to screen.")
    ("info,i" , "Output information about a log file. Log filename "
                "should be specified using the --file option.")
    ("step,s" , "Step through the content of a log file.")
    ("filter" , po::value<std::string>(),
         "Filter only broker and BOO entries.")
    ("file,f" , po::value<std::string>()->required(),
         "Path to a Swarm log file.");

  try
  {
    po::store(po::command_line_parser(argc, argv).options(desc).run(), _vm);

    // We require to specify echo or step.
    if ((_vm.count("help")) ||
        (!_vm.count("echo") && !_vm.count("info") && !_vm.count("step") &&
         !_vm.count("filter")))
      return false;

    po::notify(_vm);
  }
  catch(po::error& e)
  {
    std::cerr << "Error: " << e.what() << std::endl << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Parse command line arguments;
  po::variables_map vm;
  if (!parseArguments(argc, argv, vm))
  {
    usage();
    return 1;
  }

  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  std::string logfile = vm["file"].as<std::string>();
  swarm::LogParser parser(logfile);
  swarm::msgs::LogEntry logEntry;
  char c = '\0';

  if (vm.count("info"))
  {
    swarm::msgs::LogHeader header;
    if (!parser.Header(header))
    {
      std::cerr << "Error parsing header from [" << logfile << "]" << std::endl;
      return 1;
    }

    std::cout << "Swarm Version:         " << header.swarm_version() << std::endl;
    std::cout << "Gazebo Version:        " << header.gazebo_version() << std::endl;
    boost::filesystem::path p(logfile);
    std::cout << "Random Seed:           " << header.seed() << std::endl;
    if (header.has_num_ground_vehicles())
    {
      std::cout << "# ground vehicles:     "
                << header.num_ground_vehicles() << std::endl;
    }
    if (header.has_num_fixed_vehicles())
    {
      std::cout << "# fixed-wing vehicles: "
                << header.num_fixed_vehicles() << std::endl;
    }
    if (header.has_num_rotor_vehicles())
    {
      std::cout << "# rotorcraft vehicles: "
                << header.num_rotor_vehicles() << std::endl;
    }
    if (header.has_terrain_name())
    {
      std::cout << "Terrain name:          "
                << header.terrain_name() << std::endl;
    }
    if (header.has_vegetation_name())
    {
      std::cout << "Vegetation name:       "
                << header.vegetation_name() << std::endl;
    }
    if (header.has_search_area())
    {
      std::cout << "Search area:           "
                << header.search_area() << std::endl;
    }
    if (header.has_team_name())
    {
      std::cout << "Team:                  "
                << header.team_name() << std::endl;
    }
    if (header.has_max_time_allowed())
    {
      std::cout << "Max time allowed:      "
                << header.max_time_allowed() << std::endl;
    }
    if (header.has_max_wrong_reports())
    {
      std::cout << "Max wrong reports:     "
                << header.max_wrong_reports() << std::endl;
    }
    if (header.has_time_step())
    {
      std::cout << "Time step:             "
                << header.time_step() << std::endl;
    }

    auto fileSize = fileSizeStr(boost::filesystem::file_size(p));
    std::cout << "Size:                  " << fileSize << std::endl;
    std::cout << std::endl;
    return 0;
  }

  if (vm.count("filter"))
  {
    // Output file.
    std::string filteredFile = vm["filter"].as<std::string>();
    std::fstream output(filteredFile,
        std::ios::out | std::ios::trunc | std::ios::binary);

    swarm::msgs::LogHeader header;
    if (!parser.Header(header))
    {
      std::cerr << "Error parsing header from [" << logfile << "]" << std::endl;
      return 1;
    }

    // Write the length of the header to be serialized.
    int32_t headerSize = header.ByteSize();
    output.write(reinterpret_cast<char*>(&headerSize), sizeof(headerSize));

    if (!header.SerializeToOstream(&output))
    {
      std::cerr << "Failed to serialize to file." << std::endl;
      return -1;
    }

    while (parser.Next(logEntry) && c != 'q')
    {
      if (logEntry.id() == "broker" || logEntry.id() == "boo")
      {
        // Write the length of the message to be serialized.
        int32_t entrySize = logEntry.ByteSize();
        output.write(reinterpret_cast<char*>(&entrySize), sizeof(entrySize));

        if (!logEntry.SerializeToOstream(&output))
        {
          std::cerr << "Failed to serialize to file." << std::endl;
          return -1;
        }
      }
    }
    return 0;
  }

  while (parser.Next(logEntry) && c != 'q')
  {
    std::cout << logEntry.DebugString() << std::endl;
    if (vm.count("step"))
    {
      std::cout << "\n--- Press space to continue, 'q' to quit ---\n";

      c = '\0';

      // Wait for a space or 'q' key press
      while (c != ' ' && c != 'q')
        c = getChar();
    }
  }

  // Delete all global objects allocated by libprotobuf.
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
