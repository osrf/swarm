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
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <swarm/LogParser.hh>
#include <swarm/msgs/log_entry.pb.h>

namespace po = boost::program_options;

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Introspect and manipulate a Swarm log file.\n\n"
            << " swarmlog [options]\n\n"
            << "Options:\n"
            << " -h, --help\tShow this help message.\n"
            << " -e, --echo\tOutput the content of a log file to screen.\n"
            << " -s, --step\tStep through the content of a log file.\n"
            << " -f, --file\tPath to a Swarm log file."
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

//////////////////////////////////////////////////
bool parseArguments(int argc, char **argv, po::variables_map &_vm)
{
  // Define and parse the program options.
  po::options_description desc("Options");
  desc.add_options()
    ("help,h" , "Show this help message.")
    ("echo,e" , "Output the content of a log file to screen.")
    ("step,s" , "Step through the content of a log file.")
    ("file,f" , po::value<std::string>()->required(),
         "Path to a Swarm log file.");

  try
  {
    po::store(po::command_line_parser(argc, argv).options(desc).run(), _vm);

    // We require to specify echo or step.
    if ((_vm.count("help")) || (!_vm.count("echo") && !_vm.count("step")))
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
