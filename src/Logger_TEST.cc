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

#include <stdlib.h>  // setenv
#include "gtest/gtest.h"
#include "msgs/log_entry.pb.h"
#include "swarm/Logger.hh"
#include "swarm/LogParser.hh"

using namespace swarm;

//////////////////////////////////////////////////
/// \brief A sample loggable class.
class LogClient : public swarm::Loggable
{
  /// \brief Constructor.
  /// \param[in] _id The client ID.
  public: LogClient(const std::string &_id)
    : id(_id)
  {
  }

  // Documentation inherited.
  void OnLog(msgs::LogEntry &_logEntry)
  {
    // We fill the ID field.
    _logEntry.set_id(this->id);
  }

  /// \brief A client ID.
  public: std::string id;
};

//////////////////////////////////////////////////
/// \brief Create a log based on data from two clients and parse its content.
TEST(LoggerTest, Log)
{
  Logger *logger = Logger::GetInstance();

  // Client #1.
  Logger *logger1 = Logger::GetInstance();
  LogClient client1("#1");
  EXPECT_TRUE(logger1->Register(client1.id, &client1));

  // Try to register an existing client.
  EXPECT_FALSE(logger1->Register(client1.id, &client1));

  // Client #2.
  Logger *logger2 = Logger::GetInstance();
  LogClient client2("#2");
  EXPECT_TRUE(logger2->Register(client2.id, &client2));

  // Generate some log data.
  msgs::LogEntry logEntry;
  logger->Update(1.0);

  // Check that a log file has been created.
  auto filePath = logger->FilePath();
  EXPECT_TRUE(boost::filesystem::exists(filePath));

  // Parse the log.
  logEntry.Clear();
  LogParser logParser(filePath);
  EXPECT_TRUE(logParser.Next(logEntry));
  EXPECT_EQ(logEntry.id(), client1.id);
  logEntry.Clear();
  EXPECT_TRUE(logParser.Next(logEntry));
  EXPECT_EQ(logEntry.id(), client2.id);
  logEntry.Clear();
  EXPECT_FALSE(logParser.Next(logEntry));

  // Remove the log file.
  EXPECT_TRUE(boost::filesystem::remove(boost::filesystem::path(filePath)));
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Set the partition name for this process.
  setenv("SWARM_LOG", "1", 1);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
