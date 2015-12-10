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

#include <chrono>
#include <string>
#include <thread>
#include <gazebo/physics/physics.hh>
#include <gazebo/test/ServerFixture.hh>
#include <ignition/math/Vector3.hh>
#include "swarm/BooPlugin.hh"
#include "swarm/SwarmTypes.hh"
#include "test/test_config.h"

class BooTest : public gazebo::ServerFixture
{
  public: BooTest()
  {
    gazebo::common::SystemPaths::Instance()->AddPluginPaths(
      SWARM_PROJECT_TEST_INTEGRATION_PATH);
    gazebo::common::SystemPaths::Instance()->AddGazeboPaths(
      SWARM_PROJECT_TEST_WORLD_PATH);
  }
};

/////////////////////////////////////////////////
/// \brief Validate the result of each test case.
void validateResult(const int _testCase)
{
  switch (_testCase)
  {
    // Valid unicast message.
    case 0:
    // Valid broadcast message.
    case 1:
    // Valid pos/time sent to the BOO
    // with a time t older than the last entry stored.
    case 9:
    {
      // ToDo: Find a way to signal that the lost person was found.
      break;
    }
    // Unsupported command.
    case 2:
    // Malformed message.
    case 3:
    // Robot was too far from the BOO.
    case 4:
    // Robot sent an incorrect pos to the BOO.
    case 5:
    // Robot sent a negatime time to the BOO.
    case 6:
    // Robot sent a time in the future to the BOO.
    case 7:
    // Robot sent a correct pos/time to the BOO but out of the time window.
    case 8:
    {
      EXPECT_FALSE(gazebo::physics::get_world()->IsPaused());
      break;
    }
    default:
    {
      gzerr << "validateResult() Test [" << _testCase << "] "
            << "not expected." << std::endl;
      FAIL();
      break;
    }
  };
}

/////////////////////////////////////////////////
/// \brief Valid unicast message from a robot to the BOO.
TEST_F(BooTest, Unicast)
{
  auto testCase = 0;
  Load("boo_00.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // Wait some time so that the test library experiences update events.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  validateResult(testCase);
}

/////////////////////////////////////////////////
/// \brief Valid broadcast message from a robot to the BOO.
TEST_F(BooTest, Broadcast)
{
  auto testCase = 1;
  Load("boo_01.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // Wait some time so that the test library experiences update events.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  validateResult(testCase);
}

/////////////////////////////////////////////////
/// \brief Unsupported command sent to the BOO.
TEST_F(BooTest, UnsupportedCmd)
{
  auto testCase = 2;
  Load("boo_02.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // Wait some time so that the test library experiences update events.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  validateResult(testCase);
}

/////////////////////////////////////////////////
/// \brief Malformed message sent to the BOO.
TEST_F(BooTest, UnsupportedArgs)
{
  auto testCase = 3;
  Load("boo_03.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // Wait some time so that the test library experiences update events.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  validateResult(testCase);
}

/////////////////////////////////////////////////
/// \brief Robot was too far from the BOO.
TEST_F(BooTest, TooFar)
{
  auto testCase = 4;
  Load("boo_04.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // Wait some time so that the test library experiences update events.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  validateResult(testCase);
}

/////////////////////////////////////////////////
/// \brief Robot sends an incorrect lost person's position to the BOO.
TEST_F(BooTest, WrongPos)
{
  auto testCase = 5;
  Load("boo_05.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // Wait some time so that the test library experiences update events.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  validateResult(testCase);
}

/////////////////////////////////////////////////
/// \brief Negative time in the message reported to the BOO.
TEST_F(BooTest, NegativeTime)
{
  auto testCase = 6;
  Load("boo_06.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // Wait some time so that the test library experiences update events.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  validateResult(testCase);
}

/////////////////////////////////////////////////
/// \brief Future time in the message reported to the BOO.
TEST_F(BooTest, FutureTime)
{
  auto testCase = 7;
  Load("boo_07.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // Wait some time so that the test library experiences update events.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  validateResult(testCase);
}

/////////////////////////////////////////////////
/// \brief Valid pos/time sent to the BOO but out of the allowed time window.
TEST_F(BooTest, OutOfWindow)
{
  auto testCase = 8;
  Load("boo_08.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // This is the value that the world file contains.
  auto maxDt = 5;
  std::this_thread::sleep_for(std::chrono::seconds(maxDt));

  validateResult(testCase);
}

/////////////////////////////////////////////////
/// \brief Valid pos/time sent to the BOO with a time t older than the last
/// entry stored.
TEST_F(BooTest, ValidGuess)
{
  auto testCase = 9;
  Load("boo_09.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // Wait 1 second.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Teleport the lost person to a different position.
  auto model = world->GetModel("lost_person");
  model->SetWorldPose(gazebo::math::Pose(-50, -100, 0.5, 0, 0, 0));

  // This is the value that the world file contains.
  auto maxDt = 5;
  std::this_thread::sleep_for(std::chrono::seconds(maxDt));

  validateResult(testCase);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
