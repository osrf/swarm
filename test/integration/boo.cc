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

#include <gazebo/test/ServerFixture.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
#include "swarm/BooPlugin.hh"
#include "swarm/SwarmTypes.hh"
#include "msgs/personfound.pb.h"
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

/// \brief Each test case has a unique number.
int testCase;

/// \brief True if a robot found the lost person.
bool found;

/// \brief Address of the robot that found the lost person.
std::string address;

/// \brief Pos of the lost person reported by a robot.
ignition::math::Vector3d pos;

/// \brief Time at which the robot reported to see the lost person.
double t;

//////////////////////////////////////////////////
/// \brief Reset global variables.
void reset()
{
  testCase = 0;
  found = false;
  address = "";
  pos.Set(0.0, 0.0, 0.0);
  t = 0.0;
}

//////////////////////////////////////////////////
/// \brief Function called each time a lost person is found.
void onFound(const std::string &_topic, const swarm::msgs::PersonFound &_msg)
{
  EXPECT_EQ(_topic, "/swarm/found");
  found = true;
  address = _msg.address();
  pos.X(_msg.pos().x());
  pos.Y(_msg.pos().y());
  pos.Z(_msg.pos().z());
  t = _msg.time();
}

/////////////////////////////////////////////////
/// \brief Validate the result of each test case.
void validateResult()
{
  switch (testCase)
  {
    // Valid unicast message.
    case 0:
    // Valid broadcast message.
    case 1:
    // Valid pos/time sent to the BOO
    // with a time t older than the last entry stored.
    case 9:
    {
      EXPECT_TRUE(found);
      EXPECT_TRUE(gazebo::physics::get_world()->IsPaused());
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
      EXPECT_FALSE(found);
      break;
    }
    default:
    {
      gzerr << "validateResult() Test [" << testCase << "] "
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
  ignition::transport::Node node;

  reset();
  testCase = 0;
  node.Subscribe("/swarm/found", &onFound);
  Load("boo_00.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(5);

  validateResult();
}

/////////////////////////////////////////////////
/// \brief Valid broadcast message from a robot to the BOO.
TEST_F(BooTest, Broadcast)
{
  ignition::transport::Node node;

  reset();
  testCase = 1;
  node.Subscribe("/swarm/found", &onFound);
  Load("boo_01.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(5);

  validateResult();
}

/////////////////////////////////////////////////
/// \brief Unsupported command sent to the BOO.
TEST_F(BooTest, UnsupportedCmd)
{
  ignition::transport::Node node;

  reset();
  testCase = 2;
  node.Subscribe("/swarm/found", &onFound);
  Load("boo_02.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(5);

  validateResult();
}

/////////////////////////////////////////////////
/// \brief Malformed message sent to the BOO.
TEST_F(BooTest, UnsupportedArgs)
{
  ignition::transport::Node node;

  reset();
  testCase = 3;
  node.Subscribe("/swarm/found", &onFound);
  Load("boo_03.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(5);

  validateResult();
}

/////////////////////////////////////////////////
/// \brief Robot was too far from the BOO.
TEST_F(BooTest, TooFar)
{
  ignition::transport::Node node;

  reset();
  testCase = 4;
  node.Subscribe("/swarm/found", &onFound);
  Load("boo_04.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(5);

  validateResult();
}

/////////////////////////////////////////////////
/// \brief Robot sends an incorrect lost person's position to the BOO.
TEST_F(BooTest, WrongPos)
{
  ignition::transport::Node node;

  reset();
  testCase = 5;
  node.Subscribe("/swarm/found", &onFound);
  Load("boo_05.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(5);

  validateResult();
}

/////////////////////////////////////////////////
/// \brief Negative time in the message reported to the BOO.
TEST_F(BooTest, NegativeTime)
{
  ignition::transport::Node node;

  reset();
  testCase = 6;
  node.Subscribe("/swarm/found", &onFound);
  Load("boo_06.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(5);

  validateResult();
}

/////////////////////////////////////////////////
/// \brief Future time in the message reported to the BOO.
TEST_F(BooTest, FutureTime)
{
  ignition::transport::Node node;

  reset();
  testCase = 7;
  node.Subscribe("/swarm/found", &onFound);
  Load("boo_07.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(5);

  validateResult();
}

/////////////////////////////////////////////////
/// \brief Valid pos/time sent to the BOO but out of the allowed time window.
TEST_F(BooTest, OutOfWindow)
{
  ignition::transport::Node node;

  reset();
  testCase = 8;
  node.Subscribe("/swarm/found", &onFound);
  Load("boo_08.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  auto stepsPerSecond = ceil(1.0 / world->GetPhysicsEngine()->GetMaxStepSize());
  auto steps = stepsPerSecond * swarm::kMaxDt.Double() + 5;
  world->Step(steps);

  validateResult();
}

/////////////////////////////////////////////////
/// \brief Valid pos/time sent to the BOO with a time t older than the last
/// entry stored.
TEST_F(BooTest, ValidGuess)
{
  ignition::transport::Node node;

  reset();
  testCase = 9;
  node.Subscribe("/swarm/found", &onFound);
  Load("boo_09.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  auto stepsPerSecond = ceil(1.0 / world->GetPhysicsEngine()->GetMaxStepSize());

  // Wait 1 second.
  world->Step(stepsPerSecond);

  // Teleport the lost person to a different position.
  auto model = world->GetModel("lost_person");
  model->SetWorldPose(gazebo::math::Pose(-50, -100, 0.5, 0, 0, 0));

  // Wait some time to stay beyond the kMaxStorageTime window.
  auto steps = stepsPerSecond * (swarm::kMaxDt.Double() - 1) + 5;
  if (steps < 0)
    steps = 0;
  world->Step(steps);

  validateResult();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
