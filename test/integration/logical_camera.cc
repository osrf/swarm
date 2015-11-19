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
#include "test/test_config.h"

class LogicalCameraTest : public gazebo::ServerFixture
{
  public: LogicalCameraTest()
  {
    gazebo::common::SystemPaths::Instance()->AddPluginPaths(
      SWARM_PROJECT_TEST_INTEGRATION_PATH);
    gazebo::common::SystemPaths::Instance()->AddGazeboPaths(
      SWARM_PROJECT_TEST_WORLD_PATH);
    gazebo::common::SystemPaths::Instance()->AddGazeboPaths(
      SWARM_PROJECT_TEST_SOURCE_PATH);
  }
};
/*
/////////////////////////////////////////////////
TEST_F(LogicalCameraTest, FalseNegative)
{
  Load("logical_camera_00.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(1000);
}

/////////////////////////////////////////////////
TEST_F(LogicalCameraTest, FalsePositive)
{
  Load("logical_camera_01.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(1000);
}
*/
/////////////////////////////////////////////////
TEST_F(LogicalCameraTest, FalsePositiveDuration)
{
  Load("logical_camera_02.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(1000);
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ignition::math::Rand::Seed(13458);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
