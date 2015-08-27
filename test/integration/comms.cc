/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

class CommsTest : public gazebo::ServerFixture
{
  public: CommsTest()
  {
    gazebo::common::SystemPaths::Instance()->AddPluginPaths(
      SWARM_PROJECT_TEST_PLUGIN_PATH);
    gazebo::common::SystemPaths::Instance()->AddGazeboPaths(
      SWARM_PROJECT_TEST_WORLD_PATH);
  }
};

/////////////////////////////////////////////////
/// \brief No drops or outages and the two vehicles are always within range.
TEST_F(CommsTest, PerfectComms)
{
  Load("comms_0.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

/////////////////////////////////////////////////
/// \brief All packages drop because the min/max drop probabilities
/// are set to 1.0 .
TEST_F(CommsTest, AllPackagesDrop)
{
  Load("comms_1.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

/////////////////////////////////////////////////
/// \brief All packages drop because the outage probability
/// is set to 1.0 .
TEST_F(CommsTest, TotalOutage)
{
  Load("comms_2.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

/////////////////////////////////////////////////
/// \brief All packages drop because the vehicles are not within
/// the communication range.
TEST_F(CommsTest, OutOfRange)
{
  Load("comms_3.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

/////////////////////////////////////////////////
/// \brief All messages go through with one line of trees between vehicles.
TEST_F(CommsTest, OneTreeCommsOK)
{
  Load("comms_4.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

/////////////////////////////////////////////////
/// \brief All messages drop with one line of trees between vehicles.
/// Now, the vehicles are not close enough to get over the tree penalty.
TEST_F(CommsTest, OneTreeCommsFail)
{
  Load("comms_5.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

/////////////////////////////////////////////////
/// \brief All messages drop with two lines of trees between vehicles.
TEST_F(CommsTest, TwoTreesCommsFail)
{
  Load("comms_6.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

/////////////////////////////////////////////////
/// \brief 50% remote messages drop.
TEST_F(CommsTest, HalfMsgsDrop)
{
  ignition::math::Rand::Seed(13458);
  Load("comms_7.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

/////////////////////////////////////////////////
/// \brief Check temporary outages.
TEST_F(CommsTest, TemporaryOutage)
{
  ignition::math::Rand::Seed(13220);
  Load("comms_8.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

/////////////////////////////////////////////////
/// \brief Check permanent outages.
TEST_F(CommsTest, PermanentOutage)
{
  ignition::math::Rand::Seed(13111);
  Load("comms_9.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

/////////////////////////////////////////////////
/// \brief Check temporary outages + drops.
/// The expectation is to drop 5% of packages and to have one outage of 0.2 sec.
TEST_F(CommsTest, OutagesAndDrops)
{
  ignition::math::Rand::Seed(13111);
  Load("comms_10.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Step the world so that the test library experiences update events.
  world->Step(101);
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
