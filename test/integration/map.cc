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

class MapTest : public gazebo::ServerFixture
{
  public: MapTest()
  {
    gazebo::common::SystemPaths::Instance()->AddPluginPaths(
      SWARM_PROJECT_TEST_INTEGRATION_PATH);
    gazebo::common::SystemPaths::Instance()->AddGazeboPaths(
      SWARM_PROJECT_TEST_WORLD_PATH);
  }
};

/////////////////////////////////////////////////
/// \brief Valid unicast message from a robot to the BOO.
TEST_F(MapTest, Query)
{
  Load("map_00.world", true);
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
  world->SetPaused(false);

  // Wait some time so that the test library experiences update events.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
