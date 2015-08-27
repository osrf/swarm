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

class BatteryTest : public gazebo::ServerFixture
{
};

std::string batterySDF1 = R"DELIM(
<sdf version='1.5'>
  <model name='vehicle'>
    <pose>0 0 0.05 0 0 0</pose>
    <link name='link'>
      <sensor name='gps' type='gps'>
        <gps/>
        <always_on>1</always_on>
      </sensor>

      <sensor name='imu' type='imu'>
        <imu/>
        <always_on>1</always_on>
      </sensor>

      <sensor name='camera' type='logical_camera'>
        <logical_camera>
          <near>0</near>
          <far>10</far>
          <horizontal_fov>1.057</horizontal_fov>
          <aspect_ratio>1</aspect_ratio>
        </logical_camera>
        <visualize>true</visualize>
        <always_on>true</always_on>
      </sensor>

      <kinematic>true</kinematic>
      <gravity>false</gravity>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 .1 .1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 .1 .1</size>
          </box>
        </geometry>
      </visual>
      <visual name="visual_front">
        <pose>0.07 0 0.04 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>.04</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Red</name>
          </script>
        </material>
      </visual>
    </link>

    <!-- Load the plugin to control this robot -->
    <plugin name='swarm_controller' filename='libbattery_plugin.so'>
      <type>ground</type>
      <camera>link::camera</camera>
      <gps>link::gps</gps>
      <imu>link::imu</imu>
      <address>192.168.2.1</address>
      <num_messages>1</num_messages>
      <swarm_search_area>
        <min_relative_latitude_deg>-0.01</min_relative_latitude_deg>
        <max_relative_latitude_deg>0.01</max_relative_latitude_deg>
        <min_relative_longitude_deg>-0.01</min_relative_longitude_deg>
        <max_relative_longitude_deg>0.01</max_relative_longitude_deg>
      </swarm_search_area>
      <battery>
        <capacity>3500</capacity>
        <consumption>1500</consumption>
        <consumption_factor>0.7</consumption_factor>
      </battery>
    </plugin>
  </model>
</sdf>
)DELIM";

/////////////////////////////////////////////////
TEST_F(BatteryTest, Consumption)
{
  gazebo::common::SystemPaths::Instance()->AddPluginPaths(
      SWARM_PROJECT_TEST_PLUGIN_PATH);
  Load("worlds/swarm_empty.world", true);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Spawn the model
  SpawnSDF(batterySDF1);
  WaitUntilEntitySpawn("vehicle", 100, 100);

  // Step the world so that the test library experiences update events.
  world->Step(100);
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
