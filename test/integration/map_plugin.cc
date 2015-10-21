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

#include <gtest/gtest.h>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>
#include "swarm/BooPlugin.hh"
#include "swarm/SwarmTypes.hh"
#include "map_plugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(MapPlugin)

//////////////////////////////////////////////////
MapPlugin::MapPlugin()
: RobotPlugin()
{
}

//////////////////////////////////////////////////
void MapPlugin::Load(sdf::ElementPtr _sdf)
{
  this->testCase = _sdf->Get<int>("test_case");
}

//////////////////////////////////////////////////
void MapPlugin::Update(const gazebo::common::UpdateInfo & /*_info*/)
{
  double lat, lon, alt;
  double height;
  RobotPlugin::TerrainType terrainType;
  this->Pose(lat, lon, alt);
  this->MapQuery(lat, lon, height, terrainType);

  std::cout << "Alt[" << alt << "] Height[" << height << "]\n";

}
