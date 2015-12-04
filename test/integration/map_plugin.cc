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
#include <sdf/sdf.hh>
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
  this->iteration = 0;
}

//////////////////////////////////////////////////
void MapPlugin::Update(const gazebo::common::UpdateInfo & /*_info*/)
{
  this->iteration++;
  if (this->iteration < 5)
    return;

  double lat, lon, alt;
  double height;
  TerrainType tType;
  this->Pose(lat, lon, alt);
  EXPECT_TRUE(this->MapQuery(lat, lon, height, tType));
  EXPECT_FALSE(this->MapQuery(lat+1, lon+1, height, tType));
  EXPECT_FALSE(this->MapQuery(lat-1, lon-1, height, tType));

  switch (this->testCase)
  {
    default:
    case 0:
      EXPECT_NEAR(height, 993.365, 1e-3);
      EXPECT_EQ(tType, TerrainType::PLAIN);
      break;

    case 1:
      EXPECT_NEAR(height, 932.8344, 1e-3);
      EXPECT_EQ(tType, TerrainType::FOREST);
      break;

    case 2:
      EXPECT_NEAR(height, 658.507, 1e-3);
      EXPECT_EQ(tType, TerrainType::BUILDING);
      break;
  };
}
