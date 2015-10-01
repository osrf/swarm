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

#include "logical_camera_plugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(LogicalCameraPlugin)

//////////////////////////////////////////////////
LogicalCameraPlugin::LogicalCameraPlugin()
: RobotPlugin()
{
}

//////////////////////////////////////////////////
void LogicalCameraPlugin::Load(sdf::ElementPtr _sdf)
{
  this->testCase = _sdf->Get<int>("test_case");
  this->world = gazebo::physics::get_world("default");
}

//////////////////////////////////////////////////
void LogicalCameraPlugin::Update(const gazebo::common::UpdateInfo & /*_info*/)
{
  switch (this->testCase)
  {
    default:
    case 0:
      this->Update0();
      break;
    case 1:
      this->Update1();
      break;
  }
}

/////////////////////////////////////////////////
void LogicalCameraPlugin::Update0()
{
  static int iteration = 0;
  static int count = 0;

  swarm::ImageData logImg;
  EXPECT_TRUE(this->Image(logImg));

  // Keep track of the number of times the lost_person is observed
  if (logImg.objects.find("lost_person") != logImg.objects.end())
  {
    ++count;
  }

  // We assume the test runs for 1000 iterations.
  if (iteration == 999)
  {
    // Make sure the lost person is not seen 100% of the time. This number,
    // and the number in the following test is tied to
    // RobotPlugin::cameraFalseNegativeProbMin and
    // RobotPlugin::cameraFalseNegativeProbMax.
    EXPECT_LT(count, 950);
    EXPECT_GT(count, 890);
  }
  ++iteration;
}

/////////////////////////////////////////////////
void LogicalCameraPlugin::Update1()
{
  if (this->Host() != "192.168.1.1")
    return;

  static int iteration = 0;
  static int count = 0;

  swarm::ImageData logImg;
  EXPECT_TRUE(this->Image(logImg));

  // Keep track of the number of times the lost_person is observed
  if (logImg.objects.find("lost_person") != logImg.objects.end())
  {
    ++count;
  }

  // We assume the test runs for 1000 iterations.
  if (iteration == 999)
  {
    // Make sure the lost person is seen some of the time. This number,
    // and the number in the following test are tied to
    // RobotPlugin::cameraFalsePositiveProbMin and cameraFalsePositiveProbMax.
    EXPECT_LT(count, 30);
    EXPECT_GT(count, 2);
  }
  ++iteration;
}
