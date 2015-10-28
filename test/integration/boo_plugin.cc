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
#include "boo_plugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(BooFinderPlugin)

//////////////////////////////////////////////////
BooFinderPlugin::BooFinderPlugin()
: RobotPlugin()
{
}

//////////////////////////////////////////////////
void BooFinderPlugin::Load(sdf::ElementPtr _sdf)
{
  this->testCase = _sdf->Get<int>("test_case");
  this->maxDt = _sdf->Get<double>("max_dt");
}

//////////////////////////////////////////////////
void BooFinderPlugin::Update(const gazebo::common::UpdateInfo & /*_info*/)
{
  this->iterations++;

  // Only one robot will try to contact with the BOO (192.168.2.1).
  if (this->Host() != "192.168.2.1")
    return;

  // Tests #8, #9 send the message to the BOO later than the other tests.
  if ((this->testCase == 8) || (this->testCase == 9))
  {
    auto w = gazebo::physics::get_world();
    auto itersPerSecond = ceil(1.0 / w->GetPhysicsEngine()->GetMaxStepSize());
    int targetIters = itersPerSecond * this->maxDt + 1;

    if (this->iterations != targetIters)
      return;
  }
  // The rest of the tests send the message to the BOO at iterations=0.
  else if (this->iterations != 2)
    return;

  switch (this->testCase)
  {
    case 0:
      EXPECT_EQ(this->LostPersonDir(), ignition::math::Vector2d(0.5, 0.5));
    case 4:
    case 8:
    {
      // Send a unicast message to the BOO with a valid lost person pos/time.
      EXPECT_TRUE(this->SendTo("FOUND 100.0 100.0 0.5 0.0", this->kBoo,
        this->kBooPort));
      break;
    }
    case 1:
    {
      EXPECT_EQ(this->LostPersonDir(), ignition::math::Vector2d(1.0, 0.0));
      // Send a broadcast message to the BOO with a valid lost person pos/time.
      EXPECT_TRUE(this->SendTo("FOUND 110.0 0.0 0.5 0.0", this->kBroadcast,
        this->kBooPort));
      break;
    }
    case 2:
    {
      EXPECT_EQ(this->LostPersonDir(), ignition::math::Vector2d(0.0, 1.0));
      // Send a message to the BOO containing an unsupported command.
      EXPECT_TRUE(this->SendTo("HELP 1.0 2.0 3.0 0.0", this->kBoo,
        this->kBooPort));
      break;
    }
    case 3:
    {
      EXPECT_EQ(this->LostPersonDir(), ignition::math::Vector2d(-0.5, -0.5));
      // Send a message to the BOO containing a malformed FOUND command.
      EXPECT_TRUE(this->SendTo("FOUND 1.0 xxx 3.0 0.0", this->kBoo,
        this->kBooPort));
      break;
    }
    case 5:
    {
      EXPECT_EQ(this->LostPersonDir(), ignition::math::Vector2d(-1.0, 0.0));
      // Send a message to the BOO containing a wrong position.
      EXPECT_TRUE(this->SendTo("FOUND 1.0 2.0 3.0 0.0", this->kBoo,
        this->kBooPort));
      break;
    }
    case 6:
    {
      EXPECT_EQ(this->LostPersonDir(), ignition::math::Vector2d(-0.5, -0.5));
      // Send a message to the BOO containing a negative time.
      EXPECT_TRUE(this->SendTo("FOUND 1.0 2.0 3.0 -1.0", this->kBoo,
        this->kBooPort));
      break;
    }
    case 7:
    {
      EXPECT_EQ(this->LostPersonDir(), ignition::math::Vector2d(-1.0, 0.0));
      // Send a message to the BOO containing a future time.
      EXPECT_TRUE(this->SendTo("FOUND 1.0 2.0 3.0 10.0", this->kBoo,
        this->kBooPort));
      break;
    }
    case 9:
    {
      EXPECT_EQ(this->LostPersonDir(), ignition::math::Vector2d(-0.5, -0.5));
      // Send a message with a valid pos/time that is not the only one stored
      // by the BOO.
      EXPECT_TRUE(this->SendTo("FOUND -25.0 -30.0 0.5 0.8", this->kBoo,
        this->kBooPort));
      break;
    }
    default:
    {
      gzerr << "BooFinderPlugin::Update() Test [" << this->testCase << "] "
            << "not expected." << std::endl;
      FAIL();
      break;
    }
  };
}
