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
}

//////////////////////////////////////////////////
void BooFinderPlugin::Update(const gazebo::common::UpdateInfo & /*_info*/)
{
  this->iterations++;

  // Only one robot will try to contact with the BOO (192.168.2.1).
  if ((this->Host() != "192.168.2.1") || (this->iterations != 0))
    return;

  switch (this->testCase)
  {
    case 0:
    case 4:
    {
      // Send a unicast message to the BOO with a valid lost person position.
      EXPECT_TRUE(this->SendTo("FOUND 100.0 100.0 0.5", this->kBoo,
        this->kBooPort));
      break;
    }
    case 1:
    {
      // Send a broadcast message to the BOO with a valid lost person position.
      EXPECT_TRUE(this->SendTo("FOUND 100.0 100.0 0.5", this->kBroadcast,
        this->kBooPort));
      break;
    }
    case 2:
    {
      // Send a message to the BOO containing an unsupported command.
      EXPECT_TRUE(this->SendTo("HELP 1.0 2.0 3.0", this->kBoo, this->kBooPort));
      break;
    }
    case 3:
    {
      // Send a message to the BOO containing a malformed FOUND command.
      EXPECT_TRUE(this->SendTo("FOUND 1.0 xxx 3.0", this->kBoo,
        this->kBooPort));
      break;
    }
    case 5:
    {
      // Send a message to the BOO containing a wrong position.
      EXPECT_TRUE(this->SendTo("FOUND 1.0 2.0 3.0", this->kBoo,
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
