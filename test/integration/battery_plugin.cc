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

#include "battery_plugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin)

//////////////////////////////////////////////////
BatteryPlugin::BatteryPlugin()
: RobotPlugin()
{
}

//////////////////////////////////////////////////
void BatteryPlugin::Load(sdf::ElementPtr _sdf)
{
  this->testCase = _sdf->Get<int>("test_case");

  EXPECT_NEAR(this->BatteryCapacity(), 110000, 1e-6);
  EXPECT_NEAR(this->BatteryConsumption(), 55000, 1e-6);
  EXPECT_NEAR(this->BatteryConsumptionFactor(), 0.7, 1e-6);

  this->world = gazebo::physics::get_world("default");

  double booLat, booLon;
  EXPECT_TRUE(this->BooPose(booLat, booLon));
  EXPECT_NEAR(booLat, 35.7753, 1e-4);
  EXPECT_NEAR(booLon, -120.774, 1e-4);
}

//////////////////////////////////////////////////
void BatteryPlugin::Update(const gazebo::common::UpdateInfo & /*_info*/)
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
    case 2:
      this->Update2();
      break;
  }
}

/////////////////////////////////////////////////
void BatteryPlugin::Update0()
{
  static int counter = 1;

  double expectedBattery = this->BatteryStartCapacity() -
    (this->BatteryConsumption() *
    this->BatteryConsumptionFactor() *
    ((this->world->GetPhysicsEngine()->GetMaxStepSize() * counter)/3600.0));

  EXPECT_NEAR(this->BatteryCapacity(), expectedBattery, 1e-4);

  ++counter;
}

/////////////////////////////////////////////////
void BatteryPlugin::Update1()
{
  EXPECT_NEAR(this->BatteryCapacity(), this->BatteryStartCapacity(), 1e-4);
}

/////////////////////////////////////////////////
void BatteryPlugin::Update2()
{
  static int counter = 1;

  if (counter < 500)
  {
    double expectedBattery = this->BatteryStartCapacity() -
      (this->BatteryConsumption() *
       this->BatteryConsumptionFactor() *
       ((this->world->GetPhysicsEngine()->GetMaxStepSize() * counter)/3600.0));

    EXPECT_NEAR(this->BatteryCapacity(), expectedBattery, 1e-4);
  }
  else if (counter >= 500)
  {
    double capacityLost = this->BatteryConsumption() *
      this->BatteryConsumptionFactor() *
      ((this->world->GetPhysicsEngine()->GetMaxStepSize() * 500)/3600.0);

    double capacityGained = this->BatteryConsumption() *
       (this->BatteryConsumptionFactor()*4) *
       ((this->world->GetPhysicsEngine()->GetMaxStepSize() *
         (counter-500))/3600.0);

    double expectedBattery;
    if (capacityLost > capacityGained)
    {
      expectedBattery = this->BatteryStartCapacity() - capacityLost +
        capacityGained;
    }
    else
    {
      expectedBattery = this->BatteryStartCapacity();
    }

    EXPECT_NEAR(this->BatteryCapacity(), expectedBattery, 1e-4);
  }

  ++counter;
}
