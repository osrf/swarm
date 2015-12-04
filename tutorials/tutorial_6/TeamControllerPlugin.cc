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

#include <string>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Quaternion.hh>
#include <sdf/sdf.hh>
#include "TeamControllerPlugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(TeamControllerPlugin)

// Robot addresses.
static const uint32_t Robot1 = 10;
static const uint32_t Robot2 = 11;

//////////////////////////////////////////////////
TeamControllerPlugin::TeamControllerPlugin()
  : RobotPlugin()
{
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Load(sdf::ElementPtr _sdf)
{
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  // Only print for one robot, to minimize console output
  if (this->Host() == Robot1)
  {
    // Get IMU information.
    ignition::math::Vector3d linVel, angVel;
    ignition::math::Quaterniond orient;
    if (this->Imu(linVel, angVel, orient))
    {
      std::cout << "[" << this->Host() << "] Lin. Vel: " << linVel << std::endl;
      std::cout << "[" << this->Host() << "] Ang. Vel: " << angVel << std::endl;
      std::cout << "[" << this->Host() << "] Orient.: "  << orient.Euler()
                << std::endl;
    }

    // Get bearing.
    ignition::math::Angle bearing;
    if (this->Bearing(bearing))
      std::cout << "[" << this->Host() << "] Bearing: " << bearing << std::endl;
  }

  // Simple example for moving each type of robot.
  switch (this->Type())
  {
    case GROUND:
      {
        this->SetLinearVelocity(ignition::math::Vector3d(1, 0, 0));
        this->SetAngularVelocity(ignition::math::Vector3d(0, 0, 0.1));
        break;
      }
    case ROTOR:
      {
        this->SetLinearVelocity(ignition::math::Vector3d(0, 0, 1));
        this->SetAngularVelocity(ignition::math::Vector3d(0, 0, -0.1));
        break;
      }
    case FIXED_WING:
      {
        this->SetLinearVelocity(ignition::math::Vector3d(1, 0, 0));
        this->SetAngularVelocity(ignition::math::Vector3d(0, -0.4, 0));
        break;
      }
    default:
      {
        gzerr << "Unknown vehicle type[" << this->Type() << "]\n";
        break;
      }
  };
}
