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
#include <sdf/sdf.hh>
#include "TeamControllerPlugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(TeamControllerPlugin)

//////////////////////////////////////////////////
TeamControllerPlugin::TeamControllerPlugin()
  : RobotPlugin(),
    msgsSent(0)
{
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Load(sdf::ElementPtr _sdf)
{
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  // Get pose and altitude.
  double latitude, longitude, altitude;
  this->Pose(latitude, longitude, altitude);
  double minLatitude, maxLatitude, minLongitude, maxLongitude;
  this->SearchArea(minLatitude, maxLatitude, minLongitude, maxLongitude);

  std::cout << "Lat/Long/Alt[" << latitude << " " << longitude
            << " " << altitude << "]\n";

  std::cout << "Search Area[" << minLatitude << " " << minLongitude << " : "
            << maxLatitude << " " << maxLongitude << "]\n";

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
