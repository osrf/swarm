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

//////////////////////////////////////////////////
TeamControllerPlugin::TeamControllerPlugin()
  : RobotPlugin()
{
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Load(sdf::ElementPtr _sdf)
{
  // We'll need a world pointer later to check the time.
  this->worldPtr = gazebo::physics::get_world();
  // Sign up to receive unicast and broadcast messages
  this->Bind(&TeamControllerPlugin::OnDataReceived, this, this->Host());
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  switch (this->Type())
  {
    case GROUND:
    case ROTOR:
      {
        // Do we see the lost person?
        // Get the camera information
        ImageData img;
        if (this->Image(img))
        {
          for (auto const obj : img.objects)
          {
            if (obj.first.find("lost_person") != std::string::npos)
            {
              std::cout << "[" << this->Host() <<
                "] I found the lost person at " << obj.second << "!" <<
                std::endl;
              // Tell everybody about it.
              std::stringstream successMsg;
              successMsg << obj.second;
              std::cout << "[" << this->Host() <<
                "] Sending " << successMsg.str() << std::endl;
              this->SendTo(successMsg.str(), this->kBroadcast);
            }
          }
        }

        // Do a random walk, changing direction every once in a while.
        gazebo::common::Time changePeriod(10, 0);
        gazebo::common::Time currTime = this->worldPtr->GetSimTime();
      
        if ((this->lastCmdTime == gazebo::common::Time::Zero) ||
            ((currTime - this->lastCmdTime) > changePeriod))
        {
          // Time has elapsed; time to pick new velocities.

          // Constant forward velocity (X)
          ignition::math::Vector3d linVel(1,0,0);
          // Bounded angular velocity (yaw about Z)
          ignition::math::Vector3d angVel(0, 0,
            ignition::math::Rand::DblUniform(-0.5, 0.5));
          std::cout << "[" << this->Host() << "] Changing velocity to (" <<
            linVel.X() << ", " << angVel.Z() << ")" << std::endl;
          this->SetLinearVelocity(linVel);
          this->SetAngularVelocity(angVel);
          this->lastLinVel = linVel;
          this->lastAngVel = angVel;
          this->lastCmdTime = currTime;
        }
        else if ((currTime - this->lastCmdTime) > changePeriod/4.0)
        {
          // Part of the time has elapsed; start going forward (to avoid just
          // doing circles).
          this->SetLinearVelocity(this->lastLinVel);
          this->lastAngVel.Z(0);
          this->SetAngularVelocity(this->lastAngVel);
        }
        else
        {
          // Not enough time has elapsed; send the last command again (otherwise
          // the robot will stop after one time step).
          this->SetLinearVelocity(this->lastLinVel);
          this->SetAngularVelocity(this->lastAngVel);
        }
        break;
      }
    case FIXED_WING:
      {
        // Not supported yet, because the control is a little tricky.
        gzerr << "Fixed wing unsupported\n";
        break;
      }
    default:
      {
        gzerr << "Unknown vehicle type[" << this->Type() << "]\n";
        break;
      }
  };
}

//////////////////////////////////////////////////
void TeamControllerPlugin::OnDataReceived(const std::string &_srcAddress,
    const std::string &_data)
{
  std::cout << "---" << std::endl;
  std::cout << "[" << this->Host() << "] New message received" << std::endl;
  std::cout << "\tFrom: [" << _srcAddress << "]" << std::endl;
  std::cout << "\tData: [" << _data << "]" << std::endl;
}

