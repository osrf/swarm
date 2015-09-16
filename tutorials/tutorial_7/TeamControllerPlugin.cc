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

#include <iostream>
#include <sstream>
#include <string>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Rand.hh>
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
  // Sign up to receive unicast and broadcast messages
  this->Bind(&TeamControllerPlugin::OnDataReceived, this, this->Host(),
    this->kBooPort);
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
          for (auto const &obj : img.objects)
          {
            if (obj.first.find("lost_person") != std::string::npos)
            {
              // Convert to the world frame.
              auto personInWorld = this->CameraToWorld(obj.second);
              // Tell everybody about it.
              std::stringstream successMsg;
              successMsg << "FOUND " << personInWorld.Pos() << " " <<
                _info.simTime.Double();
              std::cout << "[" << this->Host() <<
                "] I found the lost person.  Sending: " << successMsg.str() <<
                std::endl;
              this->SendTo(successMsg.str(), this->kBroadcast, this->kBooPort);
              // Remember this message for later, to avoid relaying it.
              this->messagesSent.push_back(successMsg.str());
            }
          }
        }

        // Do a random walk, changing direction every once in a while.
        gazebo::common::Time changePeriod(10, 0);

        if ((this->lastCmdTime == gazebo::common::Time::Zero) ||
            ((_info.simTime - this->lastCmdTime) > changePeriod))
        {
          // Time has elapsed; time to pick new velocities.

          // Constant forward velocity (X)
          ignition::math::Vector3d linVel(1,0,0);
          // Bounded angular velocity (yaw about Z)
          ignition::math::Vector3d angVel(0, 0,
            ignition::math::Rand::DblUniform(-0.5, 0.5));
          //std::cout << "[" << this->Host() << "] Changing velocity to (" <<
            //linVel.X() << ", " << angVel.Z() << ")" << std::endl;
          this->SetLinearVelocity(linVel);
          this->SetAngularVelocity(angVel);
          this->lastLinVel = linVel;
          this->lastAngVel = angVel;
          this->lastCmdTime = _info.simTime;
        }
        else if ((_info.simTime - this->lastCmdTime) > changePeriod/4.0)
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
  // Totally dumb mesh network strategy: relay everything you hear that you
  // haven't previously relayed.
  auto alreadySent = false;
  for (auto const &msg : this->messagesSent)
  {
    if (msg == _data)
    {
      alreadySent = true;
      break;
    }
  }

  if (!alreadySent)
  {
    // I haven't sent this before; relay it.
    std::cout << "[" << this->Host() << "] Relaying message from " <<
      _srcAddress << " with payload " << _data << std::endl;
    // At this point, I'd like to have the original dest address.  I'll just
    // assume that it should go to (kBroadcast, kBooPort).
    this->SendTo(_data, this->kBroadcast, this->kBooPort);
    // Remember this message for later.
    this->messagesSent.push_back(_data);
  }
  else
  {
    // I've sent this already; don't relay it.

    //std::cout << "[" << this->Host() << "] NOT relaying message from " <<
      //_srcAddress << " with payload " << _data << std::endl;
  }
}

