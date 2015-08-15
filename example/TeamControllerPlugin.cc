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
  // Read the <num_messages> SDF parameter.
  if (!_sdf->HasElement("num_messages"))
  {
    gzerr << "TeamControllerPlugin::Load(): Unable to find the <num_messages> "
          << "parameter" << std::endl;
    return;
  }

  this->numMessageToSend = _sdf->Get<int>("num_messages");

  // Bind on my local address and default port.
  this->Bind(&TeamControllerPlugin::OnDataReceived, this, this->Host());

  // Bind on the multicast group and default port.
  this->Bind(&TeamControllerPlugin::OnDataReceived, this, this->kMulticast);
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  // Check if we already reached the limit of messages to be sent.
  if (this->msgsSent < this->numMessageToSend)
  {
    this->msgsSent++;

    std::string dstAddress;

    if (this->Host() == "192.168.2.1")
      dstAddress = "192.168.2.2";
    else
      dstAddress = "192.168.2.1";

    // Send a unicast message.
    if (!this->SendTo("Unicast data", dstAddress))
    {
      gzerr << "[" << this->Host() << "] TeamControllerPlugin::Update(): "
            << "Error sending a message to <" << dstAddress << ",DEFAULT_PORT>"
            << std::endl;
      return;
    }

    // Send a broadcast message.
    dstAddress = this->kBroadcast;
    if (!this->SendTo("Broadcast data", dstAddress))
    {
      gzerr << "[" << this->Host() << "] TeamControllerPlugin::Update(): "
            << "Error sending a message to <" << dstAddress << ",DEFAULT_PORT>"
            << std::endl;
      return;
    }

    // Send a multicast message.
    dstAddress = this->kMulticast;
    if (!this->SendTo("Multicast data", dstAddress))
    {
      gzerr << "[" << this->Host() << "] TeamControllerPlugin::Update(): "
            << "Error sending a message to <" << dstAddress << ",DEFAULT_PORT>"
            << std::endl;
      return;
    }

    // Show the list of neighbors.
    if (this->Neighbors().empty())
      gzmsg << "[" << this->Host() << "] Neighbors: EMPTY" << std::endl;
    else
    {
      gzmsg << "[" << this->Host() << "] Neighbors:" << std::endl;
      for (auto const &neighbor : this->Neighbors())
        gzmsg << "\t" << neighbor << std::endl;
    }
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

  // Get pose
  double latitude, longitude, altitude;
  this->Pose(latitude, longitude, altitude);
  double minLatitude, maxLatitude, minLongitude, maxLongitude;
  this->SearchArea(minLatitude, maxLatitude, minLongitude, maxLongitude);

  // Get the camera information
  ImageData img;
  if (this->Image(img))
  {
    // check if we can see the lost person
    if (img.objects.find("lost_person") != img.objects.end())
      gzmsg << "Lost person found at[" << img.objects["lost_person"] << "]\n";
  }

  // Get velocity and orientation
  ignition::math::Vector3d linVel, angVel;
  ignition::math::Quaterniond orient;
  if (this->Velocity(linVel, angVel))
  {
    gzmsg << "[" << this->Host() << "] Linear Vel: " << linVel << std::endl;
    gzmsg << "[" << this->Host() << "] Angular Vel: " << angVel << std::endl;
  }

  if (this->Orientation(orient))
  {
    gzmsg << "[" << this->Host() << "] Orientation: " << orient << std::endl;
  }

  // Only print for one robot, to minimize console output
  if (this->Host() == "192.168.2.1")
  {
    gzmsg << "[" << this->Host() << "] search area: " <<
      minLatitude << " " << maxLatitude << " " <<
      minLongitude << " " << maxLongitude << std::endl;
    gzmsg << "[" << this->Host() << "] lat long alt: " <<
      latitude << " " << longitude << " " << altitude << std::endl;
  }
}

//////////////////////////////////////////////////
void TeamControllerPlugin::OnDataReceived(const std::string &_srcAddress,
    const std::string &_data)
{
  gzmsg << "---" << std::endl;
  gzmsg << "[" << this->Host() << "] New message received" << std::endl;
  gzmsg << "\tFrom: [" << _srcAddress << "]" << std::endl;
  gzmsg << "\tData: [" << _data << "]" << std::endl;
}
