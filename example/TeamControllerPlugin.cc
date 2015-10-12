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
  // Launch rotor vehicles after 5 seconds of simulation time.
  if (_info.simTime > gazebo::common::Time(5, 0))
  {
    this->Launch();
  }

  // Check if we already reached the limit of messages to be sent.
  if (this->msgsSent < this->numMessageToSend)
  {
    this->msgsSent++;

    std::string dstAddress;

    if (this->Host() == "192.168.3.1")
      dstAddress = "192.168.3.2";
    else
      dstAddress = "192.168.3.1";

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
        this->SetAngularVelocity(ignition::math::Vector3d(0.1, 0, 0));
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
    {
      // Uncomment the following block to display if you found the lost person.
      // std::cout << "Lost person found at[" << img.objects["lost_person"]
      //          << "]" << std::endl;
    }
  }

  // Only print for one robot, to minimize console output
  if (this->Host() == "192.168.2.1")
  {
    // Get the list of neighbors.
    auto myNeighbors = this->Neighbors();
    // Uncomment the following block to display the list of neighbors.
    // if (myNeighbors.empty())
    //   std::cout << "[" << this->Host() << "] Neighbors: EMPTY" << std::endl;
    // else
    // {
    //   std::cout << "[" << this->Host() << "] Neighbors:" << std::endl;
    //   for (auto const &neighbor : myNeighbors)
    //     std::cout << "\t" << neighbor << std::endl;
    // }

    // The following chunk of code will pitch and yaw the camera.
    double camPitch, camYaw;
    this->CameraOrientation(camPitch, camYaw);
    camYaw += 0.001;
    camPitch -= 0.001;
    this->SetCameraOrientation(camPitch, camYaw);

    // Uncomment the following block to display the search area.
    // std::cout << "[" << this->Host() << "] search area: "
    //           << minLatitude << " " << maxLatitude << " "
    //           << minLongitude << " " << maxLongitude << std::endl;

    // Uncomment this line to show your current position according to the GPS.
    // std::cout << "[" << this->Host() << "] lat long alt: "
    //           << latitude << " " << longitude << " " << altitude
    //           << std::endl;

    // Get IMU information
    ignition::math::Vector3d linVel, angVel;
    ignition::math::Quaterniond orient;
    if (this->Imu(linVel, angVel, orient))
    {
      // Uncomment the following block to display the IMU information.
      // std::cout << "[" << this->Host() << "] Linear Vel: " << linVel
      //           << std::endl;
      // std::cout << "[" << this->Host() << "] Angular Vel: " << angVel
      //           << std::endl;
      // std::cout << "[" << this->Host() << "] Orientation: " << orient.Euler()
      //           << std::endl;
    }

    // Get bearing
    ignition::math::Angle bearing;
    if (this->Bearing(bearing))
    {
      // Uncomment the following block to display the compass information.
      // std::cout << "[" << this->Host() << "] Bearing: " << bearing
      //           << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void TeamControllerPlugin::OnDataReceived(const std::string &_srcAddress,
    const std::string &_dstAddress, const uint32_t _dstPort,
    const std::string &_data)
{
  // New data received from a teammate.
}
