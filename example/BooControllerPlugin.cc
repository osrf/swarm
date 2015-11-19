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
#include "BooControllerPlugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(BooControllerPlugin)

//////////////////////////////////////////////////
BooControllerPlugin::BooControllerPlugin()
  : BooPlugin()
{
}

//////////////////////////////////////////////////
void BooControllerPlugin::Load(sdf::ElementPtr _sdf)
{
}

//////////////////////////////////////////////////
void BooControllerPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
}

//////////////////////////////////////////////////
void BooControllerPlugin::OnData(const std::string &_srcAddress,
    const std::string &_dstAddress, const uint32_t _dstPort,
    const std::string &_data)
{
  // New data received from a vehicle.

  // The default behavior is to send and ack.
  gzerr << "BooPlugin::OnDataReceived() Unable to parse incoming message ["
        << _data << "]" << std::endl;
  this->SendAck(_srcAddress, 3);
}

//////////////////////////////////////////////////
void BooControllerPlugin::Reset()
{
  // Pass reset up the chain.
  BooPlugin::Reset();

  // Add code to handle simulation reset.
  // Make sure to always call BooPlugin::Reset();
}
