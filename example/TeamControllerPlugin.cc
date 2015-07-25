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
    else if (this->Host() == "192.168.2.2")
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
