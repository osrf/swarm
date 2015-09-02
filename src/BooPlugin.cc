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

#include <boost/algorithm/string.hpp>
#include <mutex>
#include <string>
#include <vector>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>
#include "swarm/BooPlugin.hh"
#include "msgs/personfound.pb.h"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(BooPlugin)

//////////////////////////////////////////////////
BooPlugin::BooPlugin()
  : RobotPlugin()
{
}

//////////////////////////////////////////////////
BooPlugin::~BooPlugin()
{
  gazebo::event::Events::DisconnectWorldUpdateEnd(this->updateEndConnection);
}

//////////////////////////////////////////////////
void BooPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  RobotPlugin::Load(_model, _sdf);

  // Sanity check.
  if (this->Host() != this->kBoo)
  {
    gzerr << "BooPlugin::Load(): Please, use <address>boo</address>."
          << " Ignoring address, using 'boo'." << std::endl;
  }

  // Read the <lost_person_model> SDF parameter.
  if (!_sdf->HasElement("lost_person_model"))
  {
    gzerr << "BooPlugin::Load(): Unable to find the <lost_person_model> "
          << "parameter" << std::endl;
    return;
  }

  this->node.Advertise("/swarm/found");

  auto modelName = _sdf->Get<std::string>("lost_person_model");
  this->lostPerson = this->model->GetWorld()->GetModel(modelName);
  GZ_ASSERT(this->lostPerson, "Victim's model not found");

  // Initialize the position of the lost person.
  this->lostPersonPose = this->lostPerson->GetWorldPose().Ign().Pos();

  // Bind on my BOO address and default BOO port.
  this->Bind(&BooPlugin::OnDataReceived, this, this->kBoo, this->kBooPort);

  // Listen to the OnWorldUpdateEnd event broadcasted every simulation iteration
  this->updateEndConnection = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&BooPlugin::OnUpdateEnd, this));
}

//////////////////////////////////////////////////
void BooPlugin::OnUpdateEnd()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->lostPersonPose = this->lostPerson->GetWorldPose().Ign().Pos();
}

//////////////////////////////////////////////////
void BooPlugin::OnDataReceived(const std::string &_srcAddress,
    const std::string &_data)
{
  // Check if a robot found the lost person.
  // The format of this message should be: <cmd> [args].
  //
  // List of supported commands:
  // <FOUND> <x> <y> <z> <t> : Person found in [x,y,z] at time t.

  // Split the string into a vector of parameters.
  std::vector<std::string> v;
  std::string data = _data;
  boost::trim_if(data, boost::is_any_of("\t "));
  boost::split(v, data, boost::is_any_of("\t "), boost::token_compress_on);

  if (v.empty())
  {
    gzerr << "BooPlugin::OnDataReceived() Unable to parse incoming message ["
          << _data << "]" << std::endl;
    return;
  }

  auto cmd = v.at(0);
  if (cmd == "FOUND")
  {
    // Sanity check.
    if (v.size() != 5)
    {
      gzerr << "BooPlugin::OnDataReceived() Unable to parse a FOUND message ["
            << _data << "]" << std::endl << "Make sure that you're sending the"
            << " message in the proper format: FOUND <x> <y> <z>" << std::endl;
      return;
    }

    ignition::math::Vector3d poseReceived;
    try
    {
      double x = std::stod(v.at(1));
      double y = std::stod(v.at(2));
      double z = std::stod(v.at(3));
      double t = std::stod(v.at(4));
      poseReceived.Set(x, y, z);
    }
    catch(const std::invalid_argument &_e)
    {
      gzerr << "BooPlugin::OnDataReceived() Unable to parse the FOUND arguments"
            << " [" << v.at(1) << "," << v.at(2) << "," << v.at(3) << ","
            << v.at(4) << "]" << std::endl;
      return;
    }

    // Validate the result.
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (poseReceived.Distance(this->lostPersonPose) <= this->kTolerance)
      {
        found = true;
        gzdbg << "Congratulations! Robot [" << _srcAddress << "] has found "
              << "the lost person" << std::endl;

        swarm::msgs::PersonFound msg;
        msg.set_address(_srcAddress);
        msg.mutable_pos()->set_x(poseReceived.X());
        msg.mutable_pos()->set_y(poseReceived.Y());
        msg.mutable_pos()->set_z(poseReceived.Z());
        this->node.Publish("/swarm/found", msg);

        // Pause the simulation to make the lost person detection obvious.
        gazebo::physics::get_world()->SetPaused(true);
      }
      else
        gzerr << "Sorry, the reported position seems incorrect" << std::endl;
    }
  }
  else
  {
    gzerr << "BooPlugin::OnDataReceived() Unrecognized command [" << v.at(0)
          << "]" << std::endl;
  }
}
