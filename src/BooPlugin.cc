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
#include <string>
#include <vector>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>
#include "swarm/BooPlugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(BooPlugin)

//////////////////////////////////////////////////
BooPlugin::BooPlugin()
  : RobotPlugin()
{
}

//////////////////////////////////////////////////
void BooPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Read the <lost_person_model> SDF parameter.
  if (!_sdf->HasElement("lost_person_model"))
  {
    gzerr << "BooPlugin::Load(): Unable to find the <lost_person_model> "
          << "parameter" << std::endl;
    return;
  }

  auto modelName = _sdf->Get<std::string>("lost_person_model");
  this->lostPerson = _model->GetWorld()->GetModel(modelName);
  GZ_ASSERT(this->lostPerson, "Victim's model not found");

  // Initialize the position of the lost person.
  this->prevLostPersonPose = this->lostPerson->GetWorldPose().Ign().Pos();

  // Bind on my local address and default port.
  this->Bind(&BooPlugin::OnDataReceived, this, this->Host());

  // Listen to the OnWorldUpdateEnd event broadcasted every simulation iteration
  this->updateEndConnection = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&BooPlugin::OnUpdateEnd, this));
}

//////////////////////////////////////////////////
void BooPlugin::OnUpdateEnd()
{
  this->prevLostPersonPose = this->lostPerson->GetWorldPose().Ign().Pos();
}

//////////////////////////////////////////////////
void BooPlugin::OnDataReceived(const std::string &_srcAddress,
    const std::string &_data)
{
  // Check if a robot found the lost person.
  // The format of this message should be: <cmd> [args].
  //
  // List of supported commands:
  // <FOUND> <x> <y> <z> : The lost person has been found at [x,y,z].

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
    if (v.size() != 4)
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
      double y = std::stod(v.at(1));
      double z = std::stod(v.at(1));
      poseReceived.Set(x, y, z);
    }
    catch(const std::invalid_argument &_e)
    {
      gzerr << "BooPlugin::OnDataReceived() Unable to parse the FOUND arguments"
            << " [" << v.at(1) << "," << v.at(2) << "," << v.at(3) << "]"
            << std::endl;
      return;
    }

    // Validate the result.
    if (poseReceived == this->prevLostPersonPose)
      gzdbg << "Congratulations! The person has been found!" << std::endl;
    else
      gzerr << "Sorry, the reported position seems incorrect" << std::endl;
  }
  else
  {
    gzerr << "BooPlugin::OnDataReceived() Unrecognized command [" << v.at(0)
          << "]" << std::endl;
  }

  gzdbg << "Robot " << _srcAddress << " found the lost person!" << std::endl;
}
