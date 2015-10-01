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

#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include "LostPersonControllerPlugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(LostPersonControllerPlugin)

//////////////////////////////////////////////////
LostPersonControllerPlugin::LostPersonControllerPlugin()
  : LostPersonPlugin(),
    speed(0.0)
{
}

//////////////////////////////////////////////////
void LostPersonControllerPlugin::Load(sdf::ElementPtr _sdf)
{
  // Add code here that should be executed once on load.

  // Example of reading a value from the SDF file.
  if (_sdf->HasElement("speed"))
    this->speed = _sdf->Get<double>("speed");
}

//////////////////////////////////////////////////
void LostPersonControllerPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  // Add code here that should be updated every iteration.

  // Apply a linear velocity to the model
  this->model->SetLinearVel(ignition::math::Vector3d(this->speed, 0, 0));
}
