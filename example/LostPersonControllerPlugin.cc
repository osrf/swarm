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

#include <gazebo/physics/physics.hh>
#include <ignition/math/Rand.hh>
#include "LostPersonControllerPlugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(LostPersonControllerPlugin)

//////////////////////////////////////////////////
LostPersonControllerPlugin::LostPersonControllerPlugin()
  : LostPersonPlugin()
{
}

//////////////////////////////////////////////////
void LostPersonControllerPlugin::Load(sdf::ElementPtr _sdf)
{
  // Add code here that should be executed once on load.

  // Set the initial velocity, if present
  if (_sdf->HasElement("initial_velocity"))
    this->velocity = _sdf->Get<ignition::math::Vector3d>("initial_velocity");

  // Set the velocity factor
  if (_sdf->HasElement("velocity_factor"))
    this->velocityFactor = _sdf->Get<double>("velocity_factor");

  // Set the update period
  if (_sdf->HasElement("update_period"))
    this->updatePeriod = _sdf->Get<double>("update_period");
}

//////////////////////////////////////////////////
void LostPersonControllerPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  // Add code here that should be updated every iteration.

  // Change direction when enough time has elapsed
  if (_info.simTime - this->prevUpdate > this->updatePeriod)
  {

    // Get a random velocity value.
    this->velocity.Set(
        ignition::math::Rand::DblUniform(-1, 1),
        ignition::math::Rand::DblUniform(-1, 1),
        ignition::math::Rand::DblUniform(-1, 1));

    // Apply scaling factor
    this->velocity.Normalize();
    this->velocity *= this->velocityFactor;

    this->prevUpdate = _info.simTime;
  }

  // Apply velocity
  this->model->SetLinearVel(this->velocity);
}

//////////////////////////////////////////////////
void LostPersonControllerPlugin::Reset()
{
  // Add code to handle simulation reset.
  this->prevUpdate.Set(0, 0);
}
