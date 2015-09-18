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
#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>
#include "swarm/BooPlugin.hh"
#include "swarm/SwarmTypes.hh"
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
    gzerr << "BooPlugin::Load() error: Please, use <address>boo</address>."
          << std::endl;
    gazebo::shutdown();
  }

  // Read the <lost_person_model> SDF parameter.
  if (!_sdf->HasElement("lost_person_model"))
  {
    gzerr << "BooPlugin::Load() error: Unable to find the <lost_person_model> "
          << "parameter" << std::endl;
    gazebo::shutdown();
  }

  // Read the <max_dt> SDF parameter.
  if (_sdf->HasElement("max_dt"))
  {
    auto dt = _sdf->Get<double>("max_dt");
    this->maxDt.Set(dt);
  }

  // Read the <cell_size> SDF parameter.
  if (_sdf->HasElement("cell_size"))
    this->cellSize = _sdf->Get<double>("cell_size");

  this->node.Advertise("/swarm/found");

  auto modelName = _sdf->Get<std::string>("lost_person_model");
  this->lostPerson = this->model->GetWorld()->GetModel(modelName);
  GZ_ASSERT(this->lostPerson, "Victim's model not found");

  // Initialize the position of the lost person.
  auto personPos = this->lostPerson->GetWorldPose().Ign().Pos();
  auto personPosInGrid = this->PosToGrid(personPos);
  this->lostPersonBuffer[gazebo::common::Time::Zero] = personPosInGrid;
  this->lastPersonPosInGrid = personPosInGrid;

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

  // Translate the person's position to a grid cell.
  auto personPos = this->lostPerson->GetWorldPose().Ign().Pos();
  auto personPosInGrid = this->PosToGrid(personPos);
  auto now = gazebo::physics::get_world()->GetSimTime();

  // Check the buffer of stored lost person's poses.
  while (!this->lostPersonBuffer.empty())
  {
    auto oldestEntry = this->lostPersonBuffer.lower_bound(
      gazebo::common::Time::Zero);
    auto oldestTime(oldestEntry->first);

    auto boundaryTime = now - this->maxDt;
    if (boundaryTime <= oldestTime)
      break;

    auto nextEntry = this->lostPersonBuffer.upper_bound(oldestTime);
    if ((nextEntry == this->lostPersonBuffer.end()) ||
        (nextEntry->first > boundaryTime))
      break;

    // Remove old entry.
    this->lostPersonBuffer.erase(oldestEntry);
  }

  GZ_ASSERT(!this->lostPersonBuffer.empty(),
            "Buffer of lost person's positions is empty");

  // The lost person has changed the cell.
  if (personPosInGrid != this->lastPersonPosInGrid)
  {
    this->lostPersonBuffer[now] = personPosInGrid;
    this->lastPersonPosInGrid = personPosInGrid;
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3i BooPlugin::PosToGrid(ignition::math::Vector3d _pos)
{
  int cellX, cellY, cellZ;

  if (_pos.X() >= 0)
    cellX = trunc((_pos.X() + this->cellSize / 2.0) / this->cellSize);
  else
    cellX = trunc((_pos.X() - this->cellSize / 2.0) / this->cellSize);

  if (_pos.Y() >= 0)
    cellY = trunc((_pos.Y() + this->cellSize / 2.0) / this->cellSize);
  else
    cellY = trunc((_pos.Y() - this->cellSize / 2.0) / this->cellSize);

  if (_pos.Z() >= 0)
    cellZ = trunc((_pos.Z() + this->cellSize / 2.0) / this->cellSize);
  else
    cellZ = trunc((_pos.Z() - this->cellSize / 2.0) / this->cellSize);

  return ignition::math::Vector3i(cellX, cellY, cellZ);
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

    ignition::math::Vector3d reportedPos;
    gazebo::common::Time t;
    try
    {
      double x = std::stod(v.at(1));
      double y = std::stod(v.at(2));
      double z = std::stod(v.at(3));
      t.Set(std::stod(v.at(4)));
      reportedPos.Set(x, y, z);
    }
    catch(const std::invalid_argument &_e)
    {
      gzerr << "BooPlugin::OnDataReceived() Unable to parse the FOUND arguments"
            << " [" << v.at(1) << "," << v.at(2) << "," << v.at(3) << ","
            << v.at(4) << "]" << std::endl;
      return;
    }

    // Sanity check: Time t cannot be negative.
    if (t < gazebo::common::Time::Zero)
    {
      gzerr << "BooPlugin::OnDataReceived() The reported time [" << t << "] is"
            << " negative. Your request will be ignored" << std::endl;
      return;
    }

    // Sanity check: Time t cannot be greater than the current time.
    auto now = gazebo::physics::get_world()->GetSimTime();
    if (t > now)
    {
      gzerr << "BooPlugin::OnDataReceived() The reported time [" << t << "] is"
            << " in the future. We're at [" << now << "]. Your request will be"
            << " ignored" << std::endl;
      return;
    }

    // Sanity check: Time t cannot be older than current time - maxDt.
    if (t < (now - this->maxDt))
    {
      gzerr << "BooPlugin::OnDataReceived() The reported time [" << t << "] is"
            << " too old. It should be no older than "
            << this->maxDt.Double() << " secs. We're at [" << now
            << "]. Your request will be ignored" << std::endl;
      return;
    }

    // Validate the result.
    auto reportedPosInGrid = this->PosToGrid(reportedPos);
    ignition::math::Vector3i realPosInGrid;

    {
      std::lock_guard<std::mutex> lock(this->mutex);
      auto nextRealEntry = this->lostPersonBuffer.upper_bound(t);
      GZ_ASSERT(nextRealEntry != this->lostPersonBuffer.begin(),
                "Unexpected iterator");
      --nextRealEntry;
      realPosInGrid = nextRealEntry->second;
    }

    if (reportedPosInGrid == realPosInGrid)
    {
      found = true;
      gzdbg << "Congratulations! Robot [" << _srcAddress << "] has found "
            << "the lost person at time [" << t << "]" << std::endl;

      swarm::msgs::PersonFound msg;
      msg.set_address(_srcAddress);
      msg.mutable_pos()->set_x(reportedPos.X());
      msg.mutable_pos()->set_y(reportedPos.Y());
      msg.mutable_pos()->set_z(reportedPos.Z());
      msg.set_time(t.Double());
      this->node.Publish("/swarm/found", msg);

      // Pause the simulation to make the lost person detection obvious.
      gazebo::physics::get_world()->SetPaused(true);
    }
    else
      gzerr << "Sorry, the reported position seems incorrect" << std::endl;
  }
  else
  {
    gzerr << "BooPlugin::OnDataReceived() Unrecognized command [" << v.at(0)
          << "]" << std::endl;
  }
}
