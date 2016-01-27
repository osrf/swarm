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

#include <functional>
#include <gazebo/gazebo_config.h>
#include <gazebo/math/Vector2i.hh>
#include <gazebo/physics/physics.hh>
#include "swarm/LostPersonPlugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(LostPersonPlugin)

//////////////////////////////////////////////////
LostPersonPlugin::LostPersonPlugin()
{
}

//////////////////////////////////////////////////
LostPersonPlugin::~LostPersonPlugin()
{
  gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

//////////////////////////////////////////////////
void LostPersonPlugin::Load(sdf::ElementPtr /*_sdf*/)
{
}

//////////////////////////////////////////////////
void LostPersonPlugin::Load(gazebo::physics::ModelPtr _model,
                            sdf::ElementPtr _sdf)
{
  // Get the model
  this->model = _model;
  if (!this->model)
  {
    gzerr << "Invalid model pointer. LostPersonPlugin will not load.\n";
    return;
  }
  this->modelHeight2 = this->model->GetBoundingBox().GetZLength()*0.5;

  // We assume that the physics step size will not change during simulation.
  this->world = this->model->GetWorld();
  this->common.SetWorld(this->world);

  // Get the terrain, if it's present
  gazebo::physics::ModelPtr terrainModel =
    this->model->GetWorld()->GetModel("terrain");

  // Load some info about the terrain.
  if (terrainModel)
  {
    this->common.SetTerrain(
        boost::dynamic_pointer_cast<gazebo::physics::HeightmapShape>(
          terrainModel->GetLink()->GetCollision("collision")->GetShape()));
  }

  this->common.LoadSearchArea(_sdf->GetElement("swarm_search_area"));

  sdf::ElementPtr modelSDF = _sdf->GetParent();

  // We have the search area size.  Now get the origin, which is in
  // spherical_coordinates, a child of the world.
  sdf::ElementPtr worldSDF = modelSDF->GetParent();
  sdf::ElementPtr sphericalCoordsSDF =
    worldSDF->GetElement("spherical_coordinates");

  if (!this->common.LoadSphericalCoordinates(
        worldSDF->GetElement("spherical_coordinates")))
  {
    gzerr << "Unable to load spherical coordinates\n";
  }

  // Get the gps sensor
  if (_sdf->HasElement("gps"))
  {
#if GAZEBO_MAJOR_VERSION >= 7
    this->gps =
      std::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(
        gazebo::sensors::get_sensor(this->model->GetScopedName(true) + "::" +
          _sdf->Get<std::string>("gps")));
#else
    this->gps =
      boost::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(
        gazebo::sensors::get_sensor(this->model->GetScopedName(true) + "::" +
          _sdf->Get<std::string>("gps")));
#endif
  }

  if (!this->gps)
    gzwarn << "No gps sensor found on lost person" << std::endl;

  this->AdjustPose();

  this->Load(_sdf);

  // Listen to the update event broadcasted every simulation iteration.
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&LostPersonPlugin::Loop, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void LostPersonPlugin::Update(const gazebo::common::UpdateInfo &/*_info*/)
{
}

//////////////////////////////////////////////////
void LostPersonPlugin::Loop(const gazebo::common::UpdateInfo &_info)
{
  this->UpdateSensors();

  this->Update(_info);

  this->AdjustPose();
}

//////////////////////////////////////////////////
void LostPersonPlugin::AdjustPose()
{
  if (!this->common.Terrain() || !this->model)
    return;

  // Get the pose of the model
  ignition::math::Pose3d pose = this->model->GetWorldPose().Ign();

  // Constrain X position to the terrain boundaries
  pose.Pos().X(ignition::math::clamp(pose.Pos().X(),
        -this->common.TerrainSize().X() * 0.5,
         this->common.TerrainSize().X() * 0.5));

  // Constrain Y position to the terrain boundaries
  pose.Pos().Y(ignition::math::clamp(pose.Pos().Y(),
        -this->common.TerrainSize().Y() * 0.5,
         this->common.TerrainSize().Y() * 0.5));

  ignition::math::Vector3d norm;
  ignition::math::Vector3d terrainPos;
  this->common.TerrainLookup(pose.Pos(), terrainPos, norm);

  ignition::math::Vector3d euler = pose.Rot().Euler();

  // Project normal onto xy plane
  ignition::math::Vector3d norm2d(norm.X(), norm.Y(), 0);
  norm2d.Normalize();

  // Pitch vector
  ignition::math::Vector3d normPitchDir(
      cos(euler.Z()), sin(euler.Z()), 0);

  // Roll vector
  ignition::math::Vector3d normRollDir(sin(euler.Z()),
      -cos(euler.Z()), 0);

  // Compute pitch and roll
  double pitch = norm2d.Dot(normPitchDir) * acos(norm.Z());
  double roll = norm2d.Dot(normRollDir) * acos(norm.Z());

  // Add half the height of the model
  pose.Pos().Z(terrainPos.Z() + this->modelHeight2);
  pose.Rot().Euler(roll, pitch, pose.Rot().Euler().Z());

  // Set the pose.
  this->model->SetRelativePose(pose);
}

//////////////////////////////////////////////////
bool LostPersonPlugin::MapQuery(const double _lat, const double _lon,
    double &_height, TerrainType &_type)
{
  return this->common.MapQuery(_lat, _lon, _height, _type);
}

//////////////////////////////////////////////////
bool LostPersonPlugin::Pose(double &_latitude, double &_longitude,
    double &_altitude) const
{
  if (!this->gps)
  {
    gzerr << "LostPersonPlugin::Pose() No GPS sensor available" << std::endl;
    _latitude = _longitude = _altitude = 0.0;
    return false;
  }

  _latitude = this->latitude;
  _longitude = this->longitude;
  _altitude = this->altitude;

  return true;
}

//////////////////////////////////////////////////
void LostPersonPlugin::UpdateSensors()
{
  if (this->gps)
  {
    this->latitude = this->gps->Latitude().Degree();
    this->longitude = this->gps->Longitude().Degree();
#if GAZEBO_MAJOR_VERSION >= 7
    this->altitude = this->gps->Altitude();
#else
    this->altitude = this->gps->GetAltitude();
#endif
  }
}
