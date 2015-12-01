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

  // Get the terrain, if it's present
  gazebo::physics::ModelPtr terrainModel =
    this->model->GetWorld()->GetModel("terrain");

  // Load some info about the terrain.
  if (terrainModel)
  {
    this->terrain =
      boost::dynamic_pointer_cast<gazebo::physics::HeightmapShape>(
          terrainModel->GetLink()->GetCollision("collision")->GetShape());

    // Get the size of the terrain
    this->terrainSize = this->terrain->GetSize().Ign();

    // Set the terrain scaling.
    this->terrainScaling.Set(this->terrain->GetSize().x /
        (this->terrain->GetVertexCount().x-1),
        this->terrain->GetSize().y /
        (this->terrain->GetVertexCount().y-1));
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
    this->gps =
      boost::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(
        gazebo::sensors::get_sensor(this->model->GetScopedName(true) + "::" +
          _sdf->Get<std::string>("gps")));
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
  if (!this->terrain || !this->model)
    return;

  // Get the pose of the model
  ignition::math::Pose3d pose = this->model->GetWorldPose().Ign();

  // Constrain X position to the terrain boundaries
  pose.Pos().X(ignition::math::clamp(pose.Pos().X(),
        -this->terrainSize.X() * 0.5, this->terrainSize.X() * 0.5));

  // Constrain Y position to the terrain boundaries
  pose.Pos().Y(ignition::math::clamp(pose.Pos().Y(),
        -this->terrainSize.Y() * 0.5, this->terrainSize.Y() * 0.5));

  ignition::math::Vector3d norm;
  ignition::math::Vector3d terrainPos;
  this->TerrainLookup(pose.Pos(), terrainPos, norm);

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
void LostPersonPlugin::TerrainLookup(const ignition::math::Vector3d &_pos,
    ignition::math::Vector3d &_terrainPos,
    ignition::math::Vector3d &_norm) const
{
  // The model position in the coordinate frame of the terrain
  ignition::math::Vector3d robotPos(
      (this->terrainSize.X() * 0.5 + _pos.X()) / this->terrainScaling.X(),
      (this->terrainSize.Y() * 0.5 - _pos.Y()) / this->terrainScaling.Y(), 0);

  // Three vertices that define the triangle on which the model rests
  // The first vertex is closest point on the terrain
  ignition::math::Vector3d v1(std::round(robotPos.X()),
      std::round(robotPos.Y()), 0);
  ignition::math::Vector3d v2 = v1;
  ignition::math::Vector3d v3 = v1;

  // The second and third vertices are chosen based on how OGRE layouts
  // the triangle strip.
  if (static_cast<int>(v1.X()) == static_cast<int>(std::ceil(robotPos.X())) &&
      static_cast<int>(v1.Y()) == static_cast<int>(std::ceil(robotPos.Y())))
  {
    if (static_cast<int>(v1.Y()) % 2 == 0)
    {
      v2.Y(v1.Y()-1);
      v3.X(v1.X()-1);
    }
    else
    {
      ignition::math::Vector3d b(v1.X()-1, v1.Y(), 0);
      ignition::math::Vector3d c(v1.X(), v1.Y()-1, 0);
      if (robotPos.Distance(b) < robotPos.Distance(c))
      {
        v3 = b;
        v2.X(v1.X()-1);
        v2.Y(v1.Y()-1);
      }
      else
      {
        v2 = c;
        v3.X(v1.X()-1);
        v3.Y(v1.Y()-1);
      }
    }
  }
  else if (static_cast<int>(v1.X()) ==
      static_cast<int>(std::floor(robotPos.X())) &&
      static_cast<int>(v1.Y()) == static_cast<int>(std::ceil(robotPos.Y())))
  {
    if (static_cast<int>(v1.Y()) % 2 == 0)
    {
      ignition::math::Vector3d b(v1.X()+1, v1.Y(), 0);
      ignition::math::Vector3d c(v1.X(), v1.Y()-1, 0);
      if (robotPos.Distance(b) < robotPos.Distance(c))
      {
        v2 = b;
        v3.X(v1.X()+1);
        v3.Y(v1.Y()-1);
      }
      else
      {
        v3 = c;
        v2.X(v1.X()+1);
        v2.Y(v1.Y()-1);
      }
    }
    else
    {
      v2.X(v1.X()+1);
      v3.Y(v1.Y()-1);
    }
  }
  else if (static_cast<int>(v1.X()) ==
      static_cast<int>(std::floor(robotPos.X())) &&
      static_cast<int>(v1.Y()) == static_cast<int>(std::floor(robotPos.Y())))
  {
    if (static_cast<int>(v1.Y()) % 2 == 0)
    {
      ignition::math::Vector3d b(v1.X()+1, v1.Y(), 0);
      ignition::math::Vector3d c(v1.X(), v1.Y()+1, 0);
      if (robotPos.Distance(b) < robotPos.Distance(c))
      {
        v2.X(v1.X()+1);
        v2.Y(v1.Y()+1);
        v3 = b;
      }
      else
      {
        v2 = c;
        v3.X(v1.X()+1);
        v3.Y(v1.Y()+1);
      }
    }
    else
    {
      v2.Y(v1.Y()+1);
      v3.X(v1.X()+1);
    }
  }
  else
  {
    if (static_cast<int>(v1.Y()) % 2 == 0)
    {
      v2.X() -= 1;
      v3.Y() += 1;
    }
    else
    {
      ignition::math::Vector3d b(v1.X()-1, v1.Y(), 0);
      ignition::math::Vector3d c(v1.X(), v1.Y()+1, 0);
      if (robotPos.Distance(b) < robotPos.Distance(c))
      {
        v2 = b;
        v3.X(v1.X()-1);
        v3.Y(v1.Y()+1);
      }
      else
      {
        v2.X(v1.X()-1);
        v2.Y(v1.Y()+1);
        v3 = c;
      }
    }
  }

  // Get the height at each vertex
  v1.Z(this->terrain->GetHeight(v1.X(), v1.Y()));
  v2.Z(this->terrain->GetHeight(v2.X(), v2.Y()));
  v3.Z(this->terrain->GetHeight(v3.X(), v3.Y()));

  // Display a marker that highlights the vertices currently used to
  // compute the vehicles height. This is debug code that is very useful
  // but it requires a version of gazebo with visual markers.
  //
  // gazebo::msgs::Marker markerMsg;
  // markerMsg.set_layer("default");
  // markerMsg.set_id(0);
  // markerMsg.set_action(gazebo::msgs::Marker::MODIFY);
  // markerMsg.set_type(gazebo::msgs::Marker::LINE_STRIP);


  // v1a.Z() += 0.1;
  // v2a.Z() += 0.1;
  // v3a.Z() += 0.1;
  // gazebo::msgs::Set(markerMsg.add_point(), v1a);
  // gazebo::msgs::Set(markerMsg.add_point(), v2a);
  // gazebo::msgs::Set(markerMsg.add_point(), v3a);
  // if (this->markerPub)
  //   this->markerPub->Publish(markerMsg);
  // END DEBUG CODE

  ignition::math::Vector3d v1a = v1;
  ignition::math::Vector3d v2a = v2;
  ignition::math::Vector3d v3a = v3;
  v1a.X(v1a.X()*this->terrainScaling.X() - this->terrainSize.X()*0.5);
  v1a.Y(this->terrainSize.Y()*0.5 - v1a.Y()*this->terrainScaling.Y());

  v2a.X(v2a.X()*this->terrainScaling.X() - this->terrainSize.X()*0.5);
  v2a.Y(this->terrainSize.Y()*0.5 - v2a.Y()*this->terrainScaling.Y());

  v3a.X(v3a.X()*this->terrainScaling.X() - this->terrainSize.X()*0.5);
  v3a.Y(this->terrainSize.Y()*0.5 - v3a.Y()*this->terrainScaling.Y());

  _norm = ignition::math::Vector3d::Normal(v1a, v2a, v3a);

  // Triangle normal
  ignition::math::Vector3d norm = ignition::math::Vector3d::Normal(v1, v2, v3);

  // Ray direction to intersect with triangle
  ignition::math::Vector3d rayDir(0, 0, -1);

  // Ray start point
  ignition::math::Vector3d rayPt(robotPos.X(), robotPos.Y(), 1000);

  // Distance from ray start to triangle intersection
  double intersection = -norm.Dot(rayPt - v1) / norm.Dot(rayDir);

  // Height of the terrain
  _terrainPos = rayPt + intersection * rayDir;
}

//////////////////////////////////////////////////
bool LostPersonPlugin::MapQuery(const double _lat, const double _lon,
    double &_height, LostPersonPlugin::TerrainType &_type)
{
  // Check that the lat and lon is in the search area
  if (_lat < this->common.SearchMinLatitude()  ||
      _lat > this->common.SearchMaxLatitude() ||
      _lon < this->common.SearchMinLongitude() ||
      _lon > this->common.SearchMaxLongitude())
  {
    return false;
  }

  // Get the location in the local coordinate frame
  ignition::math::Vector3d local =
    this->world->GetSphericalCoordinates()->LocalFromSpherical(
        ignition::math::Vector3d(_lat, _lon, 0));

  local = this->world->GetSphericalCoordinates()->GlobalFromLocal(local);

  ignition::math::Vector3d pos, norm;

  // Reuse the terrain lookup function.
  this->TerrainLookup(local, pos, norm);

  // Add in the reference elevation.
  _height = pos.Z() +
    this->world->GetSphericalCoordinates()->GetElevationReference();
  local.Z(pos.Z());

  _type = this->TerrainAtPos(local);

  return true;
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

/////////////////////////////////////////////////
LostPersonPlugin::TerrainType LostPersonPlugin::TerrainAtPos(
    const ignition::math::Vector3d &_pos)
{
  LostPersonPlugin::TerrainType result = PLAIN;

  for (auto const &mdl : this->world->GetModels())
  {
    if (mdl->GetBoundingBox().Contains(_pos))
    {
      if (mdl->GetName().find("tree") != std::string::npos)
      {
        result = FOREST;
        break;
      }
      else if (mdl->GetName().find("building") != std::string::npos)
      {
        result = BUILDING;
        break;
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////
void LostPersonPlugin::UpdateSensors()
{
  if (this->gps)
  {
    this->latitude = this->gps->Latitude().Degree();
    this->longitude = this->gps->Longitude().Degree();
    this->altitude = this->gps->GetAltitude();
  }
}
