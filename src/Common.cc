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

#include <ignition/math.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include "swarm/Common.hh"

using namespace swarm;

//////////////////////////////////////////////////
Common::Common()
  : searchMinLatitude(0),
    searchMaxLatitude(0),
    searchMinLongitude(0),
    searchMaxLongitude(0)
{
}

//////////////////////////////////////////////////
bool Common::LoadSearchArea(sdf::ElementPtr _searchAreaSDF)
{
  while (_searchAreaSDF)
  {
    if (_searchAreaSDF->HasElement("min_relative_latitude_deg") &&
        _searchAreaSDF->HasElement("max_relative_latitude_deg") &&
        _searchAreaSDF->HasElement("min_relative_longitude_deg") &&
        _searchAreaSDF->HasElement("max_relative_longitude_deg"))
    {
      this->searchMinLatitude =
        _searchAreaSDF->GetElement("min_relative_latitude_deg")->Get<double>();
      this->searchMaxLatitude =
        _searchAreaSDF->GetElement("max_relative_latitude_deg")->Get<double>();
      this->searchMinLongitude =
        _searchAreaSDF->GetElement("min_relative_longitude_deg")->Get<double>();
      this->searchMaxLongitude =
        _searchAreaSDF->GetElement("max_relative_longitude_deg")->Get<double>();

      return true;
    }
    _searchAreaSDF = _searchAreaSDF->GetNextElement("swarm_search_area");
  }

  return false;
}

/////////////////////////////////////////////////
bool Common::LoadSphericalCoordinates(sdf::ElementPtr _sphericalCoordsSDF)
{
  while (_sphericalCoordsSDF)
  {
    if (_sphericalCoordsSDF->HasElement("latitude_deg") &&
        _sphericalCoordsSDF->HasElement("longitude_deg"))
    {
      // Offset the search borders by the origin.
      this->searchMinLatitude +=
        _sphericalCoordsSDF->GetElement("latitude_deg")->Get<double>();
      this->searchMaxLatitude +=
        _sphericalCoordsSDF->GetElement("latitude_deg")->Get<double>();
      this->searchMinLongitude +=
        _sphericalCoordsSDF->GetElement("longitude_deg")->Get<double>();
      this->searchMaxLongitude +=
        _sphericalCoordsSDF->GetElement("longitude_deg")->Get<double>();

      return true;
    }

    _sphericalCoordsSDF =
      _sphericalCoordsSDF->GetNextElement("spherical_coordinates");
  }

  return false;
}

//////////////////////////////////////////////////
void Common::SearchArea(double &_minLatitude,
                        double &_maxLatitude,
                        double &_minLongitude,
                        double &_maxLongitude)
{
  _minLatitude = this->searchMinLatitude;
  _maxLatitude = this->searchMaxLatitude;
  _minLongitude = this->searchMinLongitude;
  _maxLongitude = this->searchMaxLongitude;
}

//////////////////////////////////////////////////
double Common::SearchMinLatitude() const
{
  return this->searchMinLatitude;
}

//////////////////////////////////////////////////
double Common::SearchMaxLatitude() const
{
  return this->searchMaxLatitude;
}

//////////////////////////////////////////////////
double Common::SearchMinLongitude() const
{
  return this->searchMinLongitude;
}

//////////////////////////////////////////////////
double Common::SearchMaxLongitude() const
{
  return this->searchMaxLongitude;
}

//////////////////////////////////////////////////
void Common::SetSearchMinLatitude(const double _value)
{
  this->searchMinLatitude = _value;
}

//////////////////////////////////////////////////
void Common::SetSearchMaxLatitude(const double _value)
{
  this->searchMaxLatitude = _value;
}

//////////////////////////////////////////////////
void Common::SetSearchMinLongitude(const double _value)
{
  this->searchMinLongitude = _value;
}

//////////////////////////////////////////////////
void Common::SetSearchMaxLongitude(const double _value)
{
  this->searchMaxLongitude = _value;
}

//////////////////////////////////////////////////
bool Common::MapQuery(const double _lat, const double _lon,
                      double &_height, TerrainType &_type)
{
  // Check that the lat and lon is in the search area
  if (_lat < this->searchMinLatitude || _lat > this->searchMaxLatitude ||
      _lon < this->searchMinLongitude || _lon > this->searchMaxLongitude)
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
void Common::TerrainLookup(const ignition::math::Vector3d &_pos,
    ignition::math::Vector3d &_terrainPos,
    ignition::math::Vector3d &_norm) const
{
  // The robot position in the coordinate frame of the terrain
  ignition::math::Vector3d robotPos(
      (this->terrainSize.X() * 0.5 + _pos.X()) / this->terrainScaling.X(),
      (this->terrainSize.Y() * 0.5 - _pos.Y()) / this->terrainScaling.Y(), 0);

  // Three vertices that define the triangle on which the vehicle rests
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

/////////////////////////////////////////////////
void Common::SetWorld(gazebo::physics::WorldPtr _world)
{
  this->world = _world;
}

/////////////////////////////////////////////////
void Common::SetTerrain(gazebo::physics::HeightmapShapePtr _terrain)
{
  this->terrain = _terrain;

  // Get the size of the terrain
  this->terrainSize = this->terrain->GetSize().Ign();

  // Set the terrain scaling.
  this->terrainScaling.Set(this->terrain->GetSize().x /
      (this->terrain->GetVertexCount().x-1),
      this->terrain->GetSize().y /
      (this->terrain->GetVertexCount().y-1));
}

/////////////////////////////////////////////////
TerrainType Common::TerrainAtPos(
    const ignition::math::Vector3d &_pos)
{
  TerrainType result = TerrainType::PLAIN;

  for (auto const &mdl : this->world->GetModels())
  {
    if (mdl->GetBoundingBox().Contains(_pos))
    {
      // The bounding box of a model is aligned to the global axis, and can
      // lead to incorrect results.
      // If a point is in the bounding box, then we use a ray-cast to see
      // if the point is actually within the model.
      gazebo::physics::ModelPtr rayModel = this->world->GetModelBelowPoint(
          gazebo::math::Vector3(_pos.X(), _pos.Y(), 1000));

      // Just in case rayModel is null
      const gazebo::physics::ModelPtr m = rayModel != NULL ? rayModel : mdl;

      if (m->GetName().find("tree") != std::string::npos)
      {
        result = TerrainType::FOREST;
        break;
      }
      else if (m->GetName().find("building") != std::string::npos)
      {
        result = TerrainType::BUILDING;
        break;
      }
    }
  }

  return result;
}

/////////////////////////////////////////////////
ignition::math::Vector3d Common::TerrainSize() const
{
  return this->terrainSize;
}

/////////////////////////////////////////////////
gazebo::physics::HeightmapShapePtr Common::Terrain() const
{
  return this->terrain;
}
