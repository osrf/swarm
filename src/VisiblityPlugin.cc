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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/program_options.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "swarm/VisibilityPlugin.hh"

using namespace gazebo;
namespace po = boost::program_options;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(VisibilityPlugin)

/////////////////////////////////////////////
VisibilityPlugin::~VisibilityPlugin()
{
}

/////////////////////////////////////////////
void VisibilityPlugin::Load(int /*_argc*/, char ** /*_argv*/)
{
  std::cout << "VisibilityPlugin::Load\n";

  this->searchMinLatitude = 35.6653;
  this->searchMaxLatitude = 35.8853;

  this->searchMinLongitude = -120.884;
  this->searchMaxLongitude = -120.664;

  // std::string outFile = "/home/nkoenig/visibility.txt";
}

/////////////////////////////////////////////
void VisibilityPlugin::Init()
{
  this->worldCreatedConn = event::Events::ConnectWorldCreated(
        boost::bind(&VisibilityPlugin::OnWorldCreated, this));
}

/////////////////////////////////////////////
void VisibilityPlugin::OnWorldCreated()
{
  this->updateConn = event::Events::ConnectWorldUpdateBegin(
      std::bind(&VisibilityPlugin::Update, this));

  this->world = physics::get_world();

  // Get the terrain, if it's present
  gazebo::physics::ModelPtr terrainModel = this->world->GetModel("terrain");

  // Load some info about the terrain.
  if (terrainModel)
  {
    this->terrain =
      boost::dynamic_pointer_cast<gazebo::physics::HeightmapShape>(
          terrainModel->GetLink()->GetCollision("collision")->GetShape());

  }
  else
    gzerr << "Invalid terrain\n";

  // This ray will be used in LineOfSight() for checking obstacles
  // between a pair of vehicles.
  this->ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
    this->world->GetPhysicsEngine()->CreateShape("ray",
      gazebo::physics::CollisionPtr()));
}

/////////////////////////////////////////////
void VisibilityPlugin::Update()
{
  static bool first = true;

  if (first)
  {
    // Get the size of the terrain
    this->terrainSize = this->terrain->GetSize().Ign();

    // Set the terrain scaling.
    this->terrainScaling.Set(this->terrain->GetSize().x /
        (this->terrain->GetVertexCount().x-1),
        this->terrain->GetSize().y /
        (this->terrain->GetVertexCount().y-1));

    int rowSize = 100;

    ignition::math::Vector3 startPos;
    ignition::math::Vector3 endPos;

    for (int y=-100; y < 100; y+=10)
    {
      startPos.Y(y);
      for (int x = -100; x < 100; x+= 10)
      {
        int index = y*rowSize+x;
        startPos.X(x);
        startPos.Z(this->HeightAt(x, y));


        for (int y2 = -100; y2 < 100; y2 += 10)
        {
          endPos.Y(y2);
          for (int x2 = -100; x2 < 100; x2 += 10)
          {
            int visible = 1;

            if (index != index2)
            {
              int index2 = y2*row2Size+x2;
              endPos.X(x2);
              endPos.Z(this->HeightAt(x2, y2));
              visibile = this->LineOfSight(startPos, endPos);
            }

            std::cout << "S[" << x << " " << y << " " << index << "] "
              << "E[" << x2 << " " << y2 << " " << index2 << "] "
              << "V[" << visible << "]\n";
          }
        }

      }
    }

    std::cout << "TerrainSize[" << this->terrainSize << "] Scal[" << this->terrainScaling << "]\n";
    first = false;
  }
}

//////////////////////////////////////////////////
void VisibilityPlugin::TerrainLookup(const ignition::math::Vector3d &_pos,
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

//////////////////////////////////////////////////
bool VisibilityPlugin::MapQuery(const double _lat, const double _lon,
    double &_height, TerrainType &_type)
{
  // Check that the lat and lon is in the search area
  if (_lat < this->searchMinLatitude  ||
      _lat > this->searchMaxLatitude ||
      _lon < this->searchMinLongitude ||
      _lon > this->searchMaxLongitude)
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

/////////////////////////////////////////////////
VisibilityPlugin::TerrainType VisibilityPlugin::TerrainAtPos(
    const ignition::math::Vector3d &_pos)
{
  TerrainType result = PLAIN;

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

      if (rayModel->GetName().find("tree") != std::string::npos)
      {
        result = FOREST;
        break;
      }
      else if (rayModel->GetName().find("building") != std::string::npos)
      {
        result = BUILDING;
        break;
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////
int VisibilityPlugin::LineOfSight(const ignition::math::Vector3d &_p1,
                             const ignition::math::Vector3d &_p2)
                             //std::vector<std::string> &_entities)
{
  std::string firstEntity;
  std::string lastEntity;
  double dist;

  // _entities.clear();

  this->ray->SetPoints(_p1, _p2);
  // Get the first obstacle from _p1 to _p2.
  this->ray->GetIntersection(dist, firstEntity);
  /*_entities.push_back(firstEntity);

  if (!firstEntity.empty())
  {
    this->ray->SetPoints(end, start);
    // Get the last obstacle from _p1 to _p2.
    this->ray->GetIntersection(dist, lastEntity);

    if (firstEntity != lastEntity)
      _entities.push_back(lastEntity);
  }*/

  if (firstEntity.empty())
    return 1;
  else
    return 0;
}

/////////////////////////////////////////////////
double VisibilityPlugin::HeightAt(double _x, double _y)
{
  double dist;
  std::string ent;

  startPos.Z(1000);

  this->ray->SetPoints(
      ignition::math::Vector3d(_x, _y, 1000,
      ignition::math::Vector3d(_x, _y, 0));
  this->ray->GetIntersection(dist, ent);

  return 1000 - dist;
}

