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

#include "tbb/tbb.h"

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

  // The version of multiray shape only works with ODE>
  if (this->world->GetPhysicsEngine()->GetType() == "ode")
  {
    this->multiRay =
      boost::dynamic_pointer_cast<gazebo::physics::MultiRayShape>(
          this->world->GetPhysicsEngine()->CreateShape("multiray",
            gazebo::physics::CollisionPtr()));
  }
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

    double heightOffset = 1.0;
    int stepSize = 10;
    int range[2] = {-1000, 1000};
    int rowSize = (range[1] / stepSize) - (range[0] / stepSize) + 1;

    ignition::math::Vector3d startPos;
    ignition::math::Vector3d endPos;

    std::map<uint64_t, int> visibilityMap;
    std::map<uint64_t, double> heights;

    int rayCount = 0;

    // TODO: Check that rowSize is divisible by 2.

    int maxRays = 1000;

    // Create all the rays up front

    if (this->multiRay)
    {
      for (int i = 0; i < maxRays; i++)
      {
        this->multiRay->AddRay(ignition::math::Vector3d::Zero,
            ignition::math::Vector3d::Zero);
      }
    }

    // Store height values.
    for (int y = range[0]; y <= range[1]; y += stepSize)
    {
      for (int x = range[0]; x <= range[1]; x += stepSize)
      {
        int index = ((y + range[1])/stepSize) * rowSize +
          ((x + range[1])/stepSize);

        heights[index] =  this->HeightAt(x, y) + heightOffset;
      }
    }

    int testCount = 0;

    auto startTime =
      std::chrono::system_clock::now().time_since_epoch();

    // Iterate over the possible y values.
    for (int y = range[0]; y <= range[1]; y += stepSize)
    {
      //startPos.Y(y);
      // Iterate over the possible x values.
      for (int x = range[0]; x <= range[1]; x += stepSize)
      {
        int index = this->Index(x, y, range[1], stepSize, rowSize);
        //startPos.X(x);

        // Get the height at the start pos, plus some offset so that it
        // is not in the terrain
        //startPos.Z(this->HeightAt(x, y) + heightOffset);
        // double z =

        // The inner loops checks visibility from startPos to endPos
        for (int y2 = y; y2 <= range[1]; y2 += stepSize)
        {
          int rx = range[0];
          if (y2 == y)
            rx = x;
          //endPos.Y(y2);
          for (int x2 = rx; x2 <= range[1]; x2 += stepSize)
          {
            int index2 = this->Index(x2, y2, range[1], stepSize, rowSize);
            double dist = sqrt((x2-x)*(x2-x) + (y2-y)*(y2-y));

            // Skip values that are too far away
            if (dist > 280)
              continue;

            // The same location is always visible
            if (index != index2)
            {
              if (this->multiRay)
              {
                this->multiRay->SetRay(rayCount++,
                    ignition::math::Vector3d(x, y, heights[index]),
                    ignition::math::Vector3d(x2, y2, heights[index2]));
                if (rayCount >= maxRays)
                {
                  this->multiRay->UpdateRays();
                  rayCount = 0;
                }
              }
              else
              {
                int visible = this->LineOfSight(
                    ignition::math::Vector3d(x, y, heights[index]),
                    ignition::math::Vector3d(x2, y2, heights[index2]));

                std::cout << "V[" << visible << "]\n";
                // Only store values that are not visible
                if (visible < 1)
                {
                  visibilityMap[this->Key(index, index2)] = visible;
                }
              }
            }

            // Only store values that are not visible

            // std::cout << "S[" << x << " " << y << " " << startPos.Z() << " "
            //   << index << "] "
            //   << "E[" << x2 << " " << y2 << " " << endPos.Z() << " "
            //   << index2 << "] V[" << visible << "]  K[" << key << "]\n";
          }
        }
      }
    }
    auto endTime = std::chrono::system_clock::now().time_since_epoch();

    auto duration = endTime - startTime;
    std::cout << "Duration["
      << std::chrono::duration_cast<std::chrono::seconds>(duration).count()
      << "]\n";

    std::cout << "TestCount[" << testCount << "]\n";
    std::cout << "Size[" << visibilityMap.size() << "] Bytes[" <<
      (sizeof(uint64_t) + sizeof(int)) * visibilityMap.size() << "]\n";

    /*for (double i = -100; i < 100; i +=10)
    {
      int lu = this->Lookup(
          ignition::math::Vector3d(0, i, 0),
          ignition::math::Vector3d(50, i, 0));
      lu = lu;
    }*/
    first = false;
  }
}

//////////////////////////////////////////////////
int VisibilityPlugin::Lookup(const ignition::math::Vector3d &_start,
    const ignition::math::Vector3d &_end)
{
  int stepSize = 100;
  int range[2] = {-100, 100};
  int rowSize = (range[1] / stepSize) - (range[0] / stepSize) + 1;

  int startX = std::round(_start.X() / 100.0) * 100;
  int startY = std::round(_start.Y() / 100.0) * 100;

  int endX = std::round(_end.X() / 100.0) * 100;
  int endY = std::round(_end.Y() / 100.0) * 100;

  int index = ((startY + range[1])/stepSize) * rowSize +
    ((startX + range[1])/stepSize);

  int index2 = ((endY + range[1])/stepSize) * rowSize +
    ((endX + range[1])/stepSize);

  // Szudzik's function
  uint64_t key = this->Key(index, index2);

  std::cout << "Start[" << startX << " " << startY << "] End["
            << endX << " " << endY << "] I[" << index << "] I2[" << index2
            << "] K[" << key << "]\n";

  return 0;
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
double VisibilityPlugin::HeightAt(const double _x, const double _y) const
{
  double dist;
  std::string ent;

  this->ray->SetPoints(
      ignition::math::Vector3d(_x, _y, 1000),
      ignition::math::Vector3d(_x, _y, 0));
  this->ray->GetIntersection(dist, ent);

  return 1000 - dist;
}

/////////////////////////////////////////////////
uint64_t VisibilityPlugin::Key(int _a, int _b)
{
  // Szudzik's function
  return _a >= _b ?  _a * _a + _a + _b : _a + _b * _b;
}

/////////////////////////////////////////////////
int VisibilityPlugin::Index(int _x, int _y, int _maxY, int _stepSize, int _rowSize)
{
  return ((_y + _maxY)/_stepSize) * _rowSize + ((_x + _maxY)/_stepSize);
}
