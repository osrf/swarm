/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <fstream>

#include "gazebo/gazebo_config.h"
#include "gazebo/physics/physics.hh"
#include "swarm/VisibilityPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(VisibilityPlugin)

/////////////////////////////////////////////
VisibilityPlugin::~VisibilityPlugin()
{
}

/////////////////////////////////////////////
void VisibilityPlugin::Load(int /*_argc*/, char ** /*_argv*/)
{
  this->range = {-20000, 20000};
  this->stepSize = 10;
  this->maxY = this->range[1];
  this->rowSize = (this->range[1] - this->range[0]) / stepSize + 1;
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

  physics::WorldPtr world = physics::get_world();

  // This ray will be used in LineOfSight() for checking obstacles
  // between a pair of vehicles.
  this->ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      world->GetPhysicsEngine()->CreateShape("ray",
      gazebo::physics::CollisionPtr()));
}

/////////////////////////////////////////////
void VisibilityPlugin::Update()
{
  std::map<uint64_t, int> visibilityMap;
  std::map<uint64_t, double> heights;

  // Cache height values for efficiency.
  for (int y = this->range[0]; y <= this->range[1]; y += this->stepSize)
  {
    for (int x = this->range[0]; x <= this->range[1]; x += this->stepSize)
    {
      heights[this->Index(x, y)] =  this->HeightAt(x, y);
    }
  }

  std::string outFilename = "/tmp/visibility.dat";
  std::fstream out(outFilename, std::ios::out | std::ios::binary);

  // Save info about the visibility table
  out.write(reinterpret_cast<const char*>(&this->range[1]), sizeof(int));
  out.write(reinterpret_cast<const char*>(&this->stepSize), sizeof(int));
  out.write(reinterpret_cast<const char*>(&this->rowSize), sizeof(int));

  // Used to compute time required to compute the visibility table
  auto startTime =
    std::chrono::system_clock::now().time_since_epoch();

  ignition::math::Vector3d startPos, endPos;

  // These two variables are used to compute percent complete
  double div = std::pow(this->rowSize, 2);
  uint64_t count = 0;

  // Iterate over the possible y values.
  for (int y = this->range[0]; y <= this->range[1]; y += this->stepSize)
  {
    // Iterate over the possible x values.
    for (int x = this->range[0]; x <= this->range[1];
         x += this->stepSize, count++)
    {
      // Output percent complete
      printf("\r%04.2f %% ", (count/div)*100);

      // Get the  index of the (x, y) coordinate
      unsigned int index = this->Index(x, y);

      startPos.Set(x, y, heights[index]);

      // The inner loops checks visibility from startPos to endPos
      for (int y2 = y; y2 <= this->range[1] && y2 <= y+250;
           y2 += this->stepSize)
      {
        endPos.Y(y2);
        int rx = this->range[0];
        if (y2 == y)
          rx = x;

        for (int x2 = rx; x2 <= this->range[1] && x2 <= rx+250;
             x2 += this->stepSize)
        {

          // Get the index of the (x2, y2) coordinate
          unsigned int index2 = this->Index(x2, y2);
          double dist = sqrt((x2-x)*(x2-x) + (y2-y)*(y2-y));

          // Skip values that are too far away, of if x,y == x2,y2
          if (dist > 250 || index == index2)
            continue;

          endPos.X(x2);
          endPos.Z(heights[index2]);

          bool visible = this->LineOfSight(startPos, endPos);

          // Only store values that are not visible
          if (!visible)
          {
            uint64_t value = this->Pair(index, index2);
            out.write(reinterpret_cast<const char*>(&value),
                sizeof(uint64_t));
          }
        }
      }
    }
  }
  auto endTime = std::chrono::system_clock::now().time_since_epoch();

  auto duration = endTime - startTime;
  std::cout << "\nLookup table created in "
    << std::chrono::duration_cast<std::chrono::seconds>(duration).count()
    << " seconds\n";
  std::cout << "Visibility table at: " << outFilename << std::endl;

  out.close();

  // Only run once
  event::Events::DisconnectWorldUpdateBegin(this->updateConn);
}

//////////////////////////////////////////////////
bool VisibilityPlugin::LineOfSight(const ignition::math::Vector3d &_p1,
                                   const ignition::math::Vector3d &_p2)
{
  std::string firstEntity;
  double dist;

  this->ray->SetPoints(_p1, _p2);
  this->ray->GetIntersection(dist, firstEntity);

  return firstEntity.empty();
}

/////////////////////////////////////////////////
double VisibilityPlugin::HeightAt(const double _x, const double _y) const
{
  double dist;
  std::string ent;

  // Get the height of the terrain at the specified point
  this->ray->SetPoints(
      ignition::math::Vector3d(_x, _y, 1000),
      ignition::math::Vector3d(_x, _y, 0));
  this->ray->GetIntersection(dist, ent);

  // Add a little offset so that the ray test don't start/end in the
  // terrain.
  return 1000 - dist + 1;
}

/////////////////////////////////////////////////
uint64_t VisibilityPlugin::Pair(uint64_t _a, uint64_t _b)
{
  // Szudzik's function
  return _a >= _b ?  _a * _a + _a + _b : _a + _b * _b;
}

/////////////////////////////////////////////////
uint64_t VisibilityPlugin::Index(int _x, int _y)
{
  return ((_y + this->maxY)/this->stepSize) * this->rowSize +
    ((_x + this->maxY)/this->stepSize);
}
