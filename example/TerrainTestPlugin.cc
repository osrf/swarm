/*
 *
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
#include "TerrainTestPlugin.hh"

using namespace swarm;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(TerrainTestPlugin)

//////////////////////////////////////////////////
TerrainTestPlugin::TerrainTestPlugin()
: RobotPlugin()
{
}

//////////////////////////////////////////////////
void TerrainTestPlugin::Load(sdf::ElementPtr _sdf)
{
  // divide the world into 1000x1000 chucks
  double minLat, maxLat, minLon, maxLon;

  // get lat/lon bounds
  this->SearchArea(minLat, maxLat, minLon, maxLon);
  double stepLon = (maxLon - minLon) / 500.0;
  double stepLat = (maxLat - minLat) / 500.0;

  double elevation;
  swarm::RobotPlugin::TerrainType terrainType;

  ofstream elevationFile("elevation.csv");
  ofstream terrainFile("terrain.csv");

  elevationFile << ", ";
  terrainFile << ", ";

  for (double lon = minLon; lon < maxLon; lon += stepLon)
  {
    elevationFile << to_string(lon) << ((lon + stepLon < maxLon) ? ", " : "\n");
    terrainFile   << to_string(lon) << ((lon + stepLon < maxLon) ? ", " : "\n");
  }

  for (double lat = minLat; lat < maxLat; lat += stepLat)
  {
    elevationFile << to_string(lat) << ", ";
    terrainFile << to_string(lat) << ", ";

    for (double lon = minLon; lon < maxLon; lon += stepLon)
    {
      MapQuery(lat, lon, elevation, terrainType);
      elevationFile << elevation << ((lon + stepLon < maxLon) ? ", " : "\n");
      terrainFile   << terrainType << ((lon + stepLon < maxLon) ? ", " : "\n");
    }
  }
  std::cout << "done!\n";

  elevationFile.close();
  terrainFile.close();
}
