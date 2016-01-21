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
#ifndef _SWARM_VISIBILITY_PLUGIN_HH_
#define _SWARM_VISIBILITY_PLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/util/system.hh"

#include "swarm/LogParser.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE VisibilityPlugin : public SystemPlugin
  {
    /// \brief The types of terrain.
    public: enum TerrainType
            {
              /// \brief Open terrain
              PLAIN     = 0,

              /// \brief Terrain with forest
              FOREST    = 1,

              /// \brief Terrain with a building
              BUILDING  = 2
            };

    /// \brief Destructor
    public: virtual ~VisibilityPlugin();

    /// \brief Load the plugin.
    /// \param[in] _argc Number of command line arguments.
    /// \param[in] _argv Array of command line arguments.
    public: void Load(int _argc, char **_argv);

    /// \brief Initialize the plugin.
    private: void Init();

    private: void Update();

    private: void OnWorldCreated();

    /// \brief Query the map to get the height and terrain type
    /// at a specific latitude and longitude.
    ///
    /// \param[in] _lat Latitude of the query (degrees).
    /// \param[in] _lon Longitude of the query (degrees).
    /// \param[out] _elev Elevation at the query point (meters).
    /// \param[out] _type Type of terrain at the query point.
    /// \return True if the latitude and longitude specify a valid point.
    /// False otherwise.
    protected: bool MapQuery(const double _lat, const double _lon,
                             double &_height, TerrainType &_type);


    /// \brief Get terrain information at the specified location.
    /// \param[in] _pos Reference position.
    /// \param[out] _terrainPos The 3d point on the terrain.
    /// \param[out] _norm Normal to the terrain.
    private: void TerrainLookup(const ignition::math::Vector3d &_pos,
                                ignition::math::Vector3d &_terrainPos,
                                ignition::math::Vector3d &_norm) const;

    /// \brief Helper function to get a terrain type at a position in
    /// Gazebo's world coordinate frame.
    /// \param[in] _pos Position to query.
    /// \return Type of terrain at the location.
    private: TerrainType TerrainAtPos(const ignition::math::Vector3d &_pos);

    private: int Lookup(const ignition::math::Vector3d &_start,
                 const ignition::math::Vector3d &_end);

    private: double HeightAt(const double _x, const double _y) const;

    private: bool LineOfSight(const ignition::math::Vector3d &_p1,
                             const ignition::math::Vector3d &_p2);

    private: uint64_t Key(int _a, int _b);

    private: int Index(int _x, int _y, int _maxY, int _stepSize, int _rowSize);

    /// \brief Pointer to the world.
    private: physics::WorldPtr world;

    /// \brief The world created connection.
    private: event::ConnectionPtr worldCreatedConn;

    /// \brief The update connection.
    private: event::ConnectionPtr updateConn;

    /// \brief Pointer to the terrain
    private: gazebo::physics::HeightmapShapePtr terrain;

    /// \brief This is the scaling from world coordinates to heightmap
    /// coordinates.
    private: ignition::math::Vector2d terrainScaling;

    /// \brief Size of the terrain
    private: ignition::math::Vector3d terrainSize;

    /// \brief Min/max lat/long of search area.
    private: double searchMinLatitude, searchMaxLatitude,
                    searchMinLongitude, searchMaxLongitude;

    // \brief Ray used to test for line of sight between vehicles.
    private: gazebo::physics::RayShapePtr ray;
    private: gazebo::physics::MultiRayShapePtr multiRay;
  };
}
#endif
