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

/// \file Common.hh
/// \brief Functions and attributes required by multiple plugins.

#ifndef __SWARM_COMMON__
#define __SWARM_COMMON__

#include <sdf/sdf.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include "swarm/SwarmTypes.hh"

namespace swarm
{
  class Common
  {
    /// \brief Constructor
    public: Common();

    /// \brief Destructor
    public: virtual ~Common() = default;

    /// \brief Get the minimum latitude of the search area.
    /// \return Search area minimum latitude
    public: double SearchMinLatitude() const;

    /// \brief Get the maximum latitude of the search area.
    /// \return Search area maximum latitude
    public: double SearchMaxLatitude() const;

    /// \brief Get the minimum longitude of the search area.
    /// \return Search area minimum longitude
    public: double SearchMinLongitude() const;

    /// \brief Get the maximum longitude of the search area.
    /// \return Search area maximum longitude
    public: double SearchMaxLongitude() const;

    /// \brief set the minimum latitude of the search area.
    /// \param[in] _value Minimum latitude
    public: void SetSearchMinLatitude(const double _value);

    /// \brief set the maximum latitude of the search area.
    /// \param[in] _value Maximum latitude
    public: void SetSearchMaxLatitude(const double _value);

    /// \brief Set the minimum longitude of the search area.
    /// \param[in] _value Minimum longitude
    public: void SetSearchMinLongitude(const double _value);

    /// \brief Set the maximum longitude of the search area.
    /// \param[in] _value Maximum longitude
    public: void SetSearchMaxLongitude(const double _value);

    /// \brief Get the search area, in GPS coordinates.
    ///
    /// \param[out] _minLatitude Minimum latitude will be written here.
    /// \param[out] _maxLatitude Maximum latitude will be written here.
    /// \param[out] _minLongitude Minimum longitude will be written here.
    /// \param[out] _maxLongitude Maximum longitude will be written here.
    public: void SearchArea(double &_minLatitude,
                            double &_maxLatitude,
                            double &_minLongitude,
                            double &_maxLongitude);

    /// \brief Load the search area from SDF
    /// \param[in] _searchAreaElem Pointer to the <search_area> element.
    /// \return True if the search area was found.
    public: bool LoadSearchArea(sdf::ElementPtr _searchAreaSDF);

    /// \brief Load spherical coordinates from SDF
    /// \param[in] _sphereicalCoordsSDF Pointer to the
    /// <spherical_coordinates> sdf element
    /// \return True if the spherical coordinates element was found.
    public: bool LoadSphericalCoordinates(sdf::ElementPtr _sphericalCoordsSDF);

    /// \brief Get terrain information at the specified location.
    /// \param[in] _pos Reference position.
    /// \param[out] _terrainPos The 3d point on the terrain.
    /// \param[out] _norm Normal to the terrain.
    public: void TerrainLookup(const ignition::math::Vector3d &_pos,
                               ignition::math::Vector3d &_terrainPos,
                               ignition::math::Vector3d &_norm) const;

    /// \brief Query the map to get the height and terrain type
    /// at a specific latitude and longitude.
    ///
    /// \param[in] _lat Latitude of the query (degrees).
    /// \param[in] _lon Longitude of the query (degrees).
    /// \param[out] _elev Elevation at the query point (meters).
    /// \param[out] _type Type of terrain at the query point.
    /// \return True if the latitude and longitude specify a valid point.
    /// False otherwise.
    public: bool MapQuery(const double _lat, const double _lon,
                          double &_height, TerrainType &_type);

    /// \brief Helper function to get a terrain type at a position in
    /// Gazebo's world coordinate frame.
    /// \param[in] _pos Position to query.
    /// \return Type of terrain at the location.
    public: TerrainType TerrainAtPos(const ignition::math::Vector3d &_pos);

    /// \brief Get a pointer to the terrain.
    /// \return Pointer to the terrain.
    public: gazebo::physics::HeightmapShapePtr Terrain() const;

    /// \brief Get the size of the terrain.
    /// \return Terrain size.
    public: ignition::math::Vector3d TerrainSize() const;

    /// \brief Set the world pointer.
    /// \param[in] _world Pointer to the world.
    public: void SetWorld(gazebo::physics::WorldPtr _world);

    /// \brief Set the terrain.
    /// \param[in] _terrain Pointer to the heightmap.
    public: void SetTerrain(gazebo::physics::HeightmapShapePtr _terrain);

    /// \brief Min/max lat/long of search area.
    private: double searchMinLatitude, searchMaxLatitude,
                    searchMinLongitude, searchMaxLongitude;

    /// \brief Pointer to the world.
    private: gazebo::physics::WorldPtr world;

    /// \brief This is the scaling from world coordinates to heightmap
    /// coordinates.
    private: ignition::math::Vector2d terrainScaling;

    /// \brief Pointer to the terrain
    private: gazebo::physics::HeightmapShapePtr terrain;

    /// \brief Size of the terrain
    private: ignition::math::Vector3d terrainSize;
  };
}
#endif
