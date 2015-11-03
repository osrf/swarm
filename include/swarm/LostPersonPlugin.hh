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

/// \file LostPersonPlugin.hh
/// \brief Main Swarm API for agent development.

#ifndef __SWARM_LOST_PERSON_PLUGIN_HH__
#define __SWARM_LOST_PERSON_PLUGIN_HH__

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

namespace swarm
{
  /// \brief A model plugin that is the base class for the lost person plugin.
  ///
  /// This plugin exposes the following functionality to the derived plugins:
  ///
  ///     - Load()    This method will allow the agent to read SDF parameters
  ///                 from the model.
  ///
  ///     - Update()  This method is called every iteration, and acts as
  ///                 the main loop.
  class IGNITION_VISIBLE LostPersonPlugin : public gazebo::ModelPlugin
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

    /// \brief Class constructor.
    public: LostPersonPlugin();

    /// \brief Class destructor.
    public: virtual ~LostPersonPlugin();

    /// \brief This method is called after the world has been loaded and gives
    /// child plugins access to the SDF model file.
    ///
    /// \param[in] _sdf Pointer to the SDF element of the model.
    protected: virtual void Load(sdf::ElementPtr _sdf);

    /// \brief Handle reset
    protected: virtual void Reset() {}

    /// \brief Update the plugin. This function is called once every iteration.
    ///
    /// \param[in] _info Update information provided by the server.
    protected: virtual void Update(const gazebo::common::UpdateInfo &_info);

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

    /// \brief Adjust the pose of the vehicle to stay within the terrain
    /// boundaries.
    private: void AdjustPose();

    /// \brief This method is called after the world has been loaded and gives
    /// child plugins access to the SDF model file.
    ///
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the SDF element of the model.
    private: virtual void Load(gazebo::physics::ModelPtr _model,
                               sdf::ElementPtr _sdf);

    /// \brief Helper function to get a terrain type at a position in
    /// Gazebo's world coordinate frame.
    /// \param[in] _pos Position to query.
    /// \return Type of terrain at the location.
    private: TerrainType TerrainAtPos(const ignition::math::Vector3d &_pos);

    /// \brief Update the plugin.
    ///
    /// \param[in] _info Update information provided by the server.
    private: virtual void Loop(const gazebo::common::UpdateInfo &_info);

    /// \brief Pointer to the model;
    protected: gazebo::physics::ModelPtr model;

    /// \brief Pointer to the update event connection.
    private: gazebo::event::ConnectionPtr updateConnection;

    /// \brief Pointer to the terrain
    private: gazebo::physics::HeightmapShapePtr terrain;

    /// \brief This is the scaling from world coordinates to heightmap
    /// coordinates.
    private: ignition::math::Vector2d terrainScaling;

    /// \brief Size of the terrain
    private: ignition::math::Vector3d terrainSize;

    /// \brief Half the height of the model.
    private: double modelHeight2 = 0;

    /// \brief Pointer to the world.
    private: gazebo::physics::WorldPtr world;

    /// \brief Min/max lat/long of search area.
    private: double searchMinLatitude, searchMaxLatitude,
                    searchMinLongitude, searchMaxLongitude;
  };
}
#endif
