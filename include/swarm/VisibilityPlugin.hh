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
#include <array>

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief This plugin generates a visibility lookup table with the
  /// following contents:
  ///
  /// int max_y_value
  /// int step_size
  /// int row_size
  /// uint64_t keys
  ///
  /// Data is stored in binary, and keys is a list of unique uint64_t values
  /// that represent two coordinates that *do not* have visiblity. It is
  /// assumed that any two coordiantes separated by more than 250m are not
  /// visible.
  ///
  /// Keys are generated using a combination of an Index and Pair function.
  ///
  /// We assume a terrain, without any other objects, is used to generate
  /// the visibility lookup table.
  ///
  /// Example usage:
  ///   gzserver -s libVisibilityPlugin.so --iters 1 worlds/swarm_vis.world
  ///
  /// The visibility table will be located at /tmp/visibility.txt
  class GAZEBO_VISIBLE VisibilityPlugin : public SystemPlugin
  {
    /// \brief Destructor
    public: virtual ~VisibilityPlugin();

    /// \brief Load the plugin.
    /// \param[in] _argc Number of command line arguments.
    /// \param[in] _argv Array of command line arguments.
    public: void Load(int _argc, char **_argv);

    /// \brief Initialize the plugin.
    private: void Init();

    /// \brief World update callback
    private: void Update();

    /// \brief World created callback
    private: void OnWorldCreated();

    /// \brief Get the height at a coordinate
    /// \param[in] _x X world coordinate
    /// \param[in] _y Y world coordinate
    /// \return Height at the coordinate
    private: double HeightAt(const double _x, const double _y) const;

    /// \brief Get whether two points have line of sight.
    /// \param[in] _p1 First coordinate
    /// \param[in] _p1 Second coordinate
    /// \return True if the two points are visible
    private: bool LineOfSight(const ignition::math::Vector3d &_p1,
                              const ignition::math::Vector3d &_p2);

    /// \brief A pairing function that maps two values to a unique third
    /// value.
    /// \param[in] _a First value
    /// \param[in] _b Second value
    /// \return A unique key value
    private: uint64_t Pair(const uint64_t _a, uint64_t _b);

    /// \brief Generate an index from a coordinate.
    /// \param[in] _x X coordinate
    /// \param[in] _y Y coordinate
    private: uint64_t Index(int _x, int _y);

    /// \brief XY range, or size of the swarm search area
    private: std::array<int, 2> range;

    /// \brief Maximum Y value of the range
    private: int maxY;

    /// \brief The granularity of the visibility table
    private: int stepSize;

    /// \brief Number of values in each row of the visibility table.
    private: int rowSize;

    /// \brief The world created connection.
    private: event::ConnectionPtr worldCreatedConn;

    /// \brief The update connection.
    private: event::ConnectionPtr updateConn;

    // \brief Ray used to test for line of sight.
    private: gazebo::physics::RayShapePtr ray;
  };
}
#endif
