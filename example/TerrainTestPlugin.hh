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

/// \file TerrainTestPlugin.hh
/// \brief A plugin that outputs an elevation.csv and a terrain.csv file.
/// The elevation.csv file contains elevation information within the search
/// area. The terrain.csv files contains terrain type informaiton within the
/// search area. These csv files can be used in conjunction with terrain.py
/// to produce png file respresentations of the raw data.

#ifndef __SWARM_TERRAIN_DEBUG_PLUGIN_HH__
#define __SWARM_TERRAIN_DEBUG_PLUGIN_HH__

#include "swarm/RobotPlugin.hh"

namespace swarm
{
  /// \brief A plugin useful for testing terrain elevation and terrain type.
  /// Refere to the following tutorial for more information:
  class TerrainTestPlugin : public swarm::RobotPlugin
  {
    /// \brief constructor
    public: TerrainTestPlugin();

    /// \brief destructor
    public: virtual ~TerrainTestPlugin() = default;

    /// \brief Produces the elevation.csv and terrain.csv files.
    /// \param[in] _sdf Pointer the sdf information.
    public: virtual void Load(sdf::ElementPtr _sdf);
  };
}
#endif
