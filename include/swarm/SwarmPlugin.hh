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

/// \file SwarmPlugin.hh
/// \brief Structures and functions for the Swarm API.

#ifndef _SWARM_API_SWARM_HH_
#define _SWARM_API_SWARM_HH_

namespace swarm
{
  /// \brief A plugin
  class SwarmPlugin
  {
    /// \brief Constructor
    public: SwarmPlugin();

    /// \brief Destructor
    public: virtual ~SwarmPlugin();
  };
}

#endif
