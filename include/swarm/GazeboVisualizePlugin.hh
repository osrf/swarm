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
#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/util/system.hh"

#include "swarm/LogParser.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE GazeboVisualizePlugin : public SystemPlugin
  {
    /// \brief Destructor
    public: virtual ~GazeboVisualizePlugin();

    /// \brief Load the plugin.
    /// \param[in] _argc Number of command line arguments.
    /// \param[in] _argv Array of command line arguments.
    public: void Load(int _argc, char **_argv);

    /// \brief Initialize the plugin.
    private: void Init();

    private: void OnWorldCreated();

    /// \brief Update the plugin.
    private: void Update();

    /// \brief Create the lost person marker
    private: void CreateLostPersonMarker();

    /// \brief Helper function to visualize messages
    private: void VisualizeMessages(swarm::msgs::LogEntry &_logEntry);

    private: void VisualizeNeighbors(swarm::msgs::LogEntry &_logEntry);

    /// \brief Pointer to the world.
    private: physics::WorldPtr world;

    /// \brief The world created connection.
    private: event::ConnectionPtr worldCreatedConn;

    /// \brief The update connection.
    private: event::ConnectionPtr updateConn;

    /// \brief Node for communication.
    private: transport::NodePtr node;

    /// \brief Publisher used to send visualization markers.
    private: transport::PublisherPtr markerPub;

    private: common::Time lastUpdate;

    private: swarm::LogParser parser;
    //private: int circleCount;
    //private: int messageCount;
  };
}
