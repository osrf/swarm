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

/// \file LostPersonControllerPlugin.hh
/// \brief An example of a Gazebo plugin for controlling a lost person.

#ifndef __SWARM_LOST_PERSON_CONTROLLER_PLUGIN_HH__
#define __SWARM_LOST_PERSON_CONTROLLER_PLUGIN_HH__

#include <gazebo/common/Time.hh>
#include <swarm/LostPersonPlugin.hh>
#include <swarm/RobotPlugin.hh>
#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>

namespace swarm
{
  /// \brief Example implementation of a lost person model
  class LostPersonControllerPlugin : public swarm::LostPersonPlugin
  {
    /// \brief Class constructor.
    public: LostPersonControllerPlugin();

    /// \brief Class destructor.
    public: virtual ~LostPersonControllerPlugin() = default;

    public: void MoveToPosition(double currentX, double currentY, double targetX, double targetY);

    public: double GetDistance(double currentX, double currentY, double targetX, double targetY);

    // Documentation inherited.
    public: virtual void Load(sdf::ElementPtr _sdf);

    // Documentation inherited.
    private: virtual void Update(const gazebo::common::UpdateInfo &_info);

    //Private Variables
    private: void findNeighbor(double bearing, double distance, double* neighborLat, double* neighborLong);
        //Some constants
    // Speed and bearing (in degrees) of the lost person
    private:
        double speed = 0.0;
        double bearing = 0.0;
         //Some constants

        int degreeThreshold = 2;
        //double AltitudeThreshold = tan(degreeThreshold)*distBetweenPoints; //10 degree incline? (tan(10degrees)*distBetweenPoints)
        const double velocityMagnitude = 1.4;   //1.4 meters per second
        //These means are trying to map to Lanny Lin's means for Arizona25
        double slopeMeans[3] = {.4, .4, .2};   //(Level, Down, Up)
        double topographyMeans[3][3] = {{.3, .2, .5}, {.3, .2, .5}, {.3, .1, .6}}; //row 0 is PLAIN, 1 for FOREST, 2 for BUILDING
        double variance = .0025;    //stdDev of .05

        //variables for storing information about the neighbors' altitudes, vegetations, and topographies
        TerrainType curType;   //an enum: 0 is PLAIN, 1 for FOREST, 2 for BUILDING
        double latitude, longitude, altitude; //Latitude and Longitude kept up by the controller.
        TerrainType neighborTypes[6];
        double neighborLats[7];
        double neighborLongs[7];
        double neighborHeights[6];
        double transitionalProbabilities[7]; //0 is 12 o'clock, 6 is transition to      //variables for finding neighbors
        double earthRad = 6378.1; //km


    /// \brief Time between recomputing a new velocity vector.
    private: gazebo::common::Time updatePeriod = 1.0;

    /// \brief Time the of the last update.
    private: gazebo::common::Time prevUpdate = 0.0;

    public: ignition::math::Vector3d velocity;
    public: double velocityFactor = 1.0;

    //Private Variables
    private: void FindNeighbor(double bearing, double distance, double* neighborLat, double* neighborLong);
    private: int DetermineNewGoal(double transitionalProbabilities[]);
  };
}
#endif
