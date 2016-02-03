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

#include <ignition/math/Helpers.hh>
#include "swarm/LostPersonControllerPlugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(LostPersonControllerPlugin)

//////////////////////////////////////////////////
LostPersonControllerPlugin::LostPersonControllerPlugin()
  : LostPersonPlugin(),
    latitude(0.0),
    longitude(0.0),
    altitude(0.0)
{
}

//////////////////////////////////////////////////
void LostPersonControllerPlugin::Load(sdf::ElementPtr _sdf)
{
  // Example of reading a value from the SDF file.
  if (_sdf->HasElement("speed"))
    this->speed = _sdf->Get<double>("speed");

  // Set the initial velocity, if present
  if (_sdf->HasElement("initial_velocity"))
    this->velocity = _sdf->Get<ignition::math::Vector3d>("initial_velocity");

  // Set the velocity factor
  if (_sdf->HasElement("velocity_factor"))
    this->velocityFactor = _sdf->Get<double>("velocity_factor");

  // Set the update period
  if (_sdf->HasElement("update_period"))
    this->updatePeriod = _sdf->Get<double>("update_period");
}

//////////////////////////////////////////////////
void LostPersonControllerPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  //Update this as often as is specified.
  if (_info.simTime - this->prevUpdate > this->updatePeriod)
  {
    /*******************************/
    //Create Transition Matrix:
    /*******************************/
    //this->Pose(curLat, curLong, curHeight);
    //printf("Latitude: %f\nLongitude: %f\nAltitude: %f\n\n", curLat, curLong, curHeight);

     //Not entirely sure how this fits currently
    // Uncomment this block to get your current position on the map.
    this->Pose(latitude, longitude, altitude);

    if (latitude >= this->common.SearchMaxLatitude() ||
        latitude <= this->common.SearchMinLatitude())
    {
      this->velocity.Y() *= -1;
    }

    if (longitude >= this->common.SearchMaxLongitude() ||
        longitude <= this->common.SearchMinLongitude())
    {
      this->velocity.X() *= -1;
    }

    if(MapQuery(latitude, longitude, altitude, curType) == false)
    {
     // printf("\n\n We're NOT inside the search area.\n\n");
    }

    //Figure out the distance we have travelled here:
    //double distance = .................
    double mybearing = 0;
    double distance = (_info.simTime.Double() - this->prevUpdate.Double()) * velocityMagnitude; //I think that this length will give the magnitude...
    double AltitudeThreshold = tan(degreeThreshold * ((2 * M_PI) / 360))*distance; //This is in meters, assuming that the height we query for is in meters.

    //Now convert distance to kilometers.
    distance = distance / 1000; //Currently it's about 5. I think this is 5m...

    //double distance = .25; //15 km...
    for(int i = 0; i < 6; i++)
      FindNeighbor(mybearing + i * 60, distance, &neighborLats[i], &neighborLongs[i]);
    neighborLats[6] = latitude;        //Adding ourselves to lat and long dictionaries for goal-finding
    neighborLongs[6] = longitude;

    //Query altitude and topography for each position. We will assume that the point we query is a decent approximation for the average for the whole area.
    for(int i = 0; i < 6; i++)
    {
      if(!MapQuery(neighborLats[i], neighborLongs[i], neighborHeights[i], neighborTypes[i]))
     {
        //printf("\nNeighbor queried outside of search area.\n");
        neighborLats[i] = neighborLongs[i] = 0;//throw away this data. When we calculate our probabilities we will ignore these points.
      }
    }

    //Calculate probabilities using the means
    double sumProbs = 0;
    //compute for neighbors:
    for(int i = 0; i < 6; i++)
    {
      if(ignition::math::equal(neighborLats[i], 0.0) &&
         ignition::math::equal(neighborLongs[i], 0.0))
      {
        transitionalProbabilities[i] = 0;
        continue;
      }
      double altProb = 0;
      double diff = altitude - neighborHeights[i];
      double realDiff = sqrt(diff * diff);
      if(realDiff < AltitudeThreshold)
      {
        altProb = slopeMeans[0];
      }
      else if (diff < 0)
      {
        altProb = slopeMeans[2];
      }
      else
      {
        altProb = slopeMeans[1];
      } //descending

      double topographyProb = topographyMeans[curType][neighborTypes[i]];

      double curProb = topographyProb * altProb;
      transitionalProbabilities[i] = curProb;
      sumProbs += curProb;
    }

    //Compute for self:
    double curProb = slopeMeans[0] * topographyMeans[curType][curType];
    sumProbs += curProb;
    transitionalProbabilities[6] = curProb;

    for(int i = 0; i < 7; i++)
    {
      transitionalProbabilities[i] = transitionalProbabilities[i] / sumProbs;
    }

    if (!gazebo::common::Console::GetQuiet())
    {
      //Print the transitional matrix for debugging purposes and the occasional sanity check.
      // printf("\n\n");
      // printf("            ______                 \n");
      // printf("           /      \\               \n");
      // printf("    ______/  %.2f  \\______        \n", transitionalProbabilities[0]);
      // printf("   /      \\        /      \\      \n");
      // printf("  /  %.2f  \\______/  %.2f  \\     \n", transitionalProbabilities[5], transitionalProbabilities[1]);
      // printf("  \\        /      \\        /     \n");
      // printf("   \\______/  %.2f  \\______/      \n", transitionalProbabilities[6]);
      // printf("   /      \\        /      \\      \n");
      // printf("  /  %.2f  \\______/  %.2f  \\     \n", transitionalProbabilities[4], transitionalProbabilities[2]);
      // printf("  \\        /      \\        /     \n");
      // printf("   \\______/  %.2f  \\______/      \n", transitionalProbabilities[3]);
      // printf("          \\        /              \n");
      // printf("           \\______/               \n");
      // printf("\n\n");
    }

    /*******************************/
    // End Transition Matrix Creation
    /*******************************/
    int goalIndex = DetermineNewGoal(transitionalProbabilities);
    //printf("\nCurrLat: %f CurLong: %f      GoalLat : %f  GoalLong: %f\n", latitude, longitude, neighborLats[goalIndex], neighborLongs[goalIndex]);
    MoveToPosition(latitude, longitude, neighborLats[goalIndex], neighborLongs[goalIndex]);
    this->prevUpdate = _info.simTime;
    //this->velocity.Normalize();
    //this->velocity *= this->velocityFactor;

    this->model->SetLinearVel(this->velocity);
    //printf("\n Velocity:    X: %f Y: %f Z: %f", this->velocity.X() , this->velocity.Y(), this->velocity.Z());
    this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
  }
  //this->model->SetAngularVel(ignition::math::Vector3d(3.14, 0, 0));
}

void LostPersonControllerPlugin::MoveToPosition(double currentX, double currentY, double targetX, double targetY)
{
  double diffX = targetX - currentX;
  double diffY = targetY - currentY;
  double distance = GetDistance(currentX, currentY, targetX, targetY);

  if (ignition::math::equal(distance, 0.0))
  {
    //printf("\nAttempting to Stay Put.\n");
    this->velocity *= 0;
  }
  else if (ignition::math::equal(targetX, 0.0) &&
           ignition::math::equal(targetY, 0.0))
  {
    // printf("\nGoal is outside of search area.\n");
    this->velocity *= 0;
  }
  else
  {
    this->velocity.X() = (diffX/distance);
    this->velocity.Y() = (diffY/distance); //this->model->SetLinearVel(ignition::math::Vector3d(diffX/distance, diffY/distance, 0));
  }
}

double LostPersonControllerPlugin::GetDistance(double currentX, double currentY, double targetX, double targetY)
{
    return sqrt((targetX - currentX)*(targetX - currentX) + (targetY - currentY)*(targetY - currentY));
}

//Distance in km, bearing in degrees. Based on current location.
void LostPersonControllerPlugin::FindNeighbor(double _bearing, double distance, double* neighborLat, double* neighborLong)
{
  //To radians:
  double lat1;
  double lon1;

  _bearing = _bearing * (2 * M_PI) / 360;
  lat1 = latitude * (2 * M_PI) / 360; //Current lat point converted to radians
  lon1 = longitude * (2 * M_PI) / 360; //Current long point converted to radians

  double lat2 = asin(sin(lat1)*cos(distance/earthRad) + cos(lat1)*sin(distance/earthRad)*cos(_bearing));

  double lon2 = lon1 + atan2(sin(_bearing)*sin(distance/earthRad)*cos(lat1), cos(distance/earthRad)-sin(lat1)*sin(lat2));

  //To degrees:
  lat2 = lat2 * 360 / (2 * M_PI);
  lon2 = lon2 * 360 / (2 * M_PI);

  *neighborLat = lat2;
  *neighborLong = lon2;
}

int LostPersonControllerPlugin::DetermineNewGoal(double _transitionalProbabilities[])
{

  double randomNumber = ((double) rand() / (RAND_MAX));
  double counter = 0;

  for (int i = 0; i < 7; i++)
  {
    counter += _transitionalProbabilities[i];
    if (randomNumber <= counter)
    {
      return i;
    }
  }

  // ToDo: Line inserted by caguero. Please review.
  return 0;
}
