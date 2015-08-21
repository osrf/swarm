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

#include <utility>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

#include "swarm/comms/CommsModel.hh"
#include "swarm/SwarmTypes.hh"

using namespace swarm;
using namespace comms;

//////////////////////////////////////////////////
CommsModel::CommsModel(SwarmMembershipPtr _swarm,
    gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
  : swarm(_swarm),
    world(_world)
{

}

//////////////////////////////////////////////////
void CommsModel::UpdateOutages(const gazebo::common::Time &_dt)
{
  for (auto const &robot : (*this->swarm))
  {
    auto address = robot.first;
    auto swarmMember = robot.second;

    // Check if I am currently on outage.
    if (swarmMember->onOutage &&
        swarmMember->onOutageUntil != gazebo::common::Time::Zero)
    {
      // Check if the outage should finish.
      if (this->world->GetSimTime() >= swarmMember->onOutageUntil)
      {
        swarmMember->onOutage = false;
        gzdbg << "Robot " << address << " is back from an outage." << std::endl;
      }
    }
    else
    {
      // Check if we should go into an outage.
      if (ignition::math::Rand::DblUniform(0.0, 1.0) <
          this->commsOutageProbability * _dt.Double())
      {
        swarmMember->onOutage = true;
        gzdbg << "Robot " << address << " has started an outage." << std::endl;

        // Decide the duration of the outage.
        if (this->commsOutageDurationMin < 0 ||
            this->commsOutageDurationMax < 0)
        {
          // Permanent outage.
          swarmMember->onOutageUntil = gazebo::common::Time::Zero;
        }
        else
        {
          // Temporal outage.
          swarmMember->onOutageUntil = this->world->GetSimTime() +
            ignition::math::Rand::DblUniform(
              this->commsOutageDurationMin,
              this->commsOutageDurationMax);
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void CommsModel::UpdateNeighbors()
{
  // Update the list of neighbors for each robot.
  for (auto const &robot : (*this->swarm))
    this->UpdateNeighborList(robot.first);
}

//////////////////////////////////////////////////
void CommsModel::UpdateNeighborList(const std::string &_address)
{
  GZ_ASSERT(this->swarm->find(_address) != this->swarm->end(),
            "_address not found in the swarm.");

  auto swarmMember = (*this->swarm)[_address];

  auto myPose = swarmMember->model->GetWorldPose();

  auto topic = "/swarm/" + swarmMember->address + "/neighbors";

  // Update the neighbor list for this robot.
  swarmMember->neighbors = {{_address, 1.0}};

  // If I am on outage, my only neighbor is myself.
  if (swarmMember->onOutage)
  {
    // Fill the message with only one neighbor (myself).
    //swarm::msgs::Neighbor_V msg;
    //msg.add_neighbors(_address);

    // Notify the node with its updated list of neighbors.
    //this->node.Publish(topic, msg);
    return;
  }

  for (auto const &member : (*this->swarm))
  {
    // Decide whether this node goes into our neighbor list

    // Where is the other node?
    auto other = member.second;
    auto otherPose = other->model->GetWorldPose();

    // How far away is it from me?
    auto dist = (myPose.pos - otherPose.pos).GetLength();
    auto neighborDist = dist;
    auto commsDist = dist;
    int numWalls = 0;
    int numTrees = 0;

    // Check if the other teammate is currenly on outage.
    if (other->onOutage)
      continue;

    // Apply the neighbor part of the comms model
    auto neighbor = true;
    if (neighbor &&
        (this->neighborDistanceMin >= 0.0) &&
        (this->neighborDistanceMin > neighborDist))
      neighbor = false;
    if (neighbor &&
        (this->neighborDistanceMax >= 0.0) &&
        (this->neighborDistanceMax < neighborDist))
      neighbor = false;
    if (neighbor && this->neighborDistancePenaltyWall > 0.0)
    {
      // We're within range.  Check for obstacles (don't want to waste time on
      // that if we're not within range).
      numWalls = this->NumWallsBetweenPoses(myPose, otherPose);
      if ((numWalls > 0) &&
          (this->neighborDistancePenaltyWall < 0.0))
        neighbor = false;
      else
        neighborDist -= numWalls * this->neighborDistancePenaltyWall;
      if (neighbor &&
          (this->neighborDistanceMin >= 0.0) &&
          (this->neighborDistanceMin > neighborDist))
        neighbor = false;
      if (neighbor &&
          (this->neighborDistanceMax >= 0.0) &&
          (this->neighborDistanceMax < neighborDist))
        neighbor = false;
    }
    if (neighbor && this->neighborDistancePenaltyTree > 0.0)
    {
      // We're within range.  Check for obstacles (don't want to waste time on
      // that if we're not within range).
      numTrees = this->NumTreesBetweenPoses(myPose, otherPose);
      if ((numTrees > 0) &&
          (this->neighborDistancePenaltyTree < 0.0))
        neighbor = false;
      else
        neighborDist -= numTrees * this->neighborDistancePenaltyTree;
      if (neighbor &&
          (this->neighborDistanceMin >= 0.0) &&
          (this->neighborDistanceMin > neighborDist))
        neighbor = false;
      if (neighbor &&
          (this->neighborDistanceMax >= 0.0) &&
          (this->neighborDistanceMax < neighborDist))
        neighbor = false;
    }

    if (neighbor)
    {
      // Now apply the comms model to compute a probability of a packet from
      // this neighbor arriving successfully.
      auto commsProb = 1.0;

      if ((commsProb > 0.0) &&
          (this->commsDistanceMin >= 0.0) &&
          (this->commsDistanceMin > commsDist))
        commsProb = 0.0;
      if ((commsProb > 0.0) &&
          (this->commsDistanceMax >= 0.0) &&
          (this->commsDistanceMax < commsDist))
        commsProb = 0.0;
      if ((commsProb > 0.0) && this->commsDistancePenaltyWall > 0.0)
      {
        // We're within range.  Check for obstacles (don't want to waste time on
        // that if we're not within range).
        if ((numWalls > 0) &&
            (this->commsDistancePenaltyWall < 0.0))
          commsProb = 0.0;
        else
          commsDist -= numWalls * this->commsDistancePenaltyWall;
        if ((commsProb > 0.0) &&
            (this->commsDistanceMin >= 0.0) &&
            (this->commsDistanceMin > commsDist))
          commsProb = 0.0;
        if ((commsProb > 0.0) &&
            (this->commsDistanceMax >= 0.0) &&
            (this->commsDistanceMax < commsDist))
          commsProb = 0.0;
      }
      if ((commsProb > 0.0) && this->commsDistancePenaltyTree > 0.0)
      {
        // We're within range.  Check for obstacles (don't want to waste time on
        // that if we're not within range).
        if ((numTrees > 0) &&
            (this->commsDistancePenaltyTree < 0.0))
          commsProb = 0.0;
        else
          commsDist -= numTrees * this->commsDistancePenaltyTree;
        if ((commsProb > 0.0) &&
            (this->commsDistanceMin >= 0.0) &&
            (this->commsDistanceMin > commsDist))
          commsProb = 0.0;
        if ((commsProb > 0.0) &&
            (this->commsDistanceMax >= 0.0) &&
            (this->commsDistanceMax < commsDist))
          commsProb = 0.0;
      }

      if (commsProb > 0.0)
      {
        // We made it through the outage, distance, and obstacle filters.
        // Compute a drop probability between these two nodes for this
        // time step.
        commsProb = 1.0 - ignition::math::Rand::DblUniform(
          this->commsDropProbabilityMin,
          this->commsDropProbabilityMax);
      }

      // Stuff the resulting information into our local representation of this
      // node; we'll refer back to it later when processing messages sent
      // between nodes.
      // Also a message containing the neighbor list (not the probabilities)
      // will be sent out below, to allow robot controllers to query the
      // neighbor list.
      std::pair<std::string, double> neighborAndProb;
      neighborAndProb.first = member.first;
      neighborAndProb.second = commsProb;
      swarmMember->neighbors.push_back(neighborAndProb);
    }
  }

  // Fill the message with the new neighbor list.
  //swarm::msgs::Neighbor_V msg;
  //for (auto const &neighbor : swarmMember->neighbors)
  //  msg.add_neighbors(neighbor.first);

  // Notify the node with its updated list of neighbors.
  //this->node.Publish(topic, msg);
}

//////////////////////////////////////////////////
unsigned int CommsModel::NumWallsBetweenPoses(const gazebo::math::Pose& p1,
                                              const gazebo::math::Pose& p2)
{
  // TODO: raytrace to answer this question
  return 0;
}

//////////////////////////////////////////////////
unsigned int CommsModel::NumTreesBetweenPoses(const gazebo::math::Pose& p1,
                                              const gazebo::math::Pose& p2)
{
  // TODO: raytrace to answer this question
  return 0;
}
