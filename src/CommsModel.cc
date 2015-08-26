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
#include <utility>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

#include "swarm/CommsModel.hh"
#include "swarm/SwarmTypes.hh"

using namespace swarm;

//////////////////////////////////////////////////
CommsModel::CommsModel(SwarmMembershipPtr _swarm,
    gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
  : swarm(_swarm),
    world(_world)
{
  GZ_ASSERT(_world, "CommsModel() error: _world pointer is NULL");
  GZ_ASSERT(_sdf, "CommsModel() error: _sdf pointer is NULL");

  this->LoadParameters(_sdf);
}

//////////////////////////////////////////////////
void CommsModel::UpdateOutages()
{
  gazebo::common::Time curTime = this->world->GetSimTime();

  // In case we reset simulation.
  if (curTime <= this->lastUpdateTime)
  {
    this->lastUpdateTime = curTime;
    return;
  }

  // Get elapsed time since the last update.
  auto dt = curTime - this->lastUpdateTime;

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
      // Notice that "commsOutageProbability" specifies the probability of going
      // into a comms outage at each second. This probability is adjusted using
      // the elapsed time since the last call to "UpdateOutages()".
      if (ignition::math::Rand::DblUniform(0.0, 1.0) <
          this->commsOutageProbability * dt.Double())
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
          swarmMember->onOutageUntil = curTime +
            ignition::math::Rand::DblUniform(
              this->commsOutageDurationMin,
              this->commsOutageDurationMax);
        }
      }
    }
  }

  this->lastUpdateTime = curTime;
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
  auto myPose = swarmMember->model->GetWorldPose().Ign();

  // Initialize the neighbors list with my own address.
  // A node will always receive its own messages (prob = 1.0).
  swarmMember->neighbors = {{_address, 1.0}};

  // If I am on outage, my only neighbor is myself.
  if (swarmMember->onOutage)
    return;

  // Decide whether this node goes into our neighbor list.
  for (auto const &member : (*this->swarm))
  {
    // Where is the other node?
    auto other = member.second;
    auto otherPose = other->model->GetWorldPose().Ign();

    // This is my own address and it's already in the list of neighbors.
    if (other->address == _address)
      continue;

    // How far away is it from me?
    auto dist = (myPose.Pos() - otherPose.Pos()).Length();
    auto neighborDist = dist;
    auto commsDist = dist;
    int numWalls = 0;
    int numTrees = 0;

    // Check if the other teammate is currenly on outage.
    if (other->onOutage)
      continue;

    // Apply the neighbor part of the comms model.
    if (!ignition::math::equal(neighborDistancePenaltyWall, 0.0))
    {
      // We're within range.  Check for obstacles (don't want to waste time on
      // that if we're not within range).
      numWalls = this->NumWallsBetweenPoses(myPose, otherPose);
      if ((numWalls > 0) && (this->neighborDistancePenaltyWall < 0.0))
        continue;
      else
        neighborDist += numWalls * this->neighborDistancePenaltyWall;
    }
    if (!ignition::math::equal(this->neighborDistancePenaltyTree, 0.0))
    {
      // We're within range.  Check for obstacles (don't want to waste time on
      // that if we're not within range).
      numTrees = this->NumTreesBetweenPoses(myPose, otherPose);
      if ((numTrees > 0) && (this->neighborDistancePenaltyTree < 0.0))
        continue;
      else
        neighborDist += numTrees * this->neighborDistancePenaltyTree;
    }
    if ((this->neighborDistanceMin > 0.0) &&
        (this->neighborDistanceMin > neighborDist))
      continue;
    if ((this->neighborDistanceMax >= 0.0) &&
        (this->neighborDistanceMax < neighborDist))
      continue;

    // Now apply the comms model to compute a probability of a packet from
    // this neighbor arriving successfully.
    auto commsProb = 1.0;

    if ((commsProb > 0.0) &&
        (!ignition::math::equal(this->commsDistancePenaltyWall, 0.0)))
    {
      // We're within range.  Check for obstacles (don't want to waste time on11
      // that if we're not within range).
      if ((numWalls > 0) && (this->commsDistancePenaltyWall < 0.0))
        commsProb = 0.0;
      else
        commsDist += numWalls * this->commsDistancePenaltyWall;
    }
    if ((commsProb > 0.0) &&
        (!ignition::math::equal(this->commsDistancePenaltyTree, 0.0)))
    {
      // We're within range.  Check for obstacles (don't want to waste time on
      // that if we're not within range).
      if ((numTrees > 0) && (this->commsDistancePenaltyTree < 0.0))
        commsProb = 0.0;
      else
        commsDist += numTrees * this->commsDistancePenaltyTree;
    }
    if ((commsProb > 0.0) &&
        (this->commsDistanceMin > 0.0) &&
        (this->commsDistanceMin > commsDist))
      commsProb = 0.0;
    if ((commsProb > 0.0) &&
        (this->commsDistanceMax >= 0.0) &&
        (this->commsDistanceMax < commsDist))
      commsProb = 0.0;

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

//////////////////////////////////////////////////
unsigned int CommsModel::NumWallsBetweenPoses(
  const ignition::math::Pose3d &/*_p1*/, const ignition::math::Pose3d &/*_p2*/)
{
  // TODO: raytrace to answer this question
  return 0;
}

//////////////////////////////////////////////////
unsigned int CommsModel::NumTreesBetweenPoses(
  const ignition::math::Pose3d &/*_p1*/, const ignition::math::Pose3d &/*_p2*/)
{
  // TODO: raytrace to answer this question
  return 0;
}

//////////////////////////////////////////////////
void CommsModel::LoadParameters(sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sdf, "CommsModel::LoadParameters() error: _sdf pointer is NULL");

  if (_sdf->HasElement("comms_model"))
  {
    auto const &commsModelElem = _sdf->GetElement("comms_model");

    if (commsModelElem->HasElement("neighbor_distance_min"))
    {
      this->neighborDistanceMin =
        commsModelElem->Get<double>("neighbor_distance_min");
    }
    if (commsModelElem->HasElement("neighbor_distance_max"))
    {
      this->neighborDistanceMax =
        commsModelElem->Get<double>("neighbor_distance_max");
    }
    if (commsModelElem->HasElement("neighbor_distance_penalty_wall"))
    {
      this->neighborDistancePenaltyWall =
        commsModelElem->Get<double>("neighbor_distance_penalty_wall");
    }
    if (commsModelElem->HasElement("neighbor_distance_penalty_tree"))
    {
      this->neighborDistancePenaltyTree =
        commsModelElem->Get<double>("neighbor_distance_penalty_tree");
    }
    if (commsModelElem->HasElement("comms_distance_min"))
    {
      this->commsDistanceMin =
        commsModelElem->Get<double>("comms_distance_min");
    }
    if (commsModelElem->HasElement("comms_distance_max"))
    {
      this->commsDistanceMax =
        commsModelElem->Get<double>("comms_distance_max");
    }
    if (commsModelElem->HasElement("comms_distance_penalty_wall"))
    {
      this->commsDistancePenaltyWall =
        commsModelElem->Get<double>("comms_distance_penalty_wall");
    }
    if (commsModelElem->HasElement("comms_distance_penalty_tree"))
    {
      this->commsDistancePenaltyTree =
        commsModelElem->Get<double>("comms_distance_penalty_tree");
    }
    if (commsModelElem->HasElement("comms_drop_probability_min"))
    {
      this->commsDropProbabilityMin =
        commsModelElem->Get<double>("comms_drop_probability_min");
    }
    if (commsModelElem->HasElement("comms_drop_probability_max"))
    {
      this->commsDropProbabilityMax =
        commsModelElem->Get<double>("comms_drop_probability_max");
    }
    if (commsModelElem->HasElement("comms_outage_probability"))
    {
      this->commsOutageProbability =
        commsModelElem->Get<double>("comms_outage_probability");
    }
    if (commsModelElem->HasElement("comms_outage_duration_min"))
    {
      this->commsOutageDurationMin =
        commsModelElem->Get<double>("comms_outage_duration_min");
    }
    if (commsModelElem->HasElement("comms_outage_duration_max"))
    {
      this->commsOutageDurationMax =
        commsModelElem->Get<double>("comms_outage_duration_max");
    }
  }
}
