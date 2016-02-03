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

#include <algorithm>
#include <sys/stat.h>
#include <string>
#include <utility>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

#include "swarm/VisibilityTable.hh"
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

  // Sanity check: Confirm that the penalty for crossing two lines of trees is
  // bigger than the maximum neighbor distance.
  if (this->neighborDistancePenaltyTree * 2 <= this->neighborDistanceMax)
  {
    std::cerr << "The combination of <neighborDistancePenaltyTree> and "
              << " <neighborDistanceMax> violates the rule that no more than "
              << "one line of trees is allowed between vehicles to communicate "
              << "(2*neighborDistancePenaltyTree > neighborDistanceMax)."
              << std::endl;
  }

  // Sanity check: Confirm that the penalty for crossing two lines of trees is
  // bigger than the maximum comms distance.
  if (this->commsDistancePenaltyTree * 2 <= this->commsDistanceMax)
  {
    std::cerr << "The combination of <commsDistancePenaltyTree> and "
              << " <commsDistanceMax> violates the rule that no more than "
              << "one line of trees is allowed between vehicles to communicate "
              << "(2*commsDistancePenaltyTree > commsDistanceMax)."
              << std::endl;
  }

  this->CacheVisibilityPairs();

  // Initialize visibility.
  for (auto const &robotA : (*this->swarm))
  {
    auto addressA = robotA.second->address;
    auto row = this->visibilityMsg.add_row();
    row->set_src(addressA);

    for (auto const &robotB : (*this->swarm))
    {
      auto addressB = robotB.second->address;
      this->visibility[std::make_pair(addressA, addressB)] = false;

      // Create a new visibility entry for logging with a VISIBLE status.
      auto visibilityEntry = row->add_entry();
      visibilityEntry->set_dst(addressB);
      visibilityEntry->set_status(msgs::CommsStatus::VISIBLE);

      this->visibilityMsgStatus[addressA][addressB] = visibilityEntry;
    }
  }

  std::string tableFilename = "/tmp/visibility.dat";
  struct stat buffer;
  if (stat(tableFilename.c_str(), &buffer) != 0)
  {
    std::cout << "Generating visibility table[/tmp/visibility.dat]\n.";
    VisibilityTable table;
    table.Generate();
  }

  // Read the visibility table information
  std::ifstream tableIn(tableFilename, std::ios::binary | std::ios::in);
  tableIn.read(reinterpret_cast<char*>(&this->visibilityTableMaxY),
      sizeof(int));
  tableIn.read(reinterpret_cast<char*>(&this->visibilityTableStepSize),
      sizeof(int));
  tableIn.read(reinterpret_cast<char*>(&this->visibilityTableRowSize),
      sizeof(int));

  while (true)
  {
    uint64_t data;
    tableIn.read((char*)(&data), sizeof(uint64_t));
    if (!tableIn.eof())
      this->visibilityTable.emplace(data);
    else
      break;
  }

  // Get all the trees
  for (auto const &model : this->world->GetModels())
  {
    if (model->GetName().find("tree") != std::string::npos)
    {
      this->trees.push_back(model->GetBoundingBox().Ign());
    }
    else if (model->GetName().find("building") != std::string::npos)
    {
      this->buildings.push_back(model->GetBoundingBox().Ign());
    }
  }
}

//////////////////////////////////////////////////
void CommsModel::Update()
{
  // Decide if each member of the swarm enters into a comms outage.
  this->UpdateOutages();

  // Update the visibility state between vehicles.
  // Make sure that this happens after UpdateOutages().
  this->UpdateVisibility();

  // Update the neighbors list of each member of the swarm.
  // Make sure that this happens after UpdateVisibility().
  this->UpdateNeighbors();
}

//////////////////////////////////////////////////
const msgs::VisibilityMap &CommsModel::VisibilityMap() const
{
  return this->visibilityMsg;
}

//////////////////////////////////////////////////
uint32_t CommsModel::MaxDataRate() const
{
  return this->commsDataRateMax;
}

//////////////////////////////////////////////////
uint16_t CommsModel::UdpOverhead() const
{
  return this->udpOverhead;
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

    // Check if I am currently on a temporary outage.
    if (swarmMember->onOutage &&
        swarmMember->onOutageUntil != gazebo::common::Time::Zero)
    {
      // Check if the outage should finish.
      if (this->world->GetSimTime() >= swarmMember->onOutageUntil)
      {
        swarmMember->onOutage = false;

        // Debug output.
        // gzdbg << "[" << curTime << "] Robot " << address
        //       << " is back from an outage." << std::endl;
      }
    }
    else if (!swarmMember->onOutage)
    {
      // Check if we should go into an outage.
      // Notice that "commsOutageProbability" specifies the probability of going
      // into a comms outage at each second. This probability is adjusted using
      // the elapsed time since the last call to "UpdateOutages()".
      if ((ignition::math::equal(this->commsOutageProbability, 1.0)) ||
          (ignition::math::Rand::DblUniform(0.0, 1.0) <
           this->commsOutageProbability * dt.Double()))
      {
        swarmMember->onOutage = true;
        // Debug output.
        // gzdbg << "[" << curTime << "] Robot " << address
        //       << " has started an outage." << std::endl;

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
  unsigned int counter = 0;

  // Update the list of neighbors for each robot.
  while (counter < this->neighborUpdatesPerCycle)
  {
    auto const &address = this->addresses.at(this->neighborIndex);
    this->UpdateNeighborList(address);

    this->neighborIndex = (this->neighborIndex + 1) % this->addresses.size();
    ++counter;
  }
}

//////////////////////////////////////////////////
void CommsModel::UpdateNeighborList(const std::string &_address)
{
  GZ_ASSERT(this->swarm->find(_address) != this->swarm->end(),
            "_address not found in the swarm.");

  auto swarmMember = (*this->swarm)[_address];
  auto myPose = swarmMember->model->GetWorldPose().Ign();

  // Initialize the neighbors list.
  swarmMember->neighbors.clear();

  // Decide whether this node goes into our neighbor list.
  for (auto const &member : (*this->swarm))
  {
    // Where is the other node?
    auto other = member.second;

    // Update this visibility entry with a VISIBLE status.
    auto visibilityEntry = this->visibilityMsgStatus[_address][other->address];
    visibilityEntry->set_status(msgs::CommsStatus::VISIBLE);

    // Both robots are in an outage.
    if (swarmMember->onOutage && other->onOutage)
    {
      visibilityEntry->set_status(msgs::CommsStatus::OUTAGE_BOTH);
      continue;
    }
    // I'm in an outage.
    else if (swarmMember->onOutage)
    {
      visibilityEntry->set_status(msgs::CommsStatus::OUTAGE);
      continue;
    }
    // The other robot is in an outage.
    else if (other->onOutage)
    {
      visibilityEntry->set_status(msgs::CommsStatus::OUTAGE_DST);
      continue;
    }

    // Do not include myself in the list of neighbors.
    if (other->address == _address)
      continue;

    // Check if there's line of sight between the two vehicles.
    // If there's no line of sight, guess what type of object is in between.
    auto key = std::make_pair(_address, other->address);
    GZ_ASSERT(this->visibility.find(key) != this->visibility.end(),
      "vehicle key not found in visibility");

    bool visible = this->visibility[key];
    if (!visible)
    {
      visibilityEntry->set_status(msgs::CommsStatus::OBSTACLE);
      continue;
    }

    auto otherPose = other->model->GetWorldPose().Ign();
    bool treesBlocking;
    double dist;

    // Check tree and building interference
    this->CheckObstacles(myPose.Pos(), otherPose.Pos(),
                         visible, treesBlocking, dist);

    if (!visible)
    {
      visibilityEntry->set_status(msgs::CommsStatus::OBSTACLE);
      continue;
    }

    auto neighborDist = dist;
    auto commsDist = dist;

    // Apply the neighbor part of the comms model.
    if (!ignition::math::equal(this->neighborDistancePenaltyTree, 0.0))
    {
      // We're within range.  Check for obstacles (don't want to waste time on
      // that if we're not within range).
      if (treesBlocking)
      {
        if (this->neighborDistancePenaltyTree < 0.0)
        {
          visibilityEntry->set_status(msgs::CommsStatus::DISTANCE);
          continue;
        }
        else
          neighborDist += this->neighborDistancePenaltyTree;
      }
    }

    if ((this->neighborDistanceMin > 0.0) &&
        (this->neighborDistanceMin > neighborDist))
    {
      visibilityEntry->set_status(msgs::CommsStatus::DISTANCE);
      continue;
    }
    if ((this->neighborDistanceMax >= 0.0) &&
        (this->neighborDistanceMax < neighborDist))
    {
      visibilityEntry->set_status(msgs::CommsStatus::DISTANCE);
      continue;
    }

    // Now apply the comms model to compute a probability of a packet from
    // this neighbor arriving successfully.
    auto commsProb = 1.0;

    if ((commsProb > 0.0) &&
        (!ignition::math::equal(this->commsDistancePenaltyTree, 0.0)))
    {
      // We're within range.  Check for obstacles (don't want to waste time on
      // that if we're not within range).
      if (treesBlocking)
      {
        if (this->commsDistancePenaltyTree < 0.0)
          commsProb = 0.0;
        else
          commsDist += this->commsDistancePenaltyTree;
      }
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
    swarmMember->neighbors[member.first] = commsProb;
  }
}

//////////////////////////////////////////////////
void CommsModel::CacheVisibilityPairs()
{
  for (auto const &robotA : (*this->swarm))
  {
    auto addressA = robotA.second->address;
    this->addresses.push_back(addressA);
    for (auto const &robotB : (*this->swarm))
    {
      auto addressB = robotB.second->address;
      std::pair<std::string, std::string> aPair(addressA, addressB);
      std::pair<std::string, std::string> aPairInverse(addressB, addressA);

      // Do not include this case.
      if (addressA == addressB)
        continue;

      // Check if we already have the symmetric pair stored.
      if (std::find(this->visibilityPairs.begin(), this->visibilityPairs.end(),
            aPairInverse) != this->visibilityPairs.end())
      {
        continue;
      }

      this->visibilityPairs.push_back(aPair);
    }
  }

  this->visibilityUpdatesPerCycle = this->visibilityPairs.size() /
      ((1.0 / this->updateRate) /
       this->world->GetPhysicsEngine()->GetMaxStepSize());

  this->neighborUpdatesPerCycle = this->swarm->size() /
      ((1.0 / this->updateRate) /
       this->world->GetPhysicsEngine()->GetMaxStepSize());

  if (this->visibilityPairs.size() > 0)
  {
    // Make sure that we update at least one element in each update.
    this->visibilityUpdatesPerCycle = std::max(1u,
        this->visibilityUpdatesPerCycle);

    this->neighborUpdatesPerCycle = std::max(1u,
        this->neighborUpdatesPerCycle);
  }
}

//////////////////////////////////////////////////
void CommsModel::UpdateVisibility()
{
  unsigned int counter = 0;

  // All combinations between a pair of vehicles.
  while (counter < this->visibilityUpdatesPerCycle)
  {
    auto const &keyA = this->visibilityPairs.at(this->visibilityIndex);
    auto addressA = keyA.first;
    auto addressB = keyA.second;
    auto keyB = std::make_pair(addressB, addressA);

    this->visibilityIndex =
      (this->visibilityIndex + 1) % this->visibilityPairs.size();
    ++counter;

    auto poseA = (*swarm)[addressA]->model->GetWorldPose().Ign();
    auto poseB = (*swarm)[addressB]->model->GetWorldPose().Ign();
    if (poseA.Pos().Distance(poseB.Pos()) <= this->commsDistanceMax)
      this->visibility[keyA] = this->LineOfSight(poseA, poseB);
    else
      this->visibility[keyA] = false;

    // Update the symmetric case.
    this->visibility[keyB] = this->visibility[keyA];
  }
}

//////////////////////////////////////////////////
bool CommsModel::LineOfSight(const ignition::math::Pose3d& _p1,
                             const ignition::math::Pose3d& _p2)
{
  int index1 = this->Index(_p1.Pos());
  int index2 = this->Index(_p2.Pos());

  return
    this->visibilityTable.find(this->Pair(index1, index2))
    == this->visibilityTable.end() &&
    this->visibilityTable.find(this->Pair(index2, index1))
    == this->visibilityTable.end();
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
    if (commsModelElem->HasElement("comms_data_rate_max"))
    {
      this->commsDataRateMax =
        commsModelElem->Get<double>("comms_data_rate_max");
    }
    if (commsModelElem->HasElement("update_rate"))
    {
      this->updateRate = commsModelElem->Get<double>("update_rate");
    }
  }
}

/////////////////////////////////////////////////
uint64_t CommsModel::Pair(const int _a, const int _b)
{
  // Szudzik's function
  return _a >= _b ?  _a * _a + _a + _b : _a + _b * _b;
}

/////////////////////////////////////////////////
int CommsModel::Index(const ignition::math::Vector3d &_p)
{
  int x = std::round(_p.X() / this->visibilityTableStepSize) *
    this->visibilityTableStepSize;

  int y = std::round(_p.Y() / this->visibilityTableStepSize) *
    this->visibilityTableStepSize;

  return ((y + this->visibilityTableMaxY)/ this->visibilityTableStepSize) *
    this->visibilityTableRowSize +
    ((x + this->visibilityTableMaxY)/this->visibilityTableStepSize);
}

/////////////////////////////////////////////////
void CommsModel::CheckObstacles(const ignition::math::Vector3d &_posA,
    const ignition::math::Vector3d &_posB,
    bool &_visible, bool &_treesBlocking, double &_dist)
{
  ignition::math::Vector3d origin = _posA.Pos();
  ignition::math::Vector3d dir = (_posB.Pos() - _posA.Pos()).Normalize();

  _treesBlocking = false;
  bool intersects = false;
  double dist = 0;

  for (auto const &building : this->buildings)
  {
    std::tie(intersects, dist) = building.Intersects(origin, dir);
    if (intersects)
    {
      _visible = false;
      _dist = dist;
      return;
    }
  }

  for (auto const &tree : this->trees)
  {
    std::tie(intersects, dist) = tree.Intersects(origin, dir);
    if (intersects)
    {
      _dist = dist;

      // Not visible if two trees are blocking visibility
      if (_treesBlocking)
      {
        _visible = false;
        return;
      }
      _treesBlocking = true;
    }
  }
}
