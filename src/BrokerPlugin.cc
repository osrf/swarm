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
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/math/Rand.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/transport.hh>
#include <sdf/sdf.hh>
#include "msgs/datagram.pb.h"
#include "msgs/neighbor_v.pb.h"
#include "swarm/BrokerPlugin.hh"

using namespace swarm;

GZ_REGISTER_WORLD_PLUGIN(BrokerPlugin)

//////////////////////////////////////////////////
void BrokerPlugin::Load(gazebo::physics::WorldPtr _world,
                             sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "BrokerPlugin::Load() error: _world pointer is NULL");
  GZ_ASSERT(_sdf, "BrokerPlugin::Load() error: _sdf pointer is NULL");

  this->world = _world;

  // This is the subscription that will allow us to receive incoming messages.
  const std::string kBrokerIncomingTopic = "/swarm/broker/incoming";
  if (!this->node.Subscribe(kBrokerIncomingTopic,
      &BrokerPlugin::OnMsgReceived, this))
  {
    gzerr << "BrokerPlugin::Load(): Error trying to subscribe"
          << " on topic " << kBrokerIncomingTopic << std::endl;
  }

  // Get the addresses of the swarm.
  this->ReadSwarmFromSDF(_sdf);

  // Get the comms model parameters.
  if (_sdf->HasElement("comms_model"))
  {
    auto const &commsModelElem = _sdf->GetElement("comms_model");

    if (commsModelElem->HasElement("neighbor_distance_min"))
      this->commsModel.neighborDistanceMin =
        commsModelElem->GetElement("neighbor_distance_min")->Get<double>();
    if (commsModelElem->HasElement("neighbor_distance_max"))
      this->commsModel.neighborDistanceMax =
        commsModelElem->GetElement("neighbor_distance_max")->Get<double>();
    if (commsModelElem->HasElement("neighbor_distance_penalty_wall"))
      this->commsModel.neighborDistancePenaltyWall =
        commsModelElem->GetElement("neighbor_distance_penalty_wall")->Get<double>();
    if (commsModelElem->HasElement("neighbor_distance_penalty_tree"))
      this->commsModel.neighborDistancePenaltyTree =
        commsModelElem->GetElement("neighbor_distance_penalty_tree")->Get<double>();
    if (commsModelElem->HasElement("comms_distance_min"))
      this->commsModel.commsDistanceMin =
        commsModelElem->GetElement("comms_distance_min")->Get<double>();
    if (commsModelElem->HasElement("comms_distance_max"))
      this->commsModel.commsDistanceMax =
        commsModelElem->GetElement("comms_distance_max")->Get<double>();
    if (commsModelElem->HasElement("comms_distance_penalty_wall"))
      this->commsModel.commsDistancePenaltyWall =
        commsModelElem->GetElement("comms_distance_penalty_wall")->Get<double>();
    if (commsModelElem->HasElement("comms_distance_penalty_tree"))
      this->commsModel.commsDistancePenaltyTree =
        commsModelElem->GetElement("comms_distance_penalty_tree")->Get<double>();
    if (commsModelElem->HasElement("comms_drop_probability_min"))
      this->commsModel.commsDropProbabilityMin =
        commsModelElem->GetElement("comms_drop_probability_min")->Get<double>();
    if (commsModelElem->HasElement("comms_drop_probability_max"))
      this->commsModel.commsDropProbabilityMax =
        commsModelElem->GetElement("comms_drop_probability_max")->Get<double>();
    if (commsModelElem->HasElement("comms_outage_probability"))
      this->commsModel.commsOutageProbability =
        commsModelElem->GetElement("comms_outage_probability")->Get<double>();
    if (commsModelElem->HasElement("comms_outage_duration_min"))
      this->commsModel.commsOutageDurationMin =
        commsModelElem->GetElement("comms_outage_duration_min")->Get<double>();
    if (commsModelElem->HasElement("comms_outage_duration_max"))
      this->commsModel.commsOutageDurationMax =
        commsModelElem->GetElement("comms_outage_duration_max")->Get<double>();
  }

  // Listen to the update event broadcasted every simulation iteration.
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&BrokerPlugin::Update, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void BrokerPlugin::ReadSwarmFromSDF(sdf::ElementPtr _sdf)
{
  sdf::ElementPtr worldSDF = _sdf->GetParent();
  if (!worldSDF)
  {
    gzerr << "BrokerPlugin::ReadSwarmFromSDF() Unable to read world SDF\n";
    return;
  }

  // Iterate over all the models looking for <address> inside <plugin>.
  auto modelElem = worldSDF->GetElement("model");
  while (modelElem)
  {
    if (modelElem->HasElement("plugin") && modelElem->HasAttribute("name"))
    {
      auto const &pluginElem = modelElem->GetElement("plugin");
      if (pluginElem->HasElement("address"))
      {
        std::string address = pluginElem->Get<std::string>("address");
        std::string name = modelElem->GetAttribute("name")->GetAsString();
        auto const &model = this->world->GetModel(name);
        if (model)
        {
          // Create a new SwarmMember for storing the vehicle's information.
          auto newMember = std::make_shared<SwarmMember>();
          newMember->address = address;
          newMember->name = name;
          newMember->model = model;
          this->swarm[address] = newMember;

          // Advertise the topic for future neighbor updates for this vehicle.
          std::string topic = "/swarm/" + address + "/neighbors";
          this->node.Advertise(topic);
        }
        else
        {
          gzerr << "BrokerPlugin::ReadSwarmFromSDF(): Error getting a model"
                << "pointer for robot [" << name << "]" << std::endl;
        }
      }
    }

    modelElem = modelElem->GetNextElement("model");
  }

  if (this->swarm.empty())
    gzerr << "BrokerPlugin::ReadSwarmFromSDF: No members found" << std::endl;
  else
    gzmsg << "BrokerPlugin::ReadSwarmFromSDF: " << this->swarm.size()
          << " swarm members found" << std::endl;
}

//////////////////////////////////////////////////
void BrokerPlugin::Update(const gazebo::common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Update the list of neighbors for each robot.
  for (auto const &robot : this->swarm)
    this->UpdateNeighborList(robot.first);
  printf("BrokerPlugin::Update\n");

  // Dispatch all incoming messages.
  while (!this->incomingMsgs.empty())
  {
    // Get the next message to dispatch.
    auto msg = this->incomingMsgs.front();
    this->incomingMsgs.pop();

    // Sanity check: Make sure that the sender is a member of the swarm.
    if (this->swarm.find(msg.src_address()) == this->swarm.end())
    {
      gzerr << "BrokerPlugin::Update(): Discarding message. Robot ["
            << msg.src_address() << "] is not registered as a member of the "
            << "swarm" << std::endl;
      continue;
    }

    // Add the list of neighbors of the sender to the outgoing message.
    for (auto const &neighbor : this->swarm[msg.src_address()]->neighbors)
    {
      msg.add_neighbors(neighbor.first);
      msg.add_neighbor_probabilities(neighbor.second);
    }

    // Create the topic name for the message destination.
    const std::string topic =
        "/swarm/" + msg.dst_address() + "/" + std::to_string(msg.dst_port());

    // Forward the message to the destination.
    this->node.Advertise(topic);
    this->node.Publish(topic, msg);
  }
  printf("/BrokerPlugin::Update\n");
}

//////////////////////////////////////////////////
void BrokerPlugin::UpdateNeighborList(const std::string &_address)
{
  printf("BrokerPlugin::UpdateNeighborList\n");

  GZ_ASSERT(this->swarm.find(_address) != this->swarm.end(),
            "_address not found in the swarm.");

  auto swarmMember = this->swarm[_address];

  auto myPose = swarmMember->model->GetWorldPose();

  //gzmsg << "UpdateNeighborList: " << swarmMember->name << std::endl;

  // Update the neighbor list for this robot.
  swarmMember->neighbors.clear();
  for (auto const &member : this->swarm)
  {
    printf("BrokerPlugin::UpdateNeighborList: %s\n", member.first.c_str());
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

    // Apply the neighbor part of the comms model
    auto neighbor = true;
    if (neighbor &&
        (this->commsModel.neighborDistanceMin >= 0.0) &&
        (this->commsModel.neighborDistanceMin > neighborDist))
      neighbor = false;
    if (neighbor &&
        (this->commsModel.neighborDistanceMax >= 0.0) &&
        (this->commsModel.neighborDistanceMax < neighborDist))
      neighbor = false;
    if (neighbor && this->commsModel.neighborDistancePenaltyWall > 0.0)
    {
      // We're within range.  Check for obstacles (don't want to waste time on
      // that if we're not within range).
      numWalls = this->NumWallsBetweenPoses(myPose, otherPose);
      if ((numWalls > 0) &&
          (this->commsModel.neighborDistancePenaltyWall < 0.0))
        neighbor = false;
      else
        neighborDist -= numWalls * this->commsModel.neighborDistancePenaltyWall;
      if (neighbor &&
          (this->commsModel.neighborDistanceMin >= 0.0) &&
          (this->commsModel.neighborDistanceMin > neighborDist))
        neighbor = false;
      if (neighbor &&
          (this->commsModel.neighborDistanceMax >= 0.0) &&
          (this->commsModel.neighborDistanceMax < neighborDist))
        neighbor = false;
    }
    if (neighbor && this->commsModel.neighborDistancePenaltyTree > 0.0)
    {
      // We're within range.  Check for obstacles (don't want to waste time on
      // that if we're not within range).
      numTrees = this->NumTreesBetweenPoses(myPose, otherPose);
      if ((numTrees > 0) &&
          (this->commsModel.neighborDistancePenaltyTree < 0.0))
        neighbor = false;
      else
        neighborDist -= numTrees * this->commsModel.neighborDistancePenaltyTree;
      if (neighbor &&
          (this->commsModel.neighborDistanceMin >= 0.0) &&
          (this->commsModel.neighborDistanceMin > neighborDist))
        neighbor = false;
      if (neighbor &&
          (this->commsModel.neighborDistanceMax >= 0.0) &&
          (this->commsModel.neighborDistanceMax < neighborDist))
        neighbor = false;
    }

    if (neighbor)
    {
      // Now apply the comms model to compute a probability of a packet from
      // this neighbor arriving successfully.
      auto neighborProb = 1.0;

      // TODO: apply outage model first, short-circuiting the rest of the work

      if ((neighborProb > 0.0) &&
          (this->commsModel.commsDistanceMin >= 0.0) &&
          (this->commsModel.commsDistanceMin > commsDist))
        neighborProb = 0.0;
      if ((neighborProb > 0.0) &&
          (this->commsModel.commsDistanceMax >= 0.0) &&
          (this->commsModel.commsDistanceMax < commsDist))
        neighborProb = 0.0;
      if ((neighborProb > 0.0) && this->commsModel.commsDistancePenaltyWall > 0.0)
      {
        // We're within range.  Check for obstacles (don't want to waste time on
        // that if we're not within range).
        if ((numWalls > 0) &&
            (this->commsModel.commsDistancePenaltyWall < 0.0))
          neighborProb = 0.0;
        else
          commsDist -= numWalls * this->commsModel.commsDistancePenaltyWall;
        if ((neighborProb > 0.0) &&
            (this->commsModel.commsDistanceMin >= 0.0) &&
            (this->commsModel.commsDistanceMin > commsDist))
          neighborProb = 0.0;
        if ((neighborProb > 0.0) &&
            (this->commsModel.commsDistanceMax >= 0.0) &&
            (this->commsModel.commsDistanceMax < commsDist))
          neighborProb = 0.0;
      }
      if ((neighborProb > 0.0) && this->commsModel.commsDistancePenaltyTree > 0.0)
      {
        // We're within range.  Check for obstacles (don't want to waste time on
        // that if we're not within range).
        if ((numTrees > 0) &&
            (this->commsModel.commsDistancePenaltyTree < 0.0))
          neighborProb = 0.0;
        else
          commsDist -= numTrees * this->commsModel.commsDistancePenaltyTree;
        if ((neighborProb > 0.0) &&
            (this->commsModel.commsDistanceMin >= 0.0) &&
            (this->commsModel.commsDistanceMin > commsDist))
          neighborProb = 0.0;
        if ((neighborProb > 0.0) &&
            (this->commsModel.commsDistanceMax >= 0.0) &&
            (this->commsModel.commsDistanceMax < commsDist))
          neighborProb = 0.0;
      }

      printf("neighborProb: %f\n", neighborProb);
      if (neighborProb > 0.0)
      {
        // We made it through the outage, distance, and obstacle filters.
        // Compute a drop probability for this time step.
        if (this->commsModel.commsDropProbabilityMax > 0.0)
        {
          printf("GetDblUniform\n");
          neighborProb = 1.0 - gazebo::math::Rand::GetDblUniform(
            this->commsModel.commsDropProbabilityMin,
            this->commsModel.commsDropProbabilityMax);
          printf("/GetDblUniform\n");
        }
      }

      gzmsg << other->name << " : " << neighborProb << std::endl;

      // Stuff the resulting information into our local representation of this
      // node.  A message containing this information will be sent out below.
      std::pair<std::string, double> neighborAndProb;
      neighborAndProb.first = member.first;
      neighborAndProb.second = neighborProb;
      swarmMember->neighbors.push_back(neighborAndProb);
    }
  }

  // Fill the message with the new neighbor list.
  swarm::msgs::Neighbor_V msg;
  for (auto const &neighbor : swarmMember->neighbors)
  {
    msg.add_neighbors(neighbor.first);
    msg.add_neighbor_probabilities(neighbor.second);
  }

  // Notify the node with its updated list of neighbors.
  std::string topic = "/swarm/" + swarmMember->address + "/neighbors";
  this->node.Publish(topic, msg);
}

//////////////////////////////////////////////////
void BrokerPlugin::OnMsgReceived(const std::string &/*_topic*/,
    const msgs::Datagram &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Queue the new message.
  this->incomingMsgs.push(_msg);
}

unsigned int BrokerPlugin::NumWallsBetweenPoses(const gazebo::math::Pose& p1,
                                                const gazebo::math::Pose& p2)
{
  // TODO: raytrace to answer this question
  return 0;
}

unsigned int BrokerPlugin::NumTreesBetweenPoses(const gazebo::math::Pose& p1,
                                                const gazebo::math::Pose& p2)
{
  // TODO: raytrace to answer this question
  return 0;
}

//////////////////////////////////////////////////
CommsModel::CommsModel()
{
  // Default to perfect comms.
  this->neighborDistanceMin = -1.0;
  this->neighborDistanceMax = -1.0;
  this->neighborDistancePenaltyWall = 0.0;
  this->neighborDistancePenaltyTree = 0.0;
  this->commsDistanceMin = -1.0;
  this->commsDistanceMax = -1.0;
  this->commsDistancePenaltyWall = 0.0;
  this->commsDistancePenaltyTree = 0.0;
  this->commsDropProbabilityMin = 0.0;
  this->commsDropProbabilityMax = 0.0;
  this->commsOutageProbability = 0.0;
  this->commsOutageDurationMin = -1.0;
  this->commsOutageDurationMax = -1.0;
}
