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

/// \file BrokerPlugin.hh
/// \brief Internal broker to dispatch messages inside the Swarm project.

#ifndef __SWARM_BROKER_PLUGIN_HH__
#define __SWARM_BROKER_PLUGIN_HH__

#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ignition/transport.hh>
#include <sdf/sdf.hh>
#include "msgs/datagram.pb.h"

namespace swarm
{
  /// \brief Class used to store information about a member of the Swarm.
  class IGNITION_VISIBLE SwarmMember
  {
    /// \brief Gazebo name used for this model.
    public: std::string name;

    /// \brief Address of the robot. E.g.: 192.168.1.2
    public: std::string address;

    /// \brief Model pointer.
    public: gazebo::physics::ModelPtr model;

    /// \brief List of neighbors and comms probabilities for this robot.
    public: std::vector<std::pair<std::string, double> > neighbors;

    /// \brief Is this robot on outage?
    public: bool onOutage;

    /// \brief When will the last outage finish?
    public: gazebo::common::Time onOutageUntil;
  };

  /// \brief Class used to store information about the communication model.
  class IGNITION_VISIBLE CommsModel
  {
    /// \brief Class constructor
    public: CommsModel();

    /// \brief Minimum free-space distance (m) between two nodes to be
    /// neighbors. Set to <0 for no limit.
    public: double neighborDistanceMin;

    /// \brief Maximum free-space distance (m) between two nodes to be
    /// neighbors. Set to <0 for no limit.
    public: double neighborDistanceMax;

    /// \brief Equivalent free space distance (m) that is "consumed" by an
    /// intervening solid obstacle (wall, terrain, etc.).
    /// Set to <0 for infinite penalty (i.e., one obstacle blocks neighbors).
    public: double neighborDistancePenaltyWall;

    /// \brief Equivalent free space distance (m) that is "consumed" by an
    /// intervening non-solid obstacle (forest, etc.).
    /// Set to <0 for infinite penalty (i.e., one obstacle blocks neighbors).
    public: double neighborDistancePenaltyTree;

    /// \brief Minimum free-space distance (m) between two nodes to communicate
    /// (must also be neighbors).  Set to <0 for no limit.
    public: double commsDistanceMin;

    /// \brief Maximum free-space distance (m) between two nodes to communicate
    /// (must also be neighbors).  Set to <0 for no limit.
    public: double commsDistanceMax;

    /// \brief Equivalent free space distance (m) that is "consumed" by an
    /// intervening solid obstacle (wall, terrain, etc.).
    /// Set to <0 for infinite penalty (i.e., one obstacle blocks comms).
    public: double commsDistancePenaltyWall;

    /// \brief Equivalent free space distance (m) that is "consumed" by an
    /// intervening non-solid obstacle (forest, etc.).
    /// Set to <0 for infinite penalty (i.e., one obstacle blocks comms).
    public: double commsDistancePenaltyTree;

    /// \brief Minimum probability of dropping a given packet.
    /// Used with uniform drop probability model.
    public: double commsDropProbabilityMin;

    /// \brief Maximum probability of dropping a given packet.
    /// Used with uniform drop probability model.
    public: double commsDropProbabilityMax;

    /// \brief Probability of going into a comms outage at each time step.
    public: double commsOutageProbability;

    /// \brief Minimum length of comms outage.  Set to <0 for no limit.
    /// Used with uniform outage duration probability model.
    public: double commsOutageDurationMin;

    /// \brief Maximum length of comms outage. Set to <0 for no limit.
    /// Used with uniform outage duration probability model.
    public: double commsOutageDurationMax;
  };

  /// \brief This is a world plugin designed to centralize all the messages
  /// sent by the members of the swarm. This plugin subscribes to the
  /// "/swarm/broker/incoming" topic, on which all the agents publish their
  /// messages. This plugin receives and queues the messages. The broker
  /// uses its Update() function to dispatch the queued messages.
  /// Dispatch of a message may directly deliver it to
  /// the destination/s node/s or it may forward the message to a
  /// network simulator (ns-3).
  class IGNITION_VISIBLE BrokerPlugin : public gazebo::WorldPlugin
  {
    /// \brief Class constructor.
    public: BrokerPlugin() = default;

    /// \brief Class destructor.
    public: virtual ~BrokerPlugin() = default;

    // Documentation Inherited.
    public: virtual void Load(gazebo::physics::WorldPtr _world,
                              sdf::ElementPtr _sdf);

    /// \brief Parse the SDF world file and update a map with information about
    /// all the members of the swarm. For each member of the swarm we store its
    /// gazebo model name, its address, a pointer to the gazebo model and its
    /// list of neighbors. The key of the map is the address of the robot.
    ///
    /// \param[in] _sdf SDF for this plugin.
    private: void ReadSwarmFromSDF(sdf::ElementPtr _sdf);

    /// \brief Update callback for the plugin.
    ///
    /// \param[in] _info Update information provided by the server.
    private: void Update(const gazebo::common::UpdateInfo &_info);

    /// \brief Update the neighbor list for a single robot and notifies the
    /// robot with the updated list.
    ///
    /// \param[in] _address Address of the robot to be updated.
    private: void UpdateNeighborList(const std::string &_address);

    /// \brief Callback executed when a new message is received.
    ///
    /// \param[in] _topic Topic name associated to the new message received.
    /// \param[in] _msg The new message received.
    private: void OnMsgReceived(const std::string &_topic,
                                const msgs::Datagram &_msg);

    private: unsigned int NumWallsBetweenPoses(const gazebo::math::Pose& p1,
                                               const gazebo::math::Pose& p2);

    private: unsigned int NumTreesBetweenPoses(const gazebo::math::Pose& p1,
                                               const gazebo::math::Pose& p2);

    /// \brief World pointer.
    private: gazebo::physics::WorldPtr world;

    /// \brief SDF for this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief Pointer to the update event connection.
    private: gazebo::event::ConnectionPtr updateConnection;

    /// \brief Pointer to a node for communication.
    private: ignition::transport::Node node;

    /// \brief Queue to store the incoming messages received from the agents.
    private: std::queue<msgs::Datagram> incomingMsgs;

    /// \brief Map containing information about the members of the swarm.
    /// The key is the robot address. The value is a pointer to a SwarmMember
    /// object that contains multiple information about the robot.
    private: std::map<std::string, std::shared_ptr<SwarmMember>> swarm;

    /// \brief Mutex for protecting the queue.
    private: std::mutex mutex;

    /// \brief Comms model that we're using.
    private: CommsModel commsModel;
  };
}
#endif
