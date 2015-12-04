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

#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <sdf/sdf.hh>

#include "swarm/Broker.hh"
#include "swarm/CommsModel.hh"
#include "swarm/Logger.hh"
#include "swarm/SwarmTypes.hh"
#include "msgs/log_entry.pb.h"

namespace swarm
{
  /// \brief This is a world plugin designed to centralize all the messages
  /// sent by the members of the swarm. This plugin subscribes to the
  /// "/swarm/broker/incoming" topic, on which all the agents publish their
  /// messages. This plugin receives and queues the messages. The broker
  /// uses its Update() function to dispatch the queued messages.
  /// Dispatch of a message may directly deliver it to
  /// the destination/s node/s or it may forward the message to a
  /// network simulator (ns-3).
  class IGNITION_VISIBLE BrokerPlugin
    : public gazebo::WorldPlugin, public swarm::Loggable
  {
    /// \brief Class constructor.
    public: BrokerPlugin() = default;

    /// \brief Class destructor.
    public: virtual ~BrokerPlugin();

    // Documentation Inherited.
    public: virtual void Load(gazebo::physics::WorldPtr _world,
                              sdf::ElementPtr _sdf);

    /// \brief Handle reset.
    public: virtual void Reset();

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

    /// \brief Send a message to each swarm member
    /// with its updated neighbors list.
    private: void NotifyNeighbors();

    /// \brief Dispatch all incoming messages.
    private: void DispatchMessages();

    // Documentation inherited.
    private: virtual void OnLog(msgs::LogEntry &_logEntry) const;

    /// \brief Address used by the Broker plugin.
    protected: const uint32_t kBroker = 5;

    /// \brief World pointer.
    private: gazebo::physics::WorldPtr world;

    /// \brief SDF for this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief Pointer to the update event connection.
    private: gazebo::event::ConnectionPtr updateConnection;

    /// \brief Information about the members of the swarm.
    private: SwarmMembershipPtr swarm;

    /// \brief Mutex for protecting the queue.
    private: std::mutex mutex;

    /// \brief Comms model that we're using.
    private: std::unique_ptr<CommsModel> commsModel;

    /// \brief Incoming messages from other robots used for logging.
    private: msgs::IncomingMsgs logIncomingMsgs;

    /// \brief Broker instance.
    private: Broker *broker = Broker::Instance();

    /// \brief Logger instance.
    private: Logger *logger = Logger::Instance();

    /// \brief Maximum data rate allowed per simulation cycle (bits).
    private: uint32_t maxDataRatePerCycle;

    /// \brief Random engine used to shuffle the messages.
    private: std::default_random_engine rndEngine;
  };
}
#endif
