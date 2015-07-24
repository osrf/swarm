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

/// \file RobotPlugin.hh
/// \brief Main Swarm API for agent development.

#ifndef __SWARM_ROBOT_PLUGIN_HH__
#define __SWARM_ROBOT_PLUGIN_HH__

#include <functional>
#include <map>
#include <string>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ignition/transport.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include "msgs/datagram.pb.h"

namespace swarm
{
  /// \brief A Model plugin that is the base class for all agent plugins
  /// in a swarm.
  /// This plugin exposes the following functionality to the derived plugins:
  ///
  /// * Configuration.
  ///     - Load()    This method will allow the agent to read SDF parameters
  ///                 from the model.
  ///
  /// * Communication.
  ///     - Bind()    This method binds an address to a virtual socket, and
  ///                 sends incoming messages to the specified callback.
  ///     - SendTo()  This method allows an agent to send data to other
  ///                 individual agent (unicast), all the agents (broadcast),
  ///                 or a group of agents (multicast).
  ///     - Host() This method will return the agent's address.
  ///
  ///  * Motion.
  ///
  ///  * Sensors.
  ///
  class IGNITION_VISIBLE RobotPlugin : public gazebo::ModelPlugin
  {
    /// \brief The type of vehicle.
    public: enum VehicleType
            {
              /// \brief A ground vehicle.
              GROUND = 0,

              /// \brief A rotorcraft aerial vehicle.
              ROTOR = 1,

              /// \brief A fixed wing aerial vehicle.
              FIXED_WING = 2
            };

    /// \brief Class constructor.
    public: RobotPlugin();

    /// \brief Class destructor.
    public: virtual ~RobotPlugin();

    /// \brief This method is called after the world has been loaded and gives
    /// child plugins access to the SDF model file.
    /// \param[in] _sdf Pointer to the SDF element of the model.
    protected: virtual void Load(sdf::ElementPtr _sdf);

    /// \brief This method can bind a local address and a port to a
    /// virtual socket. This is a required step if your agent needs to
    /// receive messages.
    /// \param[in] _address Local address or "kMulticast". If you specify your
    /// local address, you will receive all the messages sent where the
    /// destination is <YOUR_LOCAL_ADDRESS, port> or <"kBroadcast", port>. On
    /// the other hand, if you specify "kMulticast" as the _address parameter,
    /// you will be subscribed to the multicast group <"kMulticast, port>".
    /// You will receive all the messages sent from any node to this multicast
    /// group.
    /// \param[in] _cb Callback function to be executed when a new message is
    /// received associated to the specified <_address, port>.
    /// In the callback, "_srcAddress" contains the address of the sender of
    /// the message. "_data" will contain the payload.
    /// \param[in] _obj Instance containing the member function callback.
    /// \param[in] _port Port used to receive messages.
    /// \return True when success or false otherwise.
    ///
    /// * Example usage (bind on the local address and default port):
    ///    this->Bind(&MyClass::OnDataReceived, this, this->Host());
    /// * Example usage (Bind on the multicast group and custom port.):
    ///    this->Bind(&MyClass::OnDataReceived, this, this->kMulticast, 5123);
    protected: template<typename C>
    bool Bind(void(C::*_cb)(const std::string &_srcAddress,
                            const std::string &_data),
              C *_obj,
              const std::string &_address,
              const int _port = kDefaultPort)
    {
      // Sanity check: Make sure that you use your local address or multicast.
      if ((_address != this->kMulticast) && (_address != this->Host()))
      {
        gzerr << "[" << this->Host() << "] Bind() error: Address ["
              << _address << "] is not your local address" << std::endl;
        return false;
      }

      // Mapping the "unicast socket" to a topic name.
      const std::string unicastTopic =
        "/swarm/" + _address + "/" + std::to_string(_port);

      if (!this->node.Subscribe(unicastTopic,
          &RobotPlugin::OnMsgReceived, this))
      {
        gzerr << "RobotPlugin::Bind() error: Subscribe() returned an "
              << "error while subscribing the unicast/multicast address"
              << std::endl;
        return false;
      }

      // Register the user callback using the topic name as the key.
      this->callbacks[unicastTopic] = std::bind(_cb, _obj,
          std::placeholders::_1, std::placeholders::_2);

      // Only enable broadcast if the address is a regular unicast address.
      if (_address != this->kMulticast)
      {
        const std::string bcastTopic =
          "/swarm/broadcast/" + std::to_string(_port);

        if (!this->node.Subscribe(bcastTopic,
            &RobotPlugin::OnMsgReceived, this))
        {
          gzerr << "RobotPlugin::Bind() error: Subscribe() returned an "
                << "error while subscribing the broadcast address"
                << std::endl;
          return false;
        }

        // Register the user callback using the broadcast topic as the key.
        this->callbacks[bcastTopic] = std::bind(_cb, _obj,
            std::placeholders::_1, std::placeholders::_2);
      }

      return true;
    }

    /// \brief Send some data to other/s member/s of the swarm.
    /// \param[in] _dstAddress Destination address. Note that the destination
    /// address might be a unicast address, "kBroadcast" or "kMulticast".
    /// In the case of broadcast and multicast communications your node
    /// will receive your own message if you're bind to your local or the
    /// multicast address.
    /// \param[in] _port Destination port.
    /// \param[in] _data Payload.
    /// \return True when success or false if the underlying library used for
    /// sending messages notifies an error (meaning that the message was not
    /// sent).
    protected: bool SendTo(const std::string &_data,
                           const std::string &_dstAddress,
                           const uint32_t _port = kDefaultPort);

    /// \brief Get your local address. This address should be specified as a
    /// SDF model parameter.
    /// \return The local address.
    protected: std::string Host() const;

    /// \brief Get the type of vehicle. The type of vehicle is set in the
    /// SDF world file using the <type> XML element.
    /// \return The enum value that specifies what type of vehicles this
    /// plugin controls.
    protected: VehicleType Type() const;

    /// \brief Set the robot's linear velocity.
    ///
    /// The velocity is applied in the robot's local coordinate frame, where
    ///
    /// * x = forward/back,
    /// * y = left/right,
    /// * z = up/down.
    ///
    /// This velocity will be constrained by the type of robot. For example,
    /// a ground vehicle will ignore the y & z components of the _velocity
    /// vector, but a rotorcraft will use all three.
    ///
    /// \param[in] _velocity The velocity vector in the robot's local
    /// coordinate frame (m/s).
    protected: void SetLinearVelocity(
                   const ignition::math::Vector3d &_velocity);

    /// \brief Set the robot's linear velocity.
    ///
    /// The velocity is applied in the robot's local coordinate frame, where
    ///
    /// * x = forward/back,
    /// * y = left/right,
    /// * z = up/down.
    ///
    /// This velocity will be constrained by the type of robot. For example,
    /// a ground vehicle will ignore the y & z components of the _velocity
    /// vector, but a rotorcraft will use all three.
    ///
    /// \param[in] _x X velocity in the robot's local coordinate frame (m/s).
    /// \param[in] _y Y velocity in the robot's local coordinate frame (m/s).
    /// \param[in] _z Z velocity in the robot's local coordinate frame (m/s).
    protected: void SetLinearVelocity(const double _x,
                   const double _y, const double _z);

    /// \brief Set the robot's anular velocity, using Euler angles.
    ///
    /// The velocity is applied in the robot's local coordinate frame, where
    ///
    /// * x = rotate about x-axis (roll),
    /// * y = rotate about y-axis (pitch),
    /// * z = rotate about z-axis (yaw).
    ///
    /// This velocity will be constrained by the type of robot. For example,
    /// a ground vehicle will ignore the x and y components of the _velocity
    ///
    /// vector, but a quadcopter will use all three.
    /// \param[in] _velocity Velocity about the robot's local XYZ axes
    /// (radian/s).
    protected: void SetAngularVelocity(
                   const ignition::math::Vector3d &_velocity);

    /// \brief Set the robot's anular velocity, using Euler angles.
    ///
    /// The velocity is applied in the robot's local coordinate frame, where
    ///
    /// * x = rotate about x-axis (roll),
    /// * y = rotate about y-axis (pitch),
    /// * z = rotate about z-axis (yaw).
    ///
    /// This velocity will be constrained by the type of robot. For example,
    /// a ground vehicle will ignore the x and y components of the _velocity
    /// vector, but a quadcopter will use all three.
    ///
    /// \param[in] _x Velocity about the robot's local X axis (radian/s).
    /// \param[in] _y Velocity about the robot's local Y axis (radian/s).
    /// \param[in] _z Velocity about the robot's local Z axis (radian/s).
    protected: void SetAngularVelocity(const double _x, const double _y,
                   const double _z);

    /// \brief Update the plugin.
    /// \param[in] _info Update information provided by the server.
    private: virtual void Update(const gazebo::common::UpdateInfo &_info);

    // Documentation Inherited.
    private: virtual void Load(gazebo::physics::ModelPtr _model,
                               sdf::ElementPtr _sdf);

    /// \brief Callback executed each time that a new message is received.
    /// The messages are originally sent from an agent, and received by the
    /// broker. The broker will process and forward the message, that will be
    /// received here. Inside this method we will execute the appropritate
    /// user's callback.
    /// \param[in] _topic Topic name associated to the new message received.
    /// \param[in] _msg New message received.
    private: void OnMsgReceived(const std::string &_topic,
                                const msgs::Datagram &_msg);

    /// \def Callback_t
    /// \brief The callback specified by the user when new data is available.
    /// This callback contains two parameters: the source address of the agent
    /// sending the message and the payload of the message.
    using Callback_t =
    std::function<void(const std::string &_srcAddress,
                       const std::string &_data)>;

    /// \brief Address used to send a message to all the members of the swarm
    /// listening on a specific port.
    protected: const std::string kBroadcast = "broadcast";

    /// \brief Address used to bind to a multicast group. Note that we do not
    /// support multiple multicast groups, only one.
    protected: const std::string kMulticast = "multicast";

    /// \brief Default port.
    protected: static const uint32_t kDefaultPort = 4100;

    /// \brief The transport node.
    private: ignition::transport::Node node;

    /// \brief User callbacks. The key is the topic name
    /// (e.g.: "/swarm/192.168.2.1/4000") and the value is the user callback.
    private: std::map<std::string, Callback_t> callbacks;

    /// \brief Pointer to the model;
    private: gazebo::physics::ModelPtr model;

    /// \brief Local address.
    private: std::string address;

    /// \brief Pointer to the update event connection.
    private: gazebo::event::ConnectionPtr updateConnection;

    /// \brief Type of vehicle.
    private: VehicleType type;
  };
}
#endif
