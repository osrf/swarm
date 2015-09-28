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
#include <mutex>
#include <string>
#include <vector>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport.hh>
#include <sdf/sdf.hh>

#include "msgs/datagram.pb.h"
#include "msgs/neighbor_v.pb.h"

namespace swarm
{
  /// Typedef for object pose data.
  /// \sa ImageData
  typedef std::map<std::string, ignition::math::Pose3d> ObjPose_M;

  /// \brief A class that encapsulates image data.
  class ImageData
  {
    /// \brief Map of detected objects.
    public: ObjPose_M objects;
  };

  /// \brief A Model plugin that is the base class for all agent plugins
  /// in a swarm.
  /// This plugin exposes the following functionality to the derived plugins:
  ///
  /// * Configuration.
  ///     - Load()    This method will allow the agent to read SDF parameters
  ///                 from the model.
  ///
  /// * Communication.
  ///     - Bind()      This method binds an address to a virtual socket, and
  ///                   sends incoming messages to the specified callback.
  ///     - SendTo()    This method allows an agent to send data to other
  ///                   individual agent (unicast), all the agents (broadcast),
  ///                   or a group of agents (multicast).
  ///     - Host()      This method will return the agent's address.
  ///     - Neighbors() This method returns the addresses of other vehicles that
  ///                   are inside the communication range of this robot.
  ///
  ///  * Motion.
  ///     - Type()               This method returns the type of vehicle where
  ///                            this controller is running.
  ///     - SetLinearVelocity()  New linear velocity applied to the robot.
  ///     - SetAngularVelocity() New angular velocity applied to the robot.
  ///
  ///  * Sensors.
  ///     - Pose() Get the robot's current pose from its GPS sensor.
  ///     - SearchArea() Get the search area, in GPS coordinates.
  ///     - Image() Get the list of detected objects, and other related
  ///       information, from the camera sensor.
  ///     - CameraToWorld() Convert a pose in a robot's camera frame
  ///       into the world frame.
  ///     - Imu() Get the robot's linear and angular velocities and position
  ///       relative to a reference position (starting pose).
  ///     - Bearing() Get the angle between the true North and the robot.
  ///
  ///  * Battery
  ///     Each vehicle begins with a starting battery capacity. This
  ///     capacity lowers as time progresses. A vehicle may recharge its
  ///     battery by stopping (zero velocity) within 100 meters of the
  ///     base of operations (BOO). A battery recharges four times faster
  ///     than it discharges.
  ///
  ///     - BatteryStartCapacity() Starting battery capacity (mAh).
  ///     - BatteryCapacity() Current battery capacity (mAh).
  ///     - BatteryConsumption() Battery consumption (mA).
  ///     - BatteryConsumptionFactor() Factor applied to battery consumption
  ///                                  to account for additional loss.
  ///     - ExpectedBatteryLife() Battery life in seconds, based on the
  ///                             current capacity and consumption.
  ///
  ///  * Introspection.
  ///     - BooPose() Get the latitude and longitude of the BOO.
  ///     - Type() Get the vehicle type.
  ///     - Name() Get the name of the vehicle.
  ///     - SearchArea() Get the GPS coordinates of the search area.
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
    ///
    /// \param[in] _sdf Pointer to the SDF element of the model.
    protected: virtual void Load(sdf::ElementPtr _sdf);

    /// \brief Update the plugin. This function is called once every
    /// iteration.
    ///
    /// \param[in] _info Update information provided by the server.
    protected: virtual void Update(const gazebo::common::UpdateInfo &_info);

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
    ///
    /// \param[in] _cb Callback function to be executed when a new message is
    /// received associated to the specified <_address, port>.
    /// In the callback, "_srcAddress" contains the address of the sender of
    /// the message. "_dstAddress" contains the destination address. "_dstPort"
    /// contains the destination port. "_data" contains the payload.
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
                            const std::string &_dstAddress,
                            const uint32_t _dstPort,
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
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4);

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
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4);
      }

      return true;
    }

    /// \brief Send some data to other/s member/s of the swarm.
    /// \param[in] _dstAddress Destination address. Note that the destination
    /// address might be a unicast address, "kBroadcast" or "kMulticast".
    /// In the case of broadcast and multicast communications your node
    /// will receive your own message if you're bind to your local or the
    /// multicast address.
    ///
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
    ///
    /// \return The local address.
    protected: std::string Host() const;

    /// \brief Get the list of local neighbors.
    ///
    /// \return A vector of addresses from your local neighbors.
    protected: std::vector<std::string> Neighbors() const;

    /// \brief Get the type of vehicle. The type of vehicle is set in the
    /// SDF world file using the <type> XML element.
    /// \return The enum value that specifies what type of vehicles this
    /// plugin controls.
    protected: VehicleType Type() const;

    /// \brief Get the name of this robot.
    ///
    /// \return The name given to this robot in the SDF file.
    protected: std::string Name() const;

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
    /// \return True if the command was successful. False if the linear
    /// velocity could not be set, such as due to low battery.
    protected: bool SetLinearVelocity(
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
    /// \return True if the command was successful. False if the linear
    /// velocity could not be set, such as due to low battery.
    protected: bool SetLinearVelocity(const double _x,
                   const double _y, const double _z);

    /// \brief Set the robot's angular velocity, using Euler angles.
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
    /// \param[in] _velocity Velocity about the robot's local XYZ axes
    /// (radian/s).
    /// \return True if the command was successful. False if the angular
    /// velocity could not be set, such as due to low battery.
    protected: bool SetAngularVelocity(
                   const ignition::math::Vector3d &_velocity);

    /// \brief Set the robot's angular velocity, using Euler angles.
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
    /// \return True if the command was successful. False if the angular
    /// velocity could not be set, such as due to low battery.
    protected: bool SetAngularVelocity(const double _x, const double _y,
                   const double _z);

    /// \brief Get the robot's IMU information.
    ///
    /// The linear velocity is set in the robot's local coordinate frame, where
    ///
    /// * x = forward/back velocity,
    /// * y = left/right velociy,
    /// * z = up/down velocity.
    ///
    /// The angular velocity is set in the robot's local coordinate frame, where
    ///
    /// * x = Velocity about x-axis (roll),
    /// * y = Velocity about y-axis (pitch),
    /// * z = Velocity about z-axis (yaw).
    ///
    /// The orientation is set relative to the reference pose with a range from
    /// PI to -PI. The reference pose was initialized when the robot was spawned
    ///
    /// * x = Offset with respect the reference pos about x-axis (roll),
    /// * y = Offset with respect the reference pos about y-axis (pitch),
    /// * z = Offset with respect the reference pos about z-axis (yaw).
    ///
    /// \param[out] _linVel Linear velocity in the robot's local coordinate
    /// frame (m/s).
    /// \param[out] _angVel Angular velocity in the robot's local coordinate
    /// frame (m/s).
    /// \param[out] _orient Offset with respect the reference pos.
    protected: bool Imu(ignition::math::Vector3d &_linVel,
                        ignition::math::Vector3d &_angVel,
                        ignition::math::Quaterniond &_orient) const;

    /// \brief Angle between the true North and the robot. If the vehicle is
    /// facing North the bearing is 0. The bearing increments clockwise up to
    /// 2*PI radians.
    /// For example, a vehicle facing East would have a bearing of PI/2 radians.
    /// Note that in Gazebo the North is aligned with the +Y axis.
    ///
    /// \param[out] _bearing Bearing between the true North and the robot.
    /// \return True if the call was successful.
    protected: bool Bearing(ignition::math::Angle &_bearing) const;

    /// \brief Get the robot's current pose from its GPS sensor.
    ///
    /// \param[out] _latitude Robot latitude will be written here.
    /// \param[out] _longitude Robot longitude will be written here.
    /// \param[out] _altitude Robot altitude will be written here.
    /// \return True if the call was successful.
    protected: bool Pose(double &_latitude,
                         double &_longitude,
                         double &_altitude) const;

    /// \brief Get the base of operation's (BOO) position information. The
    /// BOO is always located on the ground.
    ///
    /// \param[out] _latitude BOO latitude
    /// \param[out] _longitude BOO longitude
    /// \return True if the function was successful. False may be returned
    /// if there is no BOO present.
    protected: bool BooPose(double &_latitude, double &_longitude) const;

    /// \brief Get the set of objects detected by the camera.
    ///
    /// \param[out] _img Image object that will hold the output from the
    /// camera. Note that each object's pose is in the robot's camera frame.
    /// Use CameraToWorld() for making a conversion to world coordinates.
    /// \return True if the call was successful.
    /// \sa CameraToWorld
    protected: bool Image(ImageData &_img) const;

    /// \brief Get the search area, in GPS coordinates.
    ///
    /// \param[out] _minLatitude Minimum latitude will be written here.
    /// \param[out] _maxLatitude Maximum latitude will be written here.
    /// \param[out] _minLongitude Minimum longitude will be written here.
    /// \param[out] _maxLongitude Maximum longitude will be written here.
    protected: void SearchArea(double &_minLatitude,
                               double &_maxLatitude,
                               double &_minLongitude,
                               double &_maxLongitude);

    /// \brief Get starting battery capacity (mAh).
    /// \return The battery's start capacity in mAh.
    protected: double BatteryStartCapacity() const;

    /// \brief Get the current battery capacity (mAh).
    /// \return The battery capacity in mAh.
    protected: double BatteryCapacity() const;

    /// \brief Get the vehicle's battery consumption (mA).
    /// \return The battery consumption in mA.
    protected: double BatteryConsumption() const;

    /// \brief Get the vehicle's battery consumption factor (unitless).
    /// \return Get the factor applied to battery consumption. This value
    /// will be between 0 and 1, where a value < 1 accounts for additional
    /// current  draw.
    protected: double BatteryConsumptionFactor() const;

    /// \brief Get the expected battery life in seconds.
    /// \return Battery life in seconds, based on the current capacity and
    /// consumption.
    protected: double ExpectedBatteryLife() const;

    /// \brief Convert a pose in a robot's camera frame into the world frame.
    /// \param[in] _poseinCamera The pose in the camera frame
    /// \return The pose in the world frame.
    protected: ignition::math::Pose3d CameraToWorld(
      const ignition::math::Pose3d &_poseinCamera) const;

    /// \brief Update the plugin.
    ///
    /// \param[in] _info Update information provided by the server.
    private: virtual void Loop(const gazebo::common::UpdateInfo &_info);

    // Documentation Inherited.
    private: virtual void Load(gazebo::physics::ModelPtr _model,
                               sdf::ElementPtr _sdf);

    /// \brief Callback executed each time that a new message is received.
    /// The messages are originally sent from an agent, and received by the
    /// broker. The broker will process and forward the message, that will be
    /// received here. Inside this method we will execute the appropritate
    /// user's callback.
    ///
    /// \param[in] _topic Topic name associated to the new message received.
    /// \param[in] _msg New message received.
    private: void OnMsgReceived(const std::string &_topic,
                                const msgs::Datagram &_msg);

    /// \brief Callback executed each time that a neighbor update is received.
    /// The messages are coming from the broker. The broker decides which are
    /// the robots inside the communication range of each other vehicle and
    /// notifies these updates.
    ///
    /// \param[in] _topic Topic name associated to the new message received.
    /// \param[in] _msg New message received containing the list of neighbors.
    private: void OnNeighborsReceived(const std::string &_topic,
                                      const msgs::Neighbor_V &_msg);

    /// \brief Adjust the pose of the vehicle to stay within the terrain
    /// boundaries.
    private: void AdjustPose();

    /// \brief Get terrain information at the specified location.
    /// \param[in] _pos Reference position.
    /// \param[out] _terrainPos The 3d point on the terrain.
    /// \param[out] _norm Normal to the terrain.
    private: void TerrainLookup(const ignition::math::Vector3d &_pos,
                                ignition::math::Vector3d &_terrainPos,
                                ignition::math::Vector3d &_norm) const;

    /// \brief Update and store sensor information.
    private: void UpdateSensors();

    /// \brief Update the battery capacity.
    private: void UpdateBattery();

    /// \def Callback_t
    /// \brief The callback specified by the user when new data is available.
    /// This callback contains two parameters: the source address of the agent
    /// sending the message and the payload of the message.
    using Callback_t =
    std::function<void(const std::string &_srcAddress,
                       const std::string &_dstAddress,
                       const uint32_t _dstPort,
                       const std::string &_data)>;

    /// \brief Address used to send a message to all the members of the swarm
    /// listening on a specific port.
    protected: const std::string kBroadcast = "broadcast";

    /// \brief Address used to bind to a multicast group. Note that we do not
    /// support multiple multicast groups, only one.
    protected: const std::string kMulticast = "multicast";

    /// \brief Address used by the base of operations.
    protected: const std::string kBoo       = "boo";

    /// \brief Default port.
    protected: static const uint32_t kDefaultPort = 4100;

    /// \brief Base of communications port.
    protected: static const uint32_t kBooPort     = 4200;

    /// \brief Maximum transmission payload size (octets) for each message.
    protected: static const uint32_t kMtu = 1500;

    /// \brief Addresses of all the local neighbors.
    private: std::vector<std::string> neighbors;

    /// \brief List of comms probabilities (parallel to neighbors).
    private: std::vector<double> neighborProbabilities;

    /// \brief The transport node.
    private: ignition::transport::Node node;

    // The gazebo transport node. Used for debugging, see source.
    // private: gazebo::transport::NodePtr gzNode;

    // Used to publish markers, Used for debugging, see source.
    // private: gazebo::transport::PublisherPtr markerPub;

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

    /// \brief Pointer to GPS sensor
    private: gazebo::sensors::GpsSensorPtr gps;

    /// \brief Pointer to IMU sensor
    private: gazebo::sensors::ImuSensorPtr imu;

    /// \brief Pointer to LogicalCamera sensor
    private: gazebo::sensors::LogicalCameraSensorPtr camera;

    /// \brief Min/max lat/long of search area.
    private: double searchMinLatitude, searchMaxLatitude,
                    searchMinLongitude, searchMaxLongitude;

    /// \brief Mutex to protect shared member variables.
    private: mutable std::mutex mutex;

    /// \brief Pointer to the terrain
    private: gazebo::physics::HeightmapShapePtr terrain;

    /// \brief This is the scaling from world coordinates to heightmap
    /// coordinates.
    private: ignition::math::Vector2d terrainScaling;

    /// \brief Size of the terrain
    private: ignition::math::Vector3d terrainSize;

    /// \brief Half the height of the model.
    private: double modelHeight2;

    /// \brief Linear velocity in the robot's local coordinate frame (m/s).
    private: ignition::math::Vector3d linearVelocity;

    /// \brief Linear velocity in the robot's local coordinate frame (m/s).
    /// This version has no noise.
    private: ignition::math::Vector3d linearVelocityNoNoise;

    /// \brief Angular velocity in the robot's local coordinate frame (m/s).
    private: ignition::math::Vector3d angularVelocity;

    /// \brief Angular velocity in the robot's local coordinate frame (m/s).
    /// This version has no noise.
    private: ignition::math::Vector3d angularVelocityNoNoise;

    /// \brief Offset with respect the reference pos.
    private: ignition::math::Quaterniond orientation;

    /// \brief Bearing between the true North and the robot.
    private: ignition::math::Angle bearing;

    /// \brief The capacity at start. This is used to handle reset.
    private: double startCapacity;

    /// \brief Milli-Amp-Hour battery capacity
    private: double capacity;

    /// \brief Milli-Amp vehicle consumption
    private: double consumption;

    /// \brief A vehicle must be less than this distance to recharge its
    ///        battery from the BOO.
    private: double booRechargeDistance = 100.0;

    /// \brief Unitless factor applied to the battery consumption. This can
    /// be use to account to extra current draw.
    private: double consumptionFactor;

    /// \brief Pointer to the world.
    private: gazebo::physics::WorldPtr world;

    /// \brief Pointer to the BOO.
    private: gazebo::physics::ModelPtr boo;

    /// \brief Max linear velocity for ground vehicles (m/s).
    /// Equivalent to 25 mph
    private: const double groundMaxLinearVel = 11.176;

    /// \brief Max linear velocity for rotor vehicles (m/s).
    /// Equivalent to 45 mph.
    private: const double rotorMaxLinearVel = 20.117;

    /// \brief Max linear velocity for fixed wing vehicles (m/s).
    /// Equivalent to 90 mph.
    private: double fixedMaxLinearVel = 40.234;

    /// \brief Max angular velocity for ground vehicles (radian/s).
    /// Equivalent to 60 degrees/second.
    private: const double groundMaxAngularVel = 1.05;

    /// \brief Max linear velocity for rotor vehicles (radian/s).
    /// Equivalent to 120 degrees/second.
    private: const double rotorMaxAngularVel = 2.1;

    /// \brief Max angular velocity for fixed wing vehicles (radian/s).
    /// Equivalent to (180 degrees/second).
    /// This is applied to the vehicle's pitch rate.
    /// Yaw and roll are computed based on the clamped linear velocity.
    private: double fixedMaxAngularVel = 3.14;

    /// \brief 5 percent of the time we get a false negative
    private: double logicalCameraFalseNegativeProb = 0.05;

    /// \brief A false positive is determined based on the distance from the
    /// vehicle to a detected object. The equation is:
    ///
    /// bool falsePositive = rand(0, 1) < dist /
    ///                          (camera_far * logicalCameraFalsePositiveFactor)
    ///
    /// * rand(0, 1) : Random number between 0 and 1 inclusive.
    /// * dist : Distance from this vehicle to the detected object.
    /// * camera_far : Camera's far clip plane.
    /// * logicalCameraFalsePositiveFactor : This variable.
    private: double logicalCameraFalsePositiveFactor = 40.0;

    /// \brief BooPlugin needs access to some of the private member variables.
    friend class BooPlugin;
  };
}
#endif
