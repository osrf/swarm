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
#include <ignition/math/Vector2.hh>
#include <sdf/sdf.hh>

#include "msgs/datagram.pb.h"
#include "msgs/log_entry.pb.h"
#include "swarm/Broker.hh"
#include "swarm/Logger.hh"

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
  ///     - Dock() Dock a rotor vehicle to a ground vehicle.
  ///     - IsDocked() Return true if the rotor vehicle is docked.
  ///     - Launch() Launch a rotor vehicle from a ground vehicle.
  ///
  ///  * Sensors and world information.
  ///     - Pose() Get the robot's current pose from its GPS sensor.
  ///     - SearchArea() Get the search area, in GPS coordinates.
  ///     - MapQuery() Query the map for height and terrain type info.
  ///     - Image() Get the list of detected objects, and other related
  ///       information, from the camera sensor.
  ///     - CameraToWorld() Convert a pose in a robot's camera frame
  ///       into the world frame.
  ///     - Imu() Get the robot's linear and angular velocities and position
  ///       relative to a reference position (starting pose).
  ///     - Bearing() Get the angle between the true North and the robot.
  ///     - SetCameraOrientation() Set the pan(yaw)/tilt(pitch) of
  ///       the camera.
  ///     - CameraOrientation() Get the pan(yaw)/tilt(pitch) of the camera.
  ///     - Terrain() Get the terrain type at this vehicle's location.
  ///     - LostPersonDir() Get the direction from the BOO to the lost person.
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
  class IGNITION_VISIBLE RobotPlugin
    : public gazebo::ModelPlugin, public swarm::Loggable,
      public swarm::BrokerClient
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

    /// \brief The types of terrain.
    public: enum TerrainType
            {
              /// \brief Open terrain
              PLAIN     = 0,

              /// \brief Terrain with forest
              FOREST    = 1,

              /// \brief Terrain with a building
              BUILDING  = 2
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

    /// \brief Handle simulation reset.
    protected: virtual void Reset();

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
      const auto unicastEndPoint = _address + ":" + std::to_string(_port);

      if (!this->broker->Bind(this->Host(), this, unicastEndPoint))
        return false;

      // Register the user callback using the topic name as the key.
      this->callbacks[unicastEndPoint] = std::bind(_cb, _obj,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4);

      // Only enable broadcast if the address is a regular unicast address.
      if (_address != this->kMulticast)
      {
        const std::string bcastEndPoint = "broadcast:" + std::to_string(_port);

        if (!this->broker->Bind(this->Host(), this, bcastEndPoint))
          return false;

        // Register the user callback using the broadcast topic as the key.
        this->callbacks[bcastEndPoint] = std::bind(_cb, _obj,
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

    /// \brief Set the robot's target linear velocity.
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

    /// \brief Set the robot's target linear velocity.
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

    /// \brief Set the robot's target angular velocity, using Euler angles.
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

    /// \brief Set the robot's target angular velocity, using Euler angles.
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

    /// \brief Query the map to get the height and terrain type
    /// at a specific latitude and longitude.
    ///
    /// \param[in] _lat Latitude of the query (degrees).
    /// \param[in] _lon Longitude of the query (degrees).
    /// \param[out] _elev Elevation at the query point (meters).
    /// \param[out] _type Type of terrain at the query point.
    /// \return True if the latitude and longitude specify a valid point.
    /// False otherwise.
    protected: bool MapQuery(const double _lat, const double _lon,
                             double &_height, TerrainType &_type);

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

    /// \brief Set the pitch and yaw of the camera.
    /// \param[in] _pitch The pitch of the camera in radians.
    /// Valid values must fall between (+/-)PI/2 radian.
    /// \param[in] _yaw The yaw of the camera in radians.
    /// The camera can rotate 2PI radians.
    protected: void SetCameraOrientation(const double _pitch,
                                         const double _yaw);

    /// \brief Get the camera's orientation (pitch and yaw).
    /// \param[out] _pitch Camera's current pitch in radians.
    /// \param[out] _yaw Camera's current yaw in radians.
    protected: void CameraOrientation(double &_pitch,
                                      double &_yaw) const;

    /// \brief Launch a rotor vehicle from a ground vehicle. When docked, a
    /// rotor vehicle cannot move, but can process sensor data. When
    /// launched, a rotor vehicle can move independently from the ground
    /// vehicle.
    /// This has no affect for ground and fixed wing vehicles.
    /// \sa Dock
    /// \sa IsDocked
    protected: void Launch();

    /// \brief Dock a rotor vehicle. When docked, a
    /// rotor vehicle cannot move independently, but can process sensor data.
    /// When launched, a rotor vehicle can move independently from the
    /// ground vehicle.
    /// This has no affect for ground and fixed wing vehicles.
    /// \param[in] _vehicle Name of the vehicle to dock with.
    /// \return True if docking was successful. Docking can fail if
    /// the vehicle is too far away, see this->rotorDockingDistance,
    /// the vehicle was not found, or the _vehicle is not a ground vehicle.
    /// \sa Launch
    /// \sa IsDocked
    protected: bool Dock(const std::string &_vehicle);

    /// \brief Get whether this vehicle is docked. This function only has
    /// meaning for rotor vehicles.
    /// \return True if the rotor vehicle is docked to a ground vehicle.
    /// False if the rotor vehicle is free to move independently.
    /// \sa Dock
    /// \sa Launch
    protected: bool IsDocked() const;

    /// \brief Get the terrain type at this vehicle's location.
    /// \return Type of terrain at this vehicle's location.
    protected: TerrainType Terrain() const;

    /// \brief Helper function to get a terrain type at a position in
    /// Gazebo's world coordinate frame.
    /// \param[in] _pos Position to query.
    /// \return Type of terrain at the location.
    private: TerrainType TerrainAtPos(const ignition::math::Vector3d &_pos);

    /// \brief Get the direction from the BOO to the lost person at the
    /// start of simulation. Each component (x, y) of the result is
    /// rounded to fall on one of: -1.0, -0.5, 0, 0.5, 1.0.
    ///
    /// \return 2D direction vector from base of operation to the lost
    /// person. The direction is computed once at simulation start. If
    /// the simulation environment has no BOO or lost_person, a value of
    /// 0,0 is returned.
    protected: ignition::math::Vector2d LostPersonDir() const;

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
    /// \param[in] _msg New message received.
    private: virtual void OnMsgReceived(const msgs::Datagram &_msg) const;

    /// \brief Callback executed each time that a neighbor update is received.
    /// The messages are coming from the broker. The broker decides which are
    /// the robots inside the communication range of each other vehicle and
    /// notifies these updates.
    ///
    /// \param[in] _msg New message received containing the list of neighbors.
    private: void OnNeighborsReceived(
      const std::vector<std::string> &_neighbors);

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

    /// \brief Update the linear velocity of the robot model in Gazebo.
    private: void UpdateLinearVelocity();

    /// \brief Update the angular velocity of the robot model in Gazebo.
    private: void UpdateAngularVelocity();

    // Documentation inherited.
    private: virtual void OnLog(msgs::LogEntry &_logEntry) const;

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

    /// \brief Max linear velocity for ground vehicles (m/s).
    /// Equivalent to 25 mph
    protected: const double groundMaxLinearVel = 11.176;

    /// \brief Max linear velocity for rotor vehicles (m/s).
    /// Equivalent to 45 mph.
    protected: const double rotorMaxLinearVel = 20.117;

    /// \brief Max linear velocity for fixed wing vehicles (m/s).
    /// Equivalent to 90 mph.
    protected: double fixedMaxLinearVel = 40.234;

    /// \brief Max angular velocity for ground vehicles (radian/s).
    /// Equivalent to 60 degrees/second.
    protected: const double groundMaxAngularVel = 1.05;

    /// \brief Max linear velocity for rotor vehicles (radian/s).
    /// Equivalent to 120 degrees/second.
    protected: const double rotorMaxAngularVel = 2.1;

    /// \brief Max angular velocity for fixed wing vehicles (radian/s).
    /// Equivalent to (180 degrees/second).
    /// This is applied to the vehicle's pitch rate.
    /// Yaw and roll are computed based on the clamped linear velocity.
    protected: double fixedMaxAngularVel = 3.14;

    /// \brief Addresses of all the local neighbors.
    private: std::vector<std::string> neighbors;

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

    /// \brief Latitude observed by the robot's GPS.
    private: double observedLatitude;

    /// \brief Longitude observed by the robot's GPS.
    private: double observedLongitude;

    /// \brief Altitude observed by the robot's GPS.
    private: double observedAltitude;

    /// \brief Linear velocity observed in the robot's local coordinate frame.
    /// Units: m/s.
    private: ignition::math::Vector3d observedlinVel;

    /// \brief Angular velocity observed in the robot's local coordinate frame.
    /// Units: m/s.
    private: ignition::math::Vector3d observedAngVel;

    /// \brief Orientation observed with respect the reference pos.
    private: ignition::math::Quaterniond observedOrient;

    /// \brief Bearing between the true North and the robot.
    private: ignition::math::Angle observedBearing;

    /// \brief Logical image observed by the robot's camera.
    private: ImageData img;

    /// \brief Target linear velocity in the robot's local coordinate frame.
    /// Units: m/s.
    private: ignition::math::Vector3d targetLinVel;

    /// \brief Target angular velocity in the robot's local reference frame.
    /// Units: m/s.
    private: ignition::math::Vector3d targetAngVel;

    /// \brief Linear velocity in the robot's local coordinate frame (m/s).
    /// This version has no noise.
    private: ignition::math::Vector3d linearVelocityNoNoise;

    /// \brief Angular velocity in the robot's local coordinate frame (m/s).
    /// This version has no noise.
    private: ignition::math::Vector3d angularVelocityNoNoise;

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

    /// \brief Min random number used to computer a false negative
    private: double cameraFalseNegativeProbMin = 0.01;

    /// \brief Max random number used to compute a false negative
    private: double cameraFalseNegativeProbMax = 0.8;

    /// \brief Min random number used to compute a false positive
    private: double cameraFalsePositiveProbMin = 0.01;

    /// \brief Max random number used to compute a false positive
    private: double cameraFalsePositiveProbMax = 0.8;

    /// \brief Max position error in objects detected by the camera
    private: double cameraMaxPositionError = 5.0;

    /// \brief Array of all the models
    private: std::vector<std::string> modelNames;

    /// \brief Pointer to the shared broker.
    private: Broker *broker = Broker::Instance();

    /// \brief Pointer to the shared logger.
    private: Logger *logger = Logger::Instance();

    /// \brief Flag used by rotorcraft to determine if it's docked to
    /// a vehicle.
    private: bool rotorDocked = true;

    /// \brief Minimum distance for rotor to dock to a ground vehicle.
    private: double rotorDockingDistance = 2.0;

    /// \brief The ground vehicle that the rotorcraft is docked to.
    private: gazebo::physics::ModelPtr rotorDockVehicle;

    /// \brief Current terrain type for this vehicle.
    private: TerrainType terrainType = PLAIN;

    /// \brief Initial vector from boo to lost person. This acts as
    /// prior knowledge about where the lost person starts.
    private: ignition::math::Vector2d lostPersonInitDir;

    /// \brief The vehicle which a rotor is initially docked to.
    private: gazebo::physics::ModelPtr rotorStartingDockVehicle;

    /// \brief Camera start pitch
    private: double cameraStartPitch = 0.0;

    /// \brief Camera start yaw
    private: double cameraStartYaw = 0.0;

    /// \brief BooPlugin needs access to some of the private member variables.
    friend class BooPlugin;
  };
}
#endif
