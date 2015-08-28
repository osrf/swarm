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

#include <mutex>
#include <string>
#include <vector>
#include <ignition/math/Helpers.hh>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Vector2i.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include "msgs/datagram.pb.h"
#include "msgs/neighbor_v.pb.h"
#include "swarm/RobotPlugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)

//////////////////////////////////////////////////
RobotPlugin::RobotPlugin()
  : type(GROUND),
    searchMinLatitude(0),
    searchMaxLatitude(0),
    searchMinLongitude(0),
    searchMaxLongitude(0),
    modelHeight2(0)
{
}

//////////////////////////////////////////////////
RobotPlugin::~RobotPlugin()
{
  gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

//////////////////////////////////////////////////
void RobotPlugin::Load(sdf::ElementPtr /*_sdf*/)
{
}

//////////////////////////////////////////////////
bool RobotPlugin::SendTo(const std::string &_data,
    const std::string &_dstAddress, const uint32_t _port)
{
  msgs::Datagram msg;
  msg.set_src_address(this->Host());
  msg.set_dst_address(_dstAddress);
  msg.set_dst_port(_port);
  msg.set_data(_data);

  // The neighbors list will be included in the broker.

  // Send the message from the agent to the broker.
  const std::string kBrokerIncomingTopic = "/swarm/broker/incoming";
  if (!this->node.Publish(kBrokerIncomingTopic, msg))
  {
    gzerr << "[" << this->Host() << "] RobotPlugin::SendTo(): Error "
          << "trying to publish on topic [" << kBrokerIncomingTopic << "]"
          << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void RobotPlugin::SetLinearVelocity(const ignition::math::Vector3d &_velocity)
{
  ignition::math::Pose3d myPose = this->model->GetWorldPose().Ign();

  switch (this->type)
  {
    default:
    case RobotPlugin::GROUND:
      {
        // Get linear velocity in world frame
        ignition::math::Vector3d linearVel = myPose.Rot().RotateVector(
            _velocity * ignition::math::Vector3d::UnitX);
        this->model->SetLinearVel(linearVel);
        break;
      }
    case RobotPlugin::ROTOR:
      {
        // Get linear velocity in world frame
        ignition::math::Vector3d linearVel = myPose.Rot().RotateVector(
            _velocity);
        this->model->SetLinearVel(linearVel);
        break;
      }
    case RobotPlugin::FIXED_WING:
      {
        // Get linear velocity in world frame
        ignition::math::Vector3d linearVel = myPose.Rot().RotateVector(
            _velocity * ignition::math::Vector3d::UnitX);
        this->model->SetLinearVel(linearVel);
        break;
      }
  };
}

//////////////////////////////////////////////////
void RobotPlugin::SetLinearVelocity(const double _x, const double _y,
    const double _z)
{
  this->SetLinearVelocity(ignition::math::Vector3d(_x, _y, _z));
}

//////////////////////////////////////////////////
void RobotPlugin::SetAngularVelocity(const ignition::math::Vector3d &_velocity)
{
  switch (this->type)
  {
    default:
    case RobotPlugin::GROUND:
      {
        this->model->SetAngularVel(
            _velocity * ignition::math::Vector3d::UnitZ);;
        break;
      }
    case RobotPlugin::ROTOR:
      {
        this->model->SetAngularVel(_velocity);
        break;
      }
    case RobotPlugin::FIXED_WING:
      {
        double yawRate = 0.0;
        double rollRate = 0.0;

        // Current orientation as Euler angles
        ignition::math::Vector3d rpy = this->orientation.Euler();

        // Make sure we don't divide by zero. The vehicle should also
        // be moving before it can bank.
        if (!ignition::math::equal(this->linearVelocity.X(), 0.0))
        {
          yawRate = (-9.81 * tan(rpy.X())) / this->linearVelocity.X();
          yawRate = ignition::math::clamp(yawRate, -IGN_DTOR(10), IGN_DTOR(10));
          rollRate = ignition::math::clamp(_velocity[0],
              -IGN_DTOR(5), IGN_DTOR(5));
        }

        this->model->SetAngularVel(
            ignition::math::Vector3d(rollRate, _velocity[1], yawRate));
        break;
      }
  };
}

//////////////////////////////////////////////////
void RobotPlugin::SetAngularVelocity(const double _x, const double _y,
    const double _z)
{
  this->SetAngularVelocity(ignition::math::Vector3d(_x, _y, _z));
}

//////////////////////////////////////////////////
void RobotPlugin::UpdateSensors()
{
  if (this->imu)
  {
    // \TODO: Consider adding noise (or just let Gazebo do it?).
    this->linearVelocity = this->model->GetRelativeLinearVel().Ign();
    this->angularVelocity = this->imu->AngularVelocity();
    this->orientation = this->imu->Orientation();
  }

  // TODO: Consider adding noise (or just let Gazebo do it?).
  // Get the Yaw angle of the model in Gazebo world coordinates.
  this->bearing = ignition::math::Angle(
      this->model->GetWorldPose().rot.GetAsEuler().z);

  // A "0" bearing value means that the model is facing North.
  // North is aligned with the Gazebo Y axis, so we should add an offset of
  // PI/2 to the bearing in the Gazebo world coordinates.
  this->bearing = ignition::math::Angle::HalfPi - this->bearing;

  // Normalize: Gazebo orientation uses PI,-PI but compasses seem
  // to use 0,2*PI.
  if (this->bearing.Radian() < 0)
    this->bearing = ignition::math::Angle::TwoPi + this->bearing;
}

//////////////////////////////////////////////////
bool RobotPlugin::Imu(ignition::math::Vector3d &_linVel,
  ignition::math::Vector3d &_angVel, ignition::math::Quaterniond &_orient) const
{
  // TODO: Consider adding noise (or just let Gazebo do it?).
  _linVel = this->linearVelocity;
  _angVel = this->angularVelocity;
  _orient = this->orientation;
  return true;
}

//////////////////////////////////////////////////
bool RobotPlugin::Bearing(ignition::math::Angle &_bearing) const
{
  _bearing = this->bearing;
  return true;
}

//////////////////////////////////////////////////
bool RobotPlugin::Pose(double &_latitude,
                       double &_longitude,
                       double &_altitude) const
{
  if (!this->gps)
  {
    gzerr << "No GPS sensor available" << std::endl;
    _latitude = _longitude = _altitude = 0.0;
    return false;
  }

  // TODO: Consider adding noise (or just let Gazebo do it?).
  _latitude = this->gps->Latitude().Degree();
  _longitude = this->gps->Longitude().Degree();
  _altitude = this->gps->GetAltitude();

  return true;
}

//////////////////////////////////////////////////
bool RobotPlugin::Image(ImageData &_img) const
{
  if (!this->camera)
  {
    gzerr << "No logical_camera sensor available" << std::endl;
    return false;
  }

  _img.objects.clear();

  gazebo::msgs::LogicalCameraImage img = this->camera->Image();
  for (auto const imgModel : img.model())
  {
    // Skip ground plane model
    if (imgModel.name() != "ground_plane")
      _img.objects[imgModel.name()] = gazebo::msgs::ConvertIgn(imgModel.pose());
  }

  return true;
}

//////////////////////////////////////////////////
void RobotPlugin::SearchArea(double &_minLatitude,
                             double &_maxLatitude,
                             double &_minLongitude,
                             double &_maxLongitude)
{
  _minLatitude = this->searchMinLatitude;
  _maxLatitude = this->searchMaxLatitude;
  _minLongitude = this->searchMinLongitude;
  _maxLongitude = this->searchMaxLongitude;
}

//////////////////////////////////////////////////
std::string RobotPlugin::Host() const
{
  return this->address;
}

//////////////////////////////////////////////////
std::vector<std::string> RobotPlugin::Neighbors() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->neighbors;
}

//////////////////////////////////////////////////
void RobotPlugin::Update(const gazebo::common::UpdateInfo & /*_info*/)
{
}

//////////////////////////////////////////////////
void RobotPlugin::Loop(const gazebo::common::UpdateInfo &_info)
{
  this->UpdateSensors();
  this->Update(_info);
  this->AdjustPose();
}

//////////////////////////////////////////////////
void RobotPlugin::AdjustPose()
{
  if (!this->terrain || !this->model)
    return;

  // Get the pose of the vehicle
  ignition::math::Pose3d pose = this->model->GetWorldPose().Ign();

  // Constrain X position to the terrain boundaries
  pose.Pos().X(ignition::math::clamp(pose.Pos().X(),
        -this->terrainSize.X() * 0.5, this->terrainSize.X() * 0.5));

  // Constrain Y position to the terrain boundaries
  pose.Pos().Y(ignition::math::clamp(pose.Pos().Y(),
        -this->terrainSize.Y() * 0.5, this->terrainSize.Y() * 0.5));

  ignition::math::Vector3d norm;
  ignition::math::Vector3d terrainPos;
  this->TerrainLookup(pose.Pos(), terrainPos, norm);

  // Constrain each type of robot
  switch (this->Type())
  {
    default:
    case GROUND:
      {
        ignition::math::Vector3d euler = pose.Rot().Euler();

        // Project normal onto xy plane
        ignition::math::Vector3d norm2d(norm.X(), norm.Y(), 0);
        norm2d.Normalize();

        // Pitch vector
        ignition::math::Vector3d normPitchDir(
            cos(euler.Z()), sin(euler.Z()), 0);

        // Roll vector
        ignition::math::Vector3d normRollDir(sin(euler.Z()),
            -cos(euler.Z()), 0);

        // Compute pitch and roll
        double pitch = norm2d.Dot(normPitchDir) * acos(norm.Z());
        double roll = norm2d.Dot(normRollDir) * acos(norm.Z());

        // Add half the height of the vehicle
        pose.Pos().Z(terrainPos.Z() + this->modelHeight2);
        pose.Rot().Euler(roll, pitch, pose.Rot().Euler().Z());

        // Set the pose.
        this->model->SetRelativePose(pose);
        break;
      }
    case ROTOR:
      {
        if (pose.Pos().Z() < terrainPos.Z() + this->modelHeight2)
        {
          pose.Pos().Z(terrainPos.Z() + this->modelHeight2);

          // Set the pose.
          this->model->SetWorldPose(pose);
        }
        break;
      }
    case FIXED_WING:
      {
        if (pose.Pos().Z() < terrainPos.Z() + this->modelHeight2)
        {
          pose.Pos().Z(terrainPos.Z() + this->modelHeight2);

          // Set the pose.
          this->model->SetWorldPose(pose);
        }
        break;
      }
  };
}

//////////////////////////////////////////////////
void RobotPlugin::Load(gazebo::physics::ModelPtr _model,
                       sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "RobotPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "RobotPlugin _sdf pointer is NULL");
  this->model = _model;
  if (!this->model)
  {
    gzerr << "Invalid model pointer. Plugin will not load.\n";
    return;
  }
  this->modelHeight2 = this->model->GetBoundingBox().GetZLength()*0.5;

  // Get the terrain, if it's present
  gazebo::physics::ModelPtr terrainModel =
    this->model->GetWorld()->GetModel("terrain");

  // Load some info about the terrain.
  if (terrainModel)
  {
    this->terrain =
      boost::dynamic_pointer_cast<gazebo::physics::HeightmapShape>(
          terrainModel->GetLink()->GetCollision("collision")->GetShape());

    // Get the size of the terrain
    this->terrainSize = this->terrain->GetSize().Ign();

    // Set the terrain scaling.
    this->terrainScaling.Set(this->terrain->GetSize().x /
        (this->terrain->GetVertexCount().x-1),
        this->terrain->GetSize().y /
        (this->terrain->GetVertexCount().y-1));
  }


  // Load the vehicle type
  if (_sdf->HasElement("type"))
  {
    std::string vehicleType = _sdf->Get<std::string>("type");
    if (vehicleType == "ground")
      this->type = GROUND;
    else if (vehicleType == "rotor")
      this->type = ROTOR;
    else if (vehicleType == "fixed_wing")
      this->type = FIXED_WING;
    else
      gzerr << "Unknown vehicle type[" << vehicleType <<"], using ground.\n";
  }
  else
  {
    gzerr << "No vehicle type specified, using ground.\n";
  }

  // Collide with nothing
  for (auto &link : this->model->GetLinks())
    link->SetCollideMode("none");

  // Read the robot address.
  if (!_sdf->HasElement("address"))
  {
    gzerr << "RobotPlugin::Load(): Unable to find the <address> parameter\n";
    return;
  }

  this->address = _sdf->Get<std::string>("address");

  // Get the camera sensor
  if (_sdf->HasElement("camera"))
  {
    this->camera =
      boost::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(
        gazebo::sensors::get_sensor(this->model->GetScopedName(true) + "::" +
          _sdf->Get<std::string>("camera")));

    if (!this->camera)
    {
      gzerr << "Trying to get a logical_camera for robot with address["
        << this->address << "], but the specified camera[" <<
        _sdf->Get<std::string>("camera") << "] has an incorrect type.\n";
    }
  }

  if (!this->camera)
  {
    gzwarn << "No camera sensor found on robot with address "
      << this->address << std::endl;
  }

  // Get the gps sensor
  if (_sdf->HasElement("gps"))
  {
    this->gps =
      boost::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(
        gazebo::sensors::get_sensor(this->model->GetScopedName(true) + "::" +
          _sdf->Get<std::string>("gps")));
  }

  if (!this->gps)
  {
    gzwarn << "No gps sensor found on robot with address "
      << this->address << std::endl;
  }

  // Get the IMU sensor
  if (_sdf->HasElement("imu"))
  {
    this->imu =
      boost::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(
        gazebo::sensors::get_sensor(this->model->GetScopedName(true) + "::" +
          _sdf->Get<std::string>("imu")));
  }

  if (!this->imu)
  {
    gzwarn << "No IMU sensor found on robot with address "
      << this->address << std::endl;
  }

  // Get the search area size, which is a child of the plugin
  this->searchMinLatitude = 0.0;
  this->searchMaxLatitude = 0.0;
  this->searchMinLongitude = 0.0;
  this->searchMaxLongitude = 0.0;
  bool foundSwarmSearchArea = false;
  sdf::ElementPtr searchAreaSDF = _sdf->GetElement("swarm_search_area");
  while (searchAreaSDF)
  {
    if (searchAreaSDF->HasElement("min_relative_latitude_deg") &&
        searchAreaSDF->HasElement("max_relative_latitude_deg") &&
        searchAreaSDF->HasElement("min_relative_longitude_deg") &&
        searchAreaSDF->HasElement("max_relative_longitude_deg"))
    {
      this->searchMinLatitude =
        searchAreaSDF->GetElement("min_relative_latitude_deg")->Get<double>();
      this->searchMaxLatitude =
        searchAreaSDF->GetElement("max_relative_latitude_deg")->Get<double>();
      this->searchMinLongitude =
        searchAreaSDF->GetElement("min_relative_longitude_deg")->Get<double>();
      this->searchMaxLongitude =
        searchAreaSDF->GetElement("max_relative_longitude_deg")->Get<double>();
      foundSwarmSearchArea = true;
      break;
    }
    searchAreaSDF = searchAreaSDF->GetNextElement("swarm_search_area");
  }

  sdf::ElementPtr modelSDF = _sdf->GetParent();

  // We have the search area size.  Now get the origin, which is in
  // spherical_coordinates, a child of the world.
  sdf::ElementPtr worldSDF = modelSDF->GetParent();
  sdf::ElementPtr sphericalCoordsSDF =
    worldSDF->GetElement("spherical_coordinates");
  bool foundSphericalCoords = false;
  while (sphericalCoordsSDF)
  {
    if (sphericalCoordsSDF->HasElement("latitude_deg") &&
        sphericalCoordsSDF->HasElement("longitude_deg"))
    {
      // Offset the search borders by the origin.
      this->searchMinLatitude +=
        sphericalCoordsSDF->GetElement("latitude_deg")->Get<double>();
      this->searchMaxLatitude +=
        sphericalCoordsSDF->GetElement("latitude_deg")->Get<double>();
      this->searchMinLongitude +=
        sphericalCoordsSDF->GetElement("longitude_deg")->Get<double>();
      this->searchMaxLongitude +=
        sphericalCoordsSDF->GetElement("longitude_deg")->Get<double>();
      foundSphericalCoords = true;
      break;
    }
    sphericalCoordsSDF =
      sphericalCoordsSDF->GetNextElement("spherical_coordinates");
  }

  if (!foundSwarmSearchArea || !foundSphericalCoords)
  {
    gzwarn << "No spherical_coordinates and/or swarm_search_area tags found. "
              "Search area will be undefined." << std::endl;
  }

  const std::string kBrokerIncomingTopic = "/swarm/broker/incoming";
  if (!this->node.Advertise(kBrokerIncomingTopic))
  {
    gzerr << "[" << this->Host() << "] RobotPlugin::Load(): Error "
          << "trying to advertise topic [" << kBrokerIncomingTopic << "]\n";
  }

  // Subscribe to the topic for receiving neighbor updates.
  const std::string kNeighborUpdatesTopic =
      "/swarm/" + this->Host() + "/neighbors";
  this->node.Subscribe(
      kNeighborUpdatesTopic, &RobotPlugin::OnNeighborsReceived, this);

  this->AdjustPose();

  // Call the Load() method from the derived plugin.
  this->Load(_sdf);

  // This is debug code that can be used to render markers in Gazebo. It
  // requires a version of gazebo with visual markers.
  // this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  // this->gzNode->Init();
  // this->markerPub =
  // this->gzNode->Advertise<gazebo::msgs::Marker>("~/marker");
  // END DEBUG CODE

  // Listen to the update event broadcasted every simulation iteration.
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotPlugin::Loop, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void RobotPlugin::OnMsgReceived(const std::string &/*_topic*/,
    const msgs::Datagram &_msg)
{
  const std::string topic = "/swarm/" + _msg.dst_address() + "/" +
      std::to_string(_msg.dst_port());

  if (this->callbacks.find(topic) == this->callbacks.end())
  {
    gzerr << "[" << this->Host() << "] RobotPlugin::OnMsgReceived(): "
          << "Address [" << topic << "] not found" << std::endl;
    return;
  }

  // Ignore if the address of this robot was not a neighbor of the sender.
  bool visible = false;
  for (auto i = 0; i < _msg.recipients().size(); ++i)
  {
    if (_msg.recipients(i) == this->Host())
    {
      visible = true;
      break;
    }
  }

  if (visible)
  {
    // Run the user callback.
    auto const &userCallback = this->callbacks[topic];
    userCallback(_msg.src_address(), _msg.data());
  }
}

//////////////////////////////////////////////////
void RobotPlugin::OnNeighborsReceived(const std::string &/*_topic*/,
    const msgs::Neighbor_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->neighbors.clear();
  this->neighborProbabilities.clear();
  for (auto i = 0; i < _msg.neighbors().size(); ++i)
  {
    if (_msg.neighbors(i) != this->Host())
      this->neighbors.push_back(_msg.neighbors(i));
  }
}

//////////////////////////////////////////////////
RobotPlugin::VehicleType RobotPlugin::Type() const
{
  return this->type;
}

//////////////////////////////////////////////////
std::string RobotPlugin::Name() const
{
  return this->model->GetName();
}

//////////////////////////////////////////////////
void RobotPlugin::TerrainLookup(const ignition::math::Vector3d &_pos,
    ignition::math::Vector3d &_terrainPos,
    ignition::math::Vector3d &_norm) const
{
  // The robot position in the coordinate frame of the terrain
  ignition::math::Vector3d robotPos(
      (this->terrainSize.X() * 0.5 + _pos.X()) / this->terrainScaling.X(),
      (this->terrainSize.Y() * 0.5 - _pos.Y()) / this->terrainScaling.Y(), 0);

  // Three vertices that define the triangle on which the vehicle rests
  // The first vertex is closest point on the terrain
  ignition::math::Vector3d v1(std::round(robotPos.X()),
      std::round(robotPos.Y()), 0);
  ignition::math::Vector3d v2 = v1;
  ignition::math::Vector3d v3 = v1;

  // The second and third vertices are chosen based on how OGRE layouts
  // the triangle strip.
  if (static_cast<int>(v1.X()) == static_cast<int>(std::ceil(robotPos.X())) &&
      static_cast<int>(v1.Y()) == static_cast<int>(std::ceil(robotPos.Y())))
  {
    if (static_cast<int>(v1.Y()) % 2 == 0)
    {
      v2.Y(v1.Y()-1);
      v3.X(v1.X()-1);
    }
    else
    {
      ignition::math::Vector3d b(v1.X()-1, v1.Y(), 0);
      ignition::math::Vector3d c(v1.X(), v1.Y()-1, 0);
      if (robotPos.Distance(b) < robotPos.Distance(c))
      {
        v3 = b;
        v2.X(v1.X()-1);
        v2.Y(v1.Y()-1);
      }
      else
      {
        v2 = c;
        v3.X(v1.X()-1);
        v3.Y(v1.Y()-1);
      }
    }
  }
  else if (static_cast<int>(v1.X()) ==
      static_cast<int>(std::floor(robotPos.X())) &&
      static_cast<int>(v1.Y()) == static_cast<int>(std::ceil(robotPos.Y())))
  {
    if (static_cast<int>(v1.Y()) % 2 == 0)
    {
      ignition::math::Vector3d b(v1.X()+1, v1.Y(), 0);
      ignition::math::Vector3d c(v1.X(), v1.Y()-1, 0);
      if (robotPos.Distance(b) < robotPos.Distance(c))
      {
        v2 = b;
        v3.X(v1.X()+1);
        v3.Y(v1.Y()-1);
      }
      else
      {
        v3 = c;
        v2.X(v1.X()+1);
        v2.Y(v1.Y()-1);
      }
    }
    else
    {
      v2.X(v1.X()+1);
      v3.Y(v1.Y()-1);
    }
  }
  else if (static_cast<int>(v1.X()) ==
      static_cast<int>(std::floor(robotPos.X())) &&
      static_cast<int>(v1.Y()) == static_cast<int>(std::floor(robotPos.Y())))
  {
    if (static_cast<int>(v1.Y()) % 2 == 0)
    {
      ignition::math::Vector3d b(v1.X()+1, v1.Y(), 0);
      ignition::math::Vector3d c(v1.X(), v1.Y()+1, 0);
      if (robotPos.Distance(b) < robotPos.Distance(c))
      {
        v2.X(v1.X()+1);
        v2.Y(v1.Y()+1);
        v3 = b;
      }
      else
      {
        v2 = c;
        v3.X(v1.X()+1);
        v3.Y(v1.Y()+1);
      }
    }
    else
    {
      v2.Y(v1.Y()+1);
      v3.X(v1.X()+1);
    }
  }
  else
  {
    if (static_cast<int>(v1.Y()) % 2 == 0)
    {
      v2.X() -= 1;
      v3.Y() += 1;
    }
    else
    {
      ignition::math::Vector3d b(v1.X()-1, v1.Y(), 0);
      ignition::math::Vector3d c(v1.X(), v1.Y()+1, 0);
      if (robotPos.Distance(b) < robotPos.Distance(c))
      {
        v2 = b;
        v3.X(v1.X()-1);
        v3.Y(v1.Y()+1);
      }
      else
      {
        v2.X(v1.X()-1);
        v2.Y(v1.Y()+1);
        v3 = c;
      }
    }
  }

  // Get the height at each vertex
  v1.Z(this->terrain->GetHeight(v1.X(), v1.Y()));
  v2.Z(this->terrain->GetHeight(v2.X(), v2.Y()));
  v3.Z(this->terrain->GetHeight(v3.X(), v3.Y()));

  // Display a marker that highlights the vertices currently used to
  // compute the vehicles height. This is debug code that is very useful
  // but it requires a version of gazebo with visual markers.
  //
  // gazebo::msgs::Marker markerMsg;
  // markerMsg.set_layer("default");
  // markerMsg.set_id(0);
  // markerMsg.set_action(gazebo::msgs::Marker::MODIFY);
  // markerMsg.set_type(gazebo::msgs::Marker::LINE_STRIP);


  // v1a.Z() += 0.1;
  // v2a.Z() += 0.1;
  // v3a.Z() += 0.1;
  // gazebo::msgs::Set(markerMsg.add_point(), v1a);
  // gazebo::msgs::Set(markerMsg.add_point(), v2a);
  // gazebo::msgs::Set(markerMsg.add_point(), v3a);
  // if (this->markerPub)
  //   this->markerPub->Publish(markerMsg);
  // END DEBUG CODE

  ignition::math::Vector3d v1a = v1;
  ignition::math::Vector3d v2a = v2;
  ignition::math::Vector3d v3a = v3;
  v1a.X(v1a.X()*this->terrainScaling.X() - this->terrainSize.X()*0.5);
  v1a.Y(this->terrainSize.Y()*0.5 - v1a.Y()*this->terrainScaling.Y());

  v2a.X(v2a.X()*this->terrainScaling.X() - this->terrainSize.X()*0.5);
  v2a.Y(this->terrainSize.Y()*0.5 - v2a.Y()*this->terrainScaling.Y());

  v3a.X(v3a.X()*this->terrainScaling.X() - this->terrainSize.X()*0.5);
  v3a.Y(this->terrainSize.Y()*0.5 - v3a.Y()*this->terrainScaling.Y());

  _norm = ignition::math::Vector3d::Normal(v1a, v2a, v3a);

  // Triangle normal
  ignition::math::Vector3d norm = ignition::math::Vector3d::Normal(v1, v2, v3);

  // Ray direction to intersect with triangle
  ignition::math::Vector3d rayDir(0, 0, -1);

  // Ray start point
  ignition::math::Vector3d rayPt(robotPos.X(), robotPos.Y(), 1000);

  // Distance from ray start to triangle intersection
  double intersection = -norm.Dot(rayPt - v1) / norm.Dot(rayDir);

  // Height of the terrain
  _terrainPos = rayPt + intersection * rayDir;
}
