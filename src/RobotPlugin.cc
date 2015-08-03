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
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include "msgs/datagram.pb.h"
#include "msgs/neighbor_v.pb.h"
#include "swarm/RobotPlugin.hh"

using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)

//////////////////////////////////////////////////
RobotPlugin::RobotPlugin()
  : type(GROUND)
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
        this->model->SetAngularVel(_velocity *
            ignition::math::Vector3d(1, 1, 0));
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
    gzerr << "No logical_camera snesor available" << std::endl;
    return false;
  }

  _img.objects.clear();

  gazebo::msgs::LogicalCameraImage img = this->camera->Image();
  for (auto const imgModel : img.model())
  {
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
void RobotPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  this->Update(_info);
}

//////////////////////////////////////////////////
void RobotPlugin::Load(gazebo::physics::ModelPtr _model,
                       sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "RobotPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "RobotPlugin _sdf pointer is NULL");
  this->model = _model;

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
        gazebo::sensors::get_sensor(_sdf->Get<std::string>("camera")));

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
        gazebo::sensors::get_sensor(_sdf->Get<std::string>("gps")));

    if (!this->camera)
    {
      gzerr << "Trying to get a gps sensor for robot with address["
        << this->address << "], but the specified gps[" <<
        _sdf->Get<std::string>("gps") << "] has an incorrect type.\n";
    }
  }

  if (!this->gps)
  {
    gzwarn << "No gps sensor found on robot with address "
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

  // Call the Load() method from the derived plugin.
  this->Load(_sdf);

  // Listen to the update event broadcasted every simulation iteration.
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotPlugin::Update, this, std::placeholders::_1));
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
  for (auto i = 0; i < _msg.neighbors().size(); ++i)
  {
    if (_msg.neighbors(i) == this->Host())
    {
      visible = true;
      break;
    }
  }

  if (visible)
  {
    // There's visibility between source and destination: run the user callback.
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
