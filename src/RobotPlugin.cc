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
#include "msgs/log_entry.pb.h"
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
    modelHeight2(0),
    observedLatitude(0),
    observedLongitude(0),
    observedAltitude(0),
    startCapacity(1),
    capacity(1),
    consumption(0),
    consumptionFactor(0)
{
}

//////////////////////////////////////////////////
RobotPlugin::~RobotPlugin()
{
  gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->broker->Unregister(this->Host());
  this->logger->Unregister(this->Host());
}

//////////////////////////////////////////////////
void RobotPlugin::Load(sdf::ElementPtr /*_sdf*/)
{
}

//////////////////////////////////////////////////
bool RobotPlugin::SendTo(const std::string &_data,
    const std::string &_dstAddress, const uint32_t _port)
{
  // Restrict the maximum size of a message.
  if (_data.size() > this->kMtu)
  {
    gzerr << "[" << this->Host() << "] RobotPlugin::SendTo() error: Payload "
          << "size (" << _data.size() << ") is greater than the maximum "
          << "allowed (" << this->kMtu << ")" << std::endl;
    return false;
  }

  msgs::Datagram msg;
  msg.set_src_address(this->Host());
  msg.set_dst_address(_dstAddress);
  msg.set_dst_port(_port);
  msg.set_data(_data);

  // The neighbors list will be included in the broker.

  this->broker->Push(msg);

  return true;
}

//////////////////////////////////////////////////
bool RobotPlugin::SetLinearVelocity(const ignition::math::Vector3d &_velocity)
{
  if (this->capacity <= 0 || (this->type == ROTOR && this->rotorDocked))
    return false;

  this->targetLinVel = _velocity;

  return true;
}

//////////////////////////////////////////////////
bool RobotPlugin::SetLinearVelocity(const double _x, const double _y,
    const double _z)
{
  return this->SetLinearVelocity(ignition::math::Vector3d(_x, _y, _z));
}

//////////////////////////////////////////////////
bool RobotPlugin::SetAngularVelocity(const ignition::math::Vector3d &_velocity)
{
  if (this->capacity <= 0 || (this->type == ROTOR && this->rotorDocked))
    return false;

  this->targetAngVel = _velocity;
  return true;
}

//////////////////////////////////////////////////
bool RobotPlugin::SetAngularVelocity(const double _x, const double _y,
    const double _z)
{
  return this->SetAngularVelocity(ignition::math::Vector3d(_x, _y, _z));
}

//////////////////////////////////////////////////
void RobotPlugin::UpdateSensors()
{
  if (this->gps)
  {
    this->observedLatitude = this->gps->Latitude().Degree();
    this->observedLongitude = this->gps->Longitude().Degree();
    this->observedAltitude = this->gps->GetAltitude();
  }

  if (this->imu)
  {
    this->linearVelocityNoNoise = this->model->GetRelativeLinearVel().Ign();
    this->angularVelocityNoNoise = this->model->GetRelativeAngularVel().Ign();

    this->observedlinVel = this->linearVelocityNoNoise +
      ignition::math::Vector3d(
          ignition::math::Rand::DblNormal(0, 0.0002),
          ignition::math::Rand::DblNormal(0, 0.0002),
          ignition::math::Rand::DblNormal(0, 0.0002));

    this->observedAngVel = this->imu->AngularVelocity();
    this->observedOrient = this->imu->Orientation();
  }

  // Get the Yaw angle of the model in Gazebo world coordinates.
  this->observedBearing = ignition::math::Angle(
      this->model->GetWorldPose().rot.GetAsEuler().z +
      ignition::math::Rand::DblNormal(0, 0.035));

  // A "0" bearing value means that the model is facing North.
  // North is aligned with the Gazebo Y axis, so we should add an offset of
  // PI/2 to the bearing in the Gazebo world coordinates.
  this->observedBearing = ignition::math::Angle::HalfPi - this->observedBearing;

  // Normalize: Gazebo orientation uses PI,-PI but compasses seem
  // to use 0,2*PI.
  if (this->observedBearing.Radian() < 0)
    this->observedBearing = ignition::math::Angle::TwoPi +this->observedBearing;

  // Update camera.
  this->img.objects.clear();
  if (this->camera)
  {
    ignition::math::Pose3d myPose = this->model->GetWorldPose().Ign();
    gazebo::msgs::LogicalCameraImage logicalImg = this->camera->Image();

    // Process each object, and add noise
    for (auto const imgModel : logicalImg.model())
    {
      // Skip ground plane model
      if (imgModel.name() == "ground_plane")
        continue;

      // Pose of the detected model
      ignition::math::Pose3d p = gazebo::msgs::ConvertIgn(imgModel.pose());

      // Distance to the detected model
      double dist = p.Pos().Length();

      // Normalized (to the camera's frustum) squared distance
      double distSquaredNormalized = std::pow(dist, 2) /
        std::pow(this->camera->Far(), 2);

      // A percentage of the time we get a false negative
      if (ignition::math::Rand::DblUniform(
            this->cameraFalseNegativeProbMin,
            this->cameraFalseNegativeProbMax) < distSquaredNormalized)
      {
        continue;
      }

      // Compute amount of possible position noise.
      double posError = this->cameraMaxPositionError * distSquaredNormalized;

      // Add noise to the position of the model.
      p.Pos().X() += ignition::math::Rand::DblUniform(-posError, posError);
      p.Pos().Y() += ignition::math::Rand::DblUniform(-posError, posError);
      p.Pos().Z() += ignition::math::Rand::DblUniform(-posError, posError);

      // A percentage of the time we get a false positive for the lost person,
      if (ignition::math::Rand::DblUniform(
            this->cameraFalsePositiveProbMin,
            this->cameraFalsePositiveProbMax) < distSquaredNormalized)
      {
        // Randomly choose a model name
        this->img.objects[this->modelNames[
          ignition::math::Rand::IntUniform(0, this->modelNames.size()-1)]] = p;
      }
      else
      {
        this->img.objects[imgModel.name()] = p;
      }
    }
  }
}

//////////////////////////////////////////////////
void RobotPlugin::UpdateLinearVelocity()
{
  if (this->capacity <= 0 || (this->type == ROTOR && this->rotorDocked))
    return;

  auto myPose = this->model->GetWorldPose().Ign();

  ignition::math::Vector3d linearVel;
  double limitFactor = 1.0;

  switch (this->type)
  {
    default:
    case RobotPlugin::GROUND:
      {
        // Get linear velocity in world frame
        linearVel = myPose.Rot().RotateVector(
            this->targetLinVel * ignition::math::Vector3d::UnitX);

        limitFactor = linearVel.Length() / this->groundMaxLinearVel;
        break;
      }
    case RobotPlugin::ROTOR:
      {
        // Get linear velocity in world frame
        linearVel = myPose.Rot().RotateVector(this->targetLinVel);

        limitFactor = linearVel.Length() / this->rotorMaxLinearVel;
        break;
      }
    case RobotPlugin::FIXED_WING:
      {
        // Get linear velocity in world frame
        linearVel = myPose.Rot().RotateVector(
            this->targetLinVel * ignition::math::Vector3d::UnitX);

        limitFactor = linearVel.Length() / this->fixedMaxLinearVel;
        break;
      }
  };

  // Clamp the linear velocity
  linearVel = linearVel /
    ignition::math::clamp(limitFactor, 1.0, limitFactor);

  this->model->SetLinearVel(linearVel);
}

//////////////////////////////////////////////////
void RobotPlugin::UpdateAngularVelocity()
{
  if (this->capacity <= 0 || (this->type == ROTOR && this->rotorDocked))
    return;

  switch (this->type)
  {
    default:
    case RobotPlugin::GROUND:
      {
        double vel = ignition::math::clamp(this->targetAngVel.Z(),
            -this->groundMaxAngularVel, this->groundMaxAngularVel);
        this->model->SetAngularVel(ignition::math::Vector3d(0, 0, vel));
        break;
      }
    case RobotPlugin::ROTOR:
      {
        // Clamp the angular velocity
        double limitFactor = this->targetAngVel.Length() /
          this->rotorMaxAngularVel;
        ignition::math::Vector3d vel = this->targetAngVel /
          ignition::math::clamp(limitFactor, 1.0, limitFactor);

        this->model->SetAngularVel(vel);
        break;
      }
    case RobotPlugin::FIXED_WING:
      {
        double yawRate = 0.0;
        double rollRate = 0.0;

        // Current orientation as Euler angles
        ignition::math::Vector3d rpy = this->observedOrient.Euler();

        // Make sure we don't divide by zero. The vehicle should also
        // be moving before it can bank.
        if (!ignition::math::equal(this->linearVelocityNoNoise.X(), 0.0))
        {
          yawRate = (-9.81 * tan(rpy.X())) / this->linearVelocityNoNoise.X();
          yawRate = ignition::math::clamp(yawRate, -IGN_DTOR(10), IGN_DTOR(10));
          rollRate = ignition::math::clamp(this->targetAngVel[0],
              -IGN_DTOR(5), IGN_DTOR(5));
        }

        this->model->SetAngularVel(ignition::math::Vector3d(rollRate,
              ignition::math::clamp(this->targetAngVel[1],
                -this->fixedMaxAngularVel, this->fixedMaxAngularVel), yawRate));
        break;
      }
  };
}

//////////////////////////////////////////////////
bool RobotPlugin::Imu(ignition::math::Vector3d &_linVel,
  ignition::math::Vector3d &_angVel, ignition::math::Quaterniond &_orient) const
{
  _linVel = this->observedlinVel;
  _angVel = this->observedAngVel;
  _orient = this->observedOrient;
  return true;
}

//////////////////////////////////////////////////
bool RobotPlugin::Bearing(ignition::math::Angle &_bearing) const
{
  _bearing = this->observedBearing;
  return true;
}

//////////////////////////////////////////////////
bool RobotPlugin::BooPose(double &_latitude, double &_longitude) const
{
  if (!this->boo)
    return false;

  // Convert gazebo pose to lat/lon
  ignition::math::Vector3d spherical =
    this->world->GetSphericalCoordinates()->SphericalFromLocal(
        this->boo->GetWorldPose().Ign().Pos());

  _latitude = spherical.X();
  _longitude = spherical.Y();

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

  _latitude = this->observedLatitude;
  _longitude = this->observedLongitude;
  _altitude = this->observedAltitude;

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

  _img = this->img;

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
  return this->neighbors;
}

//////////////////////////////////////////////////
void RobotPlugin::Update(const gazebo::common::UpdateInfo & /*_info*/)
{
}

//////////////////////////////////////////////////
void RobotPlugin::Launch()
{
  this->rotorDocked = false;
  this->rotorDockVehicle.reset();
}

//////////////////////////////////////////////////
bool RobotPlugin::Dock(const std::string &_vehicle)
{
  if (this->type != ROTOR || this->rotorDocked)
    return false;

  // We are assuming that all ground vehicles have the word "ground" in the
  // name
  if (_vehicle.find("ground") == std::string::npos)
  {
    gzerr << "Can only dock with ground vehicles.\n";
    return false;
  }

  gazebo::physics::ModelPtr m = this->world->GetModel(_vehicle);
  if (!m)
  {
    gzerr << "Invalid dock vehicle[" << _vehicle << "]\n";
    return false;
  }

  if (m->GetWorldPose().pos.Distance(this->model->GetWorldPose().pos) <=
      this->rotorDockingDistance)
  {
    this->SetLinearVelocity(0, 0, 0);
    this->SetAngularVelocity(0, 0, 0);

    this->rotorDockVehicle = m;
    this->rotorDocked = true;
  }

  return this->rotorDocked;
}

//////////////////////////////////////////////////
void RobotPlugin::Loop(const gazebo::common::UpdateInfo &_info)
{
  // Update the state of the battery
  this->UpdateBattery();

  // Only update sensors if we have enough juice
  if (this->capacity > 0)
  {
    this->UpdateSensors();
    this->SetLinearVelocity(0, 0, 0);
    this->SetAngularVelocity(0, 0, 0);
  }

  // Always give the team controller an update.
  this->Update(_info);

  // Apply the controller's actions to the simulation.
  this->UpdateLinearVelocity();
  this->UpdateAngularVelocity();

  // Adjust pose as necessary.
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
        if (!this->rotorDocked)
        {
          if (pose.Pos().Z() < terrainPos.Z() + this->modelHeight2)
          {
            pose.Pos().Z(terrainPos.Z() + this->modelHeight2);

            // Set the pose.
            this->model->SetWorldPose(pose);
          }
        }
        else
        {
          this->model->SetWorldPose(this->rotorDockVehicle->GetWorldPose());
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

  // We assume that the physics step size will not change during simulation.
  this->world = this->model->GetWorld();

  // We assume the BOO is named "boo".
  this->boo = this->world->GetModel("boo");

  if (!this->boo)
  {
    gzwarn << "No base of operations (BOO) found.\n";
  }

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


  // Load battery information
  if (_sdf->HasElement("battery"))
  {
    sdf::ElementPtr battery = _sdf->GetElement("battery");

    this->startCapacity = battery->Get<double>("capacity");
    this->capacity = this->startCapacity;

    this->consumption = battery->Get<double>("consumption");

    this->consumptionFactor = ignition::math::clamp(
        battery->Get<double>("consumption_factor"), 0.0, 1.0);
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

  // We treat the BOO specially; it's a robot, but doesn't have any sensors.
  if (this->address != "boo")
  {
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

  // Get the launch vehicle, if specified
  if (this->type == ROTOR && _sdf->HasElement("launch_vehicle"))
  {
    this->rotorDockVehicle = this->world->GetModel(
        _sdf->Get<std::string>("launch_vehicle"));

    if (!this->rotorDockVehicle)
    {
      this->rotorDocked = false;
      gzerr << "Unable to get dock vehicle["
        << _sdf->Get<std::string>("launch_vehicle") << "]\n";
    }
  }
  else
  {
    this->rotorDocked = false;
  }

  this->AdjustPose();

  // Register this plugin in the broker.
  this->broker->Register(this->Host(), this);

  // Register this plugin in the logger.
  this->logger->Register(this->Host(), this);

  // Call the Load() method from the derived plugin.
  this->Load(_sdf);

  // This is debug code that can be used to render markers in Gazebo. It
  // requires a version of gazebo with visual markers.
  // this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  // this->gzNode->Init();
  // this->markerPub =
  // this->gzNode->Advertise<gazebo::msgs::Marker>("~/marker");
  // END DEBUG CODE

  // Get all the model names, and add every name expect this->model to
  // a list. This list is used to compute false positives in the image
  // data
  for (auto const &worldModel : this->world->GetModels())
  {
    if (worldModel->GetName() != this->model->GetName() &&
        worldModel->GetName() != "ground_plane")
    {
      this->modelNames.push_back(worldModel->GetName());
    }
  }

  // Listen to the update event broadcasted every simulation iteration.
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RobotPlugin::Loop, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void RobotPlugin::OnMsgReceived(const msgs::Datagram &_msg) const
{
  const std::string topic = _msg.dst_address() + ":" +
      std::to_string(_msg.dst_port());

  if (this->callbacks.find(topic) == this->callbacks.end())
  {
    gzerr << "[" << this->Host() << "] RobotPlugin::OnMsgReceived(): "
          << "Address [" << topic << "] not found" << std::endl;
    return;
  }

  auto const &userCallback = this->callbacks.at(topic);
  userCallback(_msg.src_address(), _msg.dst_address(),
               _msg.dst_port(), _msg.data());
}

//////////////////////////////////////////////////
void RobotPlugin::OnNeighborsReceived(
  const std::vector<std::string> &_neighbors)
{
  this->neighbors = _neighbors;
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

/////////////////////////////////////////////////
void RobotPlugin::UpdateBattery()
{
  if (this->model->GetName() == "boo")
    return;

  double distToBOO = IGN_DBL_MAX;

  if (this->boo)
  {
    distToBOO = this->model->GetWorldPose().pos.Distance(
        this->boo->GetWorldPose().pos);
  }

  // Check to see if the robot is in a recharge state:
  //    - Near the BOO
  //    - Not moving
  if ((distToBOO < this->booRechargeDistance &&
      this->linearVelocityNoNoise == ignition::math::Vector3d::Zero &&
      this->angularVelocityNoNoise == ignition::math::Vector3d::Zero) ||
      (this->type == ROTOR && this->rotorDocked))
  {
    // The amount of the capacity recharged.
    double mAhRecharged = (this->consumption * (this->consumptionFactor*4) *
        (this->world->GetPhysicsEngine()->GetMaxStepSize() / 3600.0));

    this->capacity = std::min(this->capacity + mAhRecharged,
                              this->startCapacity);
  }
  else
  {
    // The amount of the capacity consumed.
    double mAhConsumed = (this->consumption * this->consumptionFactor *
        (this->world->GetPhysicsEngine()->GetMaxStepSize() / 3600.0));

    this->capacity = std::max(0.0, this->capacity - mAhConsumed);
  }
}

/////////////////////////////////////////////////
double RobotPlugin::BatteryStartCapacity() const
{
  return this->startCapacity;
}

/////////////////////////////////////////////////
double RobotPlugin::BatteryCapacity() const
{
  return this->capacity;
}

/////////////////////////////////////////////////
double RobotPlugin::BatteryConsumption() const
{
  return this->consumption;
}

/////////////////////////////////////////////////
double RobotPlugin::BatteryConsumptionFactor() const
{
  return this->consumptionFactor;
}

/////////////////////////////////////////////////
double RobotPlugin::ExpectedBatteryLife() const
{
  return ((this->capacity / this->consumption) * this->consumptionFactor) *
    3600;
}

/////////////////////////////////////////////////
ignition::math::Pose3d RobotPlugin::CameraToWorld(
  const ignition::math::Pose3d &_poseinCamera) const
{
  auto poseInWorld = _poseinCamera + this->model->GetWorldPose().Ign();
  return poseInWorld;
}

/////////////////////////////////////////////////
void RobotPlugin::OnLog(msgs::LogEntry &_logEntry) const
{
  // Fill the last GPS observation.
  msgs::Gps *obsGps = new msgs::Gps();
  obsGps->set_latitude(this->observedLatitude);
  obsGps->set_longitude(this->observedLongitude);
  obsGps->set_altitude(this->observedAltitude);

  // Fill the last IMU observation.
  gazebo::msgs::Vector3d *obsVlin = new gazebo::msgs::Vector3d();
  gazebo::msgs::Vector3d *obsVang = new gazebo::msgs::Vector3d();
  gazebo::msgs::Quaternion *obsOrient = new gazebo::msgs::Quaternion();
  msgs::Imu *obsImu = new msgs::Imu();
  obsVlin->set_x(this->observedlinVel.X());
  obsVlin->set_y(this->observedlinVel.Y());
  obsVlin->set_z(this->observedlinVel.Z());
  obsVang->set_x(this->observedAngVel.X());
  obsVang->set_y(this->observedAngVel.Y());
  obsVang->set_z(this->observedAngVel.Z());
  obsOrient->set_x(this->observedOrient.X());
  obsOrient->set_y(this->observedOrient.Y());
  obsOrient->set_z(this->observedOrient.Z());
  obsOrient->set_w(this->observedOrient.W());
  obsImu->set_allocated_linvel(obsVlin);
  obsImu->set_allocated_angvel(obsVang);
  obsImu->set_allocated_orientation(obsOrient);

  // Fill the camera observation.
  msgs::ImageData *obsImage = new msgs::ImageData();
  for (const auto imgObj : this->img.objects)
  {
    msgs::ObjPose *obj = obsImage->add_object();
    obj->set_name(imgObj.first);
    obj->mutable_pose()->mutable_position()->set_x(imgObj.second.Pos().X());
    obj->mutable_pose()->mutable_position()->set_y(imgObj.second.Pos().Y());
    obj->mutable_pose()->mutable_position()->set_z(imgObj.second.Pos().Z());
    obj->mutable_pose()->mutable_orientation()->set_x(imgObj.second.Rot().X());
    obj->mutable_pose()->mutable_orientation()->set_y(imgObj.second.Rot().Y());
    obj->mutable_pose()->mutable_orientation()->set_z(imgObj.second.Rot().Z());
    obj->mutable_pose()->mutable_orientation()->set_w(imgObj.second.Rot().W());
  }

  msgs::Sensors *sensors = new msgs::Sensors();
  sensors->set_allocated_gps(obsGps);
  sensors->set_allocated_imu(obsImu);
  sensors->set_bearing(this->observedBearing.Radian());
  sensors->set_allocated_image(obsImage);
  sensors->set_battery_capacity(this->BatteryCapacity());

  // Fill the sensor information of the log entry.
  _logEntry.set_allocated_sensors(sensors);

  // Fill the actions.
  gazebo::msgs::Vector3d *targetVlin = new gazebo::msgs::Vector3d();
  gazebo::msgs::Vector3d *targetVang = new gazebo::msgs::Vector3d();
  targetVlin->set_x(this->targetLinVel.X());
  targetVlin->set_y(this->targetLinVel.Y());
  targetVlin->set_z(this->targetLinVel.Z());
  targetVang->set_x(this->targetAngVel.X());
  targetVang->set_y(this->targetAngVel.Y());
  targetVang->set_z(this->targetAngVel.Z());

  msgs::Actions *actions = new msgs::Actions();
  actions->set_allocated_linvel(targetVlin);
  actions->set_allocated_angvel(targetVang);

  _logEntry.set_allocated_actions(actions);
}

/////////////////////////////////////////////////
void RobotPlugin::SetCameraOrientation(const double _pitch, const double _yaw)
{
  // Lock pitch to +/- 90 degrees
  double pitch = ignition::math::clamp(_pitch, -IGN_PI_2, IGN_PI_2);

  ignition::math::Pose3d camPose = this->camera->Pose();
  ignition::math::Vector3d camEuler = camPose.Rot().Euler();
  camEuler.Y(pitch);
  camEuler.Z(_yaw);
  camPose.Rot().Euler(camEuler);

  // Set the new pose
  this->camera->SetPose(camPose);
}

//////////////////////////////////////////////////
void RobotPlugin::CameraOrientation(double &_pitch, double &_yaw) const
{
  // Return the pitch and yaw of the camera
  ignition::math::Vector3d camEuler = this->camera->Pose().Rot().Euler();
  _pitch = camEuler.Y();
  _yaw = camEuler.Z();
}
