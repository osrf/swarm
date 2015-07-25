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

  // ToDo: Include here the neighbors list.

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
    gzerr << "RobotPlugin::Load(): Unable to find the <address> parameter"
          << std::endl;
    return;
  }

  this->address = _sdf->Get<std::string>("address");

  const std::string kBrokerIncomingTopic = "/swarm/broker/incoming";
  if (!this->node.Advertise(kBrokerIncomingTopic))
  {
    gzerr << "[" << this->Host() << "] RobotPlugin::Load(): Error "
          << "trying to advertise topic [" << kBrokerIncomingTopic << "]"
          << std::endl;
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

  // There's visibility between source and destination: run the user callback.
  auto const &userCallback = this->callbacks[topic];
  userCallback(_msg.src_address(), _msg.data());
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
