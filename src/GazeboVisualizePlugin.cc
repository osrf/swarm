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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/program_options.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "swarm/GazeboVisualizePlugin.hh"

using namespace gazebo;
namespace po = boost::program_options;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboVisualizePlugin)

/////////////////////////////////////////////
GazeboVisualizePlugin::~GazeboVisualizePlugin()
{
}

/////////////////////////////////////////////
void GazeboVisualizePlugin::Load(int /*_argc*/, char ** /*_argv*/)
{
  std::cout << "GazeboVisualizePlugin::Load\n";
/*  po::options_description v_desc("Options");
  v_desc.add_options()
    ("propshop-save", po::value<std::string>(),
     "Path to save image files into.")
    ("propshop-model", po::value<std::string>(), "Model to spawn.");

  po::options_description desc("Options");
  desc.add(v_desc);

  po::variables_map vm;
  try
  {
    po::store(po::command_line_parser(_argc, _argv).options(
          desc).allow_unregistered().run(), vm);
    po::notify(vm);
  } catch(boost::exception &_e)
  {
    std::cerr << "Error. Invalid arguments\n";
    return;
  }

  // Get the directory in which to save the images.
  if (vm.count("propshop-save"))
  {
    this->savePath = boost::filesystem::path(
        vm["propshop-save"].as<std::string>());
    if (!boost::filesystem::exists(this->savePath))
      boost::filesystem::create_directories(this->savePath);
  }
  else
    this->savePath = boost::filesystem::temp_directory_path();

  std::string modelFile;

  if (vm.count("propshop-model"))
    modelFile = vm["propshop-model"].as<std::string>();
  else
    return;

  std::ifstream ifs(modelFile.c_str());
  if (!ifs)
  {
    std::cerr << "Error: Unable to open file[" << modelFile << "]\n";
    return;
  }

  this->sdf.reset(new sdf::SDF());
  if (!sdf::init(this->sdf))
  {
    std::cerr << "ERROR: SDF parsing the xml failed" << std::endl;
    return;
  }

  if (!sdf::readFile(modelFile, this->sdf))
  {
    std::cerr << "Error: SDF parsing the xml failed\n";
    return;
  }

  sdf::ElementPtr modelElem = this->sdf->Root()->GetElement("model");
  this->modelName = modelElem->Get<std::string>("name");
  */

  this->searchMinLatitude = 35.7653;
  this->searchMaxLatitude = 35.7853;

  this->searchMinLongitude = -120.784;
  this->searchMaxLongitude = -120.764;

  std::string logFile = "/home/nkoenig/Downloads/swarm.log";
  this->parser.Load(logFile);

  swarm::msgs::LogHeader header;
  if (!this->parser.Header(header))
  {
    std::cerr << "Error parsing header from [" << logFile << "]" << std::endl;
  }
  else
  {
    std::cout << "Swarm Version:  " << header.swarm_version() << std::endl;
    std::cout << "Gazebo Version: " << header.gazebo_version() << std::endl;
    std::cout << "Random Seed:    " << header.seed() << std::endl;
    std::cout << std::endl;
  }
}

/////////////////////////////////////////////
void GazeboVisualizePlugin::Init()
{
  this->worldCreatedConn = event::Events::ConnectWorldCreated(
        boost::bind(&GazeboVisualizePlugin::OnWorldCreated, this));
}

/////////////////////////////////////////////
void GazeboVisualizePlugin::OnWorldCreated()
{
  this->updateConn = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboVisualizePlugin::Update, this));

  this->world = physics::get_world();

  // Create our node for communication
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();

  // Publish to a Gazebo topic
#if GAZEBO_MAJOR_VERSION >= 7
  this->markerPub = this->node->Advertise<gazebo::msgs::Marker>("~/marker");
#endif

  this->markerPub->WaitForConnection();

  // Get the terrain, if it's present
  gazebo::physics::ModelPtr terrainModel = this->world->GetModel("terrain");

  // Load some info about the terrain.
  if (terrainModel)
  {
    this->terrain =
      boost::dynamic_pointer_cast<gazebo::physics::HeightmapShape>(
          terrainModel->GetLink()->GetCollision("collision")->GetShape());

  }
  else
    gzerr << "Invalid terrain\n";

}

/////////////////////////////////////////////
void GazeboVisualizePlugin::VisualizeSearchArea()
{
#if GAZEBO_MAJOR_VERSION >= 7
  // The namespace of the markers generated in this function.
  std::string markerNS = "area";

  gazebo::msgs::Marker clearMsg;
  clearMsg.set_ns(markerNS);
  clearMsg.set_action(gazebo::msgs::Marker::DELETE_ALL);
  this->markerPub->Publish(clearMsg);

  gazebo::msgs::Marker markerMsg;
  markerMsg.set_ns(markerNS);
  markerMsg.set_id(0);
  markerMsg.set_action(gazebo::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gazebo::msgs::Marker::LINE_LIST);
  markerMsg.clear_point();
  markerMsg.mutable_material()->mutable_script()->set_name("Gazebo/Red");

  ignition::math::Vector3d pt1 =
    this->world->GetSphericalCoordinates()->LocalFromSpherical(
        ignition::math::Vector3d(this->searchMinLatitude,
          this->searchMinLongitude, 0));
  pt1 = this->world->GetSphericalCoordinates()->GlobalFromLocal(pt1);

  ignition::math::Vector3d pt2 =
    this->world->GetSphericalCoordinates()->LocalFromSpherical(
        ignition::math::Vector3d(this->searchMinLatitude, this->searchMaxLongitude, 0));
  pt2 = this->world->GetSphericalCoordinates()->GlobalFromLocal(pt2);

  ignition::math::Vector3d pt3 =
    this->world->GetSphericalCoordinates()->LocalFromSpherical(
        ignition::math::Vector3d(this->searchMaxLatitude, this->searchMaxLongitude, 0));
  pt3 = this->world->GetSphericalCoordinates()->GlobalFromLocal(pt3);

  ignition::math::Vector3d pt4 =
    this->world->GetSphericalCoordinates()->LocalFromSpherical(
        ignition::math::Vector3d(this->searchMaxLatitude, this->searchMinLongitude, 0));
  pt4 = this->world->GetSphericalCoordinates()->GlobalFromLocal(pt4);


  pt1.Z(920);
  pt2.Z(920);
  pt3.Z(920);
  pt4.Z(920);

  std::cout << "pt1[" << pt1 << "]\n";
  std::cout << "pt2[" << pt2 << "]\n";
  std::cout << "pt3[" << pt3 << "]\n";
  std::cout << "pt4[" << pt4 << "]\n";

  gazebo::msgs::Set(markerMsg.add_point(), pt1);
  gazebo::msgs::Set(markerMsg.add_point(), pt2);

  gazebo::msgs::Set(markerMsg.add_point(), pt2);
  gazebo::msgs::Set(markerMsg.add_point(), pt3);

  gazebo::msgs::Set(markerMsg.add_point(), pt3);
  gazebo::msgs::Set(markerMsg.add_point(), pt4);

  gazebo::msgs::Set(markerMsg.add_point(), pt4);
  gazebo::msgs::Set(markerMsg.add_point(), pt1);

  ignition::math::Vector3d pt1Lower = pt1;
  ignition::math::Vector3d pt2Lower = pt2;
  ignition::math::Vector3d pt3Lower = pt3;
  ignition::math::Vector3d pt4Lower = pt4;
  pt1Lower.Z(0);
  pt2Lower.Z(0);
  pt3Lower.Z(0);
  pt4Lower.Z(0);

  gazebo::msgs::Set(markerMsg.add_point(), pt1);
  gazebo::msgs::Set(markerMsg.add_point(), pt1Lower);

  gazebo::msgs::Set(markerMsg.add_point(), pt2);
  gazebo::msgs::Set(markerMsg.add_point(), pt2Lower);

  gazebo::msgs::Set(markerMsg.add_point(), pt3);
  gazebo::msgs::Set(markerMsg.add_point(), pt3Lower);

  gazebo::msgs::Set(markerMsg.add_point(), pt4);
  gazebo::msgs::Set(markerMsg.add_point(), pt4Lower);

  this->markerPub->Publish(markerMsg);

  // get lat/lon bounds
  double stepLon = (this->searchMaxLongitude - this->searchMinLongitude) / 15.0;
  double stepLat = (this->searchMaxLatitude - this->searchMinLatitude) / 15.0;

  double elevation;
  GazeboVisualizePlugin::TerrainType terrainType;

  int index = 10;
  for (double lat = this->searchMinLatitude; lat < this->searchMaxLatitude;
      lat += stepLat)
  {
    for (double lon = this->searchMinLongitude; lon < this->searchMaxLongitude; lon += stepLon)
    {
      this->MapQuery(lat, lon, elevation, terrainType);

      gazebo::msgs::Marker elevMsg;
      elevMsg.set_ns(markerNS);
      elevMsg.set_id(index++);
      elevMsg.set_action(gazebo::msgs::Marker::ADD_MODIFY);
      elevMsg.set_type(gazebo::msgs::Marker::SPHERE);

      if (terrainType == PLAIN)
        elevMsg.mutable_material()->mutable_script()->set_name("Gazebo/Red");
      else if (terrainType == FOREST)
        elevMsg.mutable_material()->mutable_script()->set_name("Gazebo/Green");
      else
        elevMsg.mutable_material()->mutable_script()->set_name("Gazebo/Black");

      gazebo::msgs::Set(elevMsg.mutable_scale(),
          ignition::math::Vector3d(10.2, 10.2, 10.2));

      ignition::math::Vector3d local =
        this->world->GetSphericalCoordinates()->LocalFromSpherical(
            ignition::math::Vector3d(lat, lon, 0));
      local = this->world->GetSphericalCoordinates()->GlobalFromLocal(local);
      local.Z(elevation -
              this->world->GetSphericalCoordinates()->GetElevationReference());

      gazebo::msgs::Set(elevMsg.mutable_pose(),
        ignition::math::Pose3d(local, ignition::math::Quaterniond::Identity));

      this->markerPub->Publish(elevMsg);
    }
  }
  std::cout << "done!\n";


#endif
}

/////////////////////////////////////////////
void GazeboVisualizePlugin::VisualizeMessages(swarm::msgs::LogEntry &_logEntry)
{
#if GAZEBO_MAJOR_VERSION >= 7
  // The namespace of the markers generated in this function.
  std::string markerNS = "messages";

  // Clear visualis in the namespace
  gazebo::msgs::Marker clearMsg;
  clearMsg.set_ns(markerNS);
  clearMsg.set_action(gazebo::msgs::Marker::DELETE_ALL);
  this->markerPub->Publish(clearMsg);

  int msgIndex = 0;

  // Construct map of messages
  if (_logEntry.has_incoming_msgs())
  {
    // Process each message received by the broker
    for (int msg = 0; msg < _logEntry.incoming_msgs().message_size(); ++msg)
    {
      // Get the source address
      std::string src = _logEntry.incoming_msgs().message(msg).src_address();

      // Make sure the source address has a dot (the address should be an IP
      // address).
      if (src.find('.') == std::string::npos)
        continue;

      // Get the physics model
      std::string srcId = src.substr(src.rfind('.')+1);
      physics::ModelPtr model = this->world->GetModel("ground_" + srcId);
      if (!model)
        continue;

      // Create a marker message
      ignition::math::Vector3d srcPos = model->GetWorldPose().pos.Ign();
      gazebo::msgs::Marker markerMsg;
      markerMsg.set_ns(markerNS);
      markerMsg.set_id(msgIndex);
      markerMsg.set_action(gazebo::msgs::Marker::ADD_MODIFY);
      markerMsg.set_type(gazebo::msgs::Marker::LINE_LIST);
      markerMsg.clear_point();
      markerMsg.mutable_material()->mutable_script()->set_name("Gazebo/Red");

      // Construct the list of destination addresses
      std::list<std::string> dst;
      for (int n = 0;
          n < _logEntry.incoming_msgs().message(msg).neighbor_size(); ++n)
      {
        std::string dest =
          _logEntry.incoming_msgs().message(msg).neighbor(n).dst();
        if (dest.find('.') == std::string::npos)
          continue;

        std::string destId = dest.substr(dest.rfind('.') + 1);
        physics::ModelPtr mdl = this->world->GetModel("ground_" + destId);
        if (!mdl)
          continue;

        ignition::math::Vector3d destPos = mdl->GetWorldPose().pos.Ign();

        // Only draw lines for messages that were delivered.
        if (_logEntry.incoming_msgs().message(msg).neighbor(n).status() ==
            swarm::msgs::CommsStatus::DELIVERED)
        {
          gazebo::msgs::Set(markerMsg.add_point(), srcPos);
          gazebo::msgs::Set(markerMsg.add_point(), destPos);
        }
      }
      this->markerPub->Publish(markerMsg);
      ++msgIndex;
    }
  }
#endif
}

/////////////////////////////////////////////
void GazeboVisualizePlugin::VisualizeNeighbors(swarm::msgs::LogEntry &_logEntry)
{
#if GAZEBO_MAJOR_VERSION >= 7
  // The namespace of the markers generated in this function.
  std::string markerNS = "neighbors";

  // Clear visualis in the namespace
  gazebo::msgs::Marker clearMsg;
  clearMsg.set_ns(markerNS);
  clearMsg.set_action(gazebo::msgs::Marker::DELETE_ALL);
  this->markerPub->Publish(clearMsg);

  std::list< std::list<std::string> > circles;

  // Construct set of visibility circles
  if (_logEntry.has_visibility())
  {
    for (int row = 0; row < _logEntry.visibility().row_size(); ++row)
    {
      std::string source = _logEntry.visibility().row(row).src();
      std::list<std::string> vehicles;
      vehicles.push_back(source);
      for (int v = 0; v < _logEntry.visibility().row(row).entry_size(); ++v)
      {
        std::string dst = _logEntry.visibility().row(row).entry(v).dst();
        if (source != dst &&
            _logEntry.visibility().row(row).entry(v).status() ==
            swarm::msgs::CommsStatus::VISIBLE)
        {
          vehicles.push_back(dst);
        }
        else if (_logEntry.visibility().row(row).entry(v).status() ==
            swarm::msgs::CommsStatus::OUTAGE)
        {
          // std::cout << "In outage. src[" << source << "] d[" << dst << "]\n";
        }
      }
      circles.push_back(vehicles);
    }
  }

  int index = 0;
  for (auto c : circles)
  {
    std::vector<ignition::math::Vector3d> positions;
    ignition::math::Vector3d sum;
    for (auto v : c)
    {
      if (v.find('.') == std::string::npos)
        continue;

      std::string vId = v.substr(v.rfind('.')+1);
      physics::ModelPtr model = this->world->GetModel("ground_" + vId);
      if (model)
      {
        positions.push_back(model->GetWorldPose().pos.Ign());
        sum += model->GetWorldPose().pos.Ign();
      }
    }

    ignition::math::Vector3d center = sum;
    if (!positions.empty())
      center /= positions.size();

    double radius = 0;
    for (auto const &p : positions)
    {
      if (p.Distance(center) > radius)
        radius = p.Distance(center);
    }

    if (ignition::math::equal(radius, 0.0))
      radius = 2.0;

    gazebo::msgs::Marker markerMsg;
    markerMsg.set_ns(markerNS);
    markerMsg.set_id(index);
    markerMsg.set_action(gazebo::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(gazebo::msgs::Marker::LINE_LIST);
    markerMsg.clear_point();
    gazebo::msgs::Set(markerMsg.mutable_pose(),
        ignition::math::Pose3d(center, ignition::math::Quaterniond::Identity));
    markerMsg.mutable_material()->mutable_script()->set_name(
        "Gazebo/Blue");

    for (double t = 0; t <= 2*M_PI; t+= 0.01)
    {
      gazebo::msgs::Set(markerMsg.add_point(),
          ignition::math::Vector3d(radius * cos(t), radius * sin(t), 20.0));
    }
    gazebo::msgs::Set(markerMsg.add_point(),
          ignition::math::Vector3d(radius * cos(2*M_PI), radius * sin(2*M_PI), 20.0));

    this->markerPub->Publish(markerMsg);

    ++index;
  }

  /*if (this->circleCount > index)
  {
    for (int i = index; i < this->circleCount; ++i)
    {
      gazebo::msgs::Marker markerMsg;
      markerMsg.set_ns("neighbors");
      markerMsg.set_id(i);
      markerMsg.set_action(gazebo::msgs::Marker::ADD_MODIFY);
      markerMsg.set_type(gazebo::msgs::Marker::TRIANGLE_FAN);
      gazebo::msgs::Set(markerMsg.mutable_pose(),
        ignition::math::Pose3d(0, 0, -100, 0, 0, 0));

      this->markerPub->Publish(markerMsg);
    }
  }
  this->circleCount = index;
  */
#endif
}

/////////////////////////////////////////////
void GazeboVisualizePlugin::Update()
{
  static bool first = true;

  swarm::msgs::LogEntry logEntry;
  do
  {
    this->parser.Next(logEntry);
  } while (logEntry.id() != "broker");

  this->VisualizeMessages(logEntry);
  this->VisualizeNeighbors(logEntry);

  if (first)
  {
    // Get the size of the terrain
    this->terrainSize = this->terrain->GetSize().Ign();

    // Set the terrain scaling.
    this->terrainScaling.Set(this->terrain->GetSize().x /
        (this->terrain->GetVertexCount().x-1),
        this->terrain->GetSize().y /
        (this->terrain->GetVertexCount().y-1));

    std::cout << "TerrainSize[" << this->terrainSize << "] Scal[" << this->terrainScaling << "]\n";
    this->CreateLostPersonMarker();
    this->VisualizeSearchArea();
    first = false;
  }
}

/////////////////////////////////////////////////
void GazeboVisualizePlugin::CreateLostPersonMarker()
{
#if GAZEBO_MAJOR_VERSION >= 7
  gazebo::msgs::Marker markerMsg;
  markerMsg.set_ns("lost_person");
  markerMsg.set_id(0);
  markerMsg.set_parent("lost_person::link::visual");
  markerMsg.set_action(gazebo::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gazebo::msgs::Marker::CYLINDER);
  markerMsg.mutable_material()->mutable_script()->set_name(
      "Gazebo/BlueLaser");
  gazebo::msgs::Set(markerMsg.mutable_scale(),
      ignition::math::Vector3d(10, 10, 50));
  gazebo::msgs::Set(markerMsg.mutable_pose(),
      ignition::math::Pose3d(0, 0, 50, 0, 0, 0));

  this->markerPub->Publish(markerMsg);

  markerMsg.set_id(1);
  markerMsg.set_action(gazebo::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gazebo::msgs::Marker::LINE_LIST);
  gazebo::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d::Zero);
  gazebo::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 0, -25));

  this->markerPub->Publish(markerMsg);
#endif
}

//////////////////////////////////////////////////
void GazeboVisualizePlugin::TerrainLookup(const ignition::math::Vector3d &_pos,
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

//////////////////////////////////////////////////
bool GazeboVisualizePlugin::MapQuery(const double _lat, const double _lon,
    double &_height, TerrainType &_type)
{
  // Check that the lat and lon is in the search area
  if (_lat < this->searchMinLatitude  ||
      _lat > this->searchMaxLatitude ||
      _lon < this->searchMinLongitude ||
      _lon > this->searchMaxLongitude)
  {
    return false;
  }

  // Get the location in the local coordinate frame
  ignition::math::Vector3d local =
    this->world->GetSphericalCoordinates()->LocalFromSpherical(
        ignition::math::Vector3d(_lat, _lon, 0));

  local = this->world->GetSphericalCoordinates()->GlobalFromLocal(local);

  ignition::math::Vector3d pos, norm;

  // Reuse the terrain lookup function.
  this->TerrainLookup(local, pos, norm);

  // Add in the reference elevation.
  _height = pos.Z() +
    this->world->GetSphericalCoordinates()->GetElevationReference();
  local.Z(pos.Z());

  _type = this->TerrainAtPos(local);

  return true;
}

/////////////////////////////////////////////////
GazeboVisualizePlugin::TerrainType GazeboVisualizePlugin::TerrainAtPos(
    const ignition::math::Vector3d &_pos)
{
  TerrainType result = PLAIN;

  for (auto const &mdl : this->world->GetModels())
  {
    if (mdl->GetBoundingBox().Contains(_pos))
    {
      // The bounding box of a model is aligned to the global axis, and can
      // lead to incorrect results.
      // If a point is in the bounding box, then we use a ray-cast to see
      // if the point is actually within the model.
      gazebo::physics::ModelPtr rayModel = this->world->GetModelBelowPoint(
          gazebo::math::Vector3(_pos.X(), _pos.Y(), 1000));

      if (rayModel->GetName().find("tree") != std::string::npos)
      {
        result = FOREST;
        break;
      }
      else if (rayModel->GetName().find("building") != std::string::npos)
      {
        result = BUILDING;
        break;
      }
    }
  }

  return result;
}
