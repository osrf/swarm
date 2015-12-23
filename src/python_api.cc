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

#include <unordered_map>

#include "swarm/RobotPlugin.hh"

#include <Python.h>

using namespace swarm;

// Track the robot pointers
std::unordered_map<std::string, RobotPlugin*> robotPointers;

static RobotPlugin*
get_robot_pointer(const std::string &addr)
{
  char err_buf[1024];
  std::unordered_map<std::string, RobotPlugin*>::const_iterator it =
    robotPointers.find(addr);
  if(it == robotPointers.end())
  {
    snprintf(err_buf, sizeof(err_buf),
             "Unknown robot address: %s", addr.c_str());
    PyErr_SetString(PyExc_RuntimeError, err_buf);
    return NULL;
  }
  else
    return it->second;
}

/**
 * Python function for: bind
 */
static PyObject *
robot_bind(PyObject *, PyObject *args)
{
  char* robot_addr;
  char* addr;
  int port;
  if(!PyArg_ParseTuple(args, "ssi", &robot_addr, &addr, &port))
    return NULL;

  // Send to the right controller.
  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    bool res = robot->Bind(&RobotPlugin::OnDataReceivedPython,
                           robot, std::string(addr), port);
    return Py_BuildValue("b", res);
  }
  else
    return NULL;
}

/**
 * Python function for: Set linear velocity
 */
static PyObject *
robot_set_linear_velocity(PyObject *, PyObject *args)
{
  char* robot_addr;
  float x, y, z;
  if(!PyArg_ParseTuple(args, "sfff", &robot_addr, &x, &y, &z))
    return NULL;

  // Send to the right controller.
  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    bool res = robot->SetLinearVelocity(ignition::math::Vector3d(x, y, z));
    return Py_BuildValue("b", res);
  }
  else
    return NULL;
}

/**
 * Python function for: Set angular velocity
 */
static PyObject *
robot_set_angular_velocity(PyObject *, PyObject *args)
{
  char* robot_addr;
  float x, y, z;
  if(!PyArg_ParseTuple(args, "sfff", &robot_addr, &x, &y, &z))
    return NULL;

  // Send to the right controller.
  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    bool res = robot->SetAngularVelocity(ignition::math::Vector3d(x, y, z));
    return Py_BuildValue("b", res);
  }
  else
    return NULL;
}

/**
 * Python function for: ask for Neighbors.
 */
static PyObject *
robot_neighbors(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  // Send to the right controller.
  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    const std::vector<std::string> &neighbors = robot->Neighbors();
    PyObject *pArgs = PyTuple_New(neighbors.size());
    for (unsigned int i = 0; i < neighbors.size(); ++i)
    {
      PyObject *pValue = Py_BuildValue("s", neighbors.at(i).c_str());
      /* pValue reference stolen here: */
      PyTuple_SetItem(pArgs, i, pValue);
    }
    return pArgs;
  }
  else
    return NULL;
}

/**
 * Python function for: ask for sending.
 */
static PyObject *
robot_send_to(PyObject *, PyObject *args)
{
  char* robot_addr;
  char *data, *dest;
  int port;
  if(!PyArg_ParseTuple(args, "sssi", &robot_addr, &data, &dest, &port))
    return NULL;

  // Send message
  // Send to the right controller.
  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    bool sent = robot->SendTo(std::string(data), std::string(dest), port);
    return Py_BuildValue("b", sent);
  }
  else
    return NULL;
}

/**
 * Python function for: ask for GPS localization.
 */
static PyObject *
robot_pose(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  // Get pose and altitude.
  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    double latitude, longitude, altitude;
    robot->Pose(latitude, longitude, altitude);
  
    PyObject *pArgs = PyTuple_New(3);
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("d", latitude));
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", longitude));
    PyTuple_SetItem(pArgs, 2, Py_BuildValue("d", altitude));
  
    return pArgs;
  }
  else
    return NULL;
}

/**
 * Python function for: ask for GPS localization.
 */
static PyObject *
robot_boo_pose(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  // Get pose and altitude.
  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    double latitude, longitude;
    robot->BooPose(latitude, longitude);
  
    PyObject *pArgs = PyTuple_New(2);
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("d", latitude));
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", longitude));
  
    return pArgs;
  }
  else
    return NULL;
}

/**
 * Python function for: ask for lost person direction
 */
static PyObject *
robot_lost_person_dir(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  // Get direction
  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    ignition::math::Vector2d dir = robot->LostPersonDir();
  
    PyObject *pArgs = PyTuple_New(2);
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("i", dir.X()));
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("i", dir.Y()));
  
    return pArgs;
  }
  else
    return NULL;
}

/**
 * Python function for: ask for IMU.
 */
static PyObject *
robot_imu(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    // Get IMU information.
    ignition::math::Vector3d linVel, angVel;
    ignition::math::Quaterniond orient;
    robot->Imu(linVel, angVel, orient);
  
    // Return
    PyObject *pArgs = PyTuple_New(9);
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("f", linVel.X()));
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("f", linVel.Y()));
    PyTuple_SetItem(pArgs, 2, Py_BuildValue("f", linVel.Z()));
    PyTuple_SetItem(pArgs, 3, Py_BuildValue("f", angVel.X()));
    PyTuple_SetItem(pArgs, 4, Py_BuildValue("f", angVel.Y()));
    PyTuple_SetItem(pArgs, 5, Py_BuildValue("f", angVel.Z()));
    PyTuple_SetItem(pArgs, 6, Py_BuildValue("f", orient.X()));
    PyTuple_SetItem(pArgs, 7, Py_BuildValue("f", orient.Y()));
    PyTuple_SetItem(pArgs, 8, Py_BuildValue("f", orient.Z()));
  
    return pArgs;
  }
  else
    return NULL;
}

/**
 * Python function for: ask for IMU.
 */
static PyObject *
robot_bearing(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    // Get IMU information.
    ignition::math::Angle bearing;
    robot->Bearing(bearing);

    // Return
    return Py_BuildValue("f", bearing.Radian());
  }
  else
    return NULL;
}

/**
 * Python function for: SearchArea function.
 */
static PyObject *
robot_search_area(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    // Get pose and altitude.
    double minLatitude, maxLatitude, minLongitude, maxLongitude;
    robot->SearchArea(minLatitude, maxLatitude, minLongitude, maxLongitude);
  
    PyObject *pArgs = PyTuple_New(4);
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("d", minLatitude));
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", maxLatitude));
    PyTuple_SetItem(pArgs, 2, Py_BuildValue("d", minLongitude));
    PyTuple_SetItem(pArgs, 3, Py_BuildValue("d", maxLongitude));
  
    return pArgs;
  }
  else
    return NULL;
}

/**
 * Python function for: image function.
 */
static PyObject *
robot_image(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!(PyArg_ParseTuple(args, "s", &robot_addr)))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    ImageData img;
    robot->Image(img);
    PyObject *camera_locs = PyTuple_New(img.objects.size());
    int i = 0;
    for (auto const obj : img.objects)
    {
      PyObject *camera_obj = PyTuple_New(2);
      PyTuple_SetItem(camera_obj, 0, Py_BuildValue("s", obj.first.c_str()));
      ignition::math::Pose3d pose = robot->CameraToWorld(obj.second);
      PyObject *obj_pos = PyTuple_New(3);
      PyTuple_SetItem(obj_pos, 0, Py_BuildValue("d", pose.Pos().X()));
      PyTuple_SetItem(obj_pos, 1, Py_BuildValue("d", pose.Pos().Y()));
      PyTuple_SetItem(obj_pos, 2, Py_BuildValue("d", pose.Pos().Z()));
      PyTuple_SetItem(camera_obj, 1, obj_pos);
      PyTuple_SetItem(camera_locs, i++, camera_obj);
    }
    return camera_locs;
  }
  else
    return NULL;
}

/**
 * Python function for: vehicle type
 */
static PyObject *
robot_type(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    // Get vehicle type
    RobotPlugin::VehicleType vtype = robot->Type();
    std::string vtypeString;
    switch(vtype)
    {
      case RobotPlugin::GROUND:
        vtypeString = "ground";
        break;
      case RobotPlugin::ROTOR:
        vtypeString = "rotor";
        break;
      case RobotPlugin::FIXED_WING:
        vtypeString = "fixed_wing";
        break;
      case RobotPlugin::BOO:
        vtypeString = "boo";
        break;
      default:
        PyErr_SetString(PyExc_RuntimeError, "unknown vehicle type");
        return NULL;
    }

    // Return
    return Py_BuildValue("s", vtypeString.c_str());
  }
  else
    return NULL;
}

/**
 * Python function for: terrain type
 */
static PyObject *
robot_terrain_type(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    // Get vehicle type
    TerrainType ttype = robot->Terrain();
    std::string ttypeString;
    switch(ttype)
    {
      case PLAIN:
        ttypeString = "plain";
        break;
      case FOREST:
        ttypeString = "forest";
        break;
      case BUILDING:
        ttypeString = "building";
        break;
      default:
        PyErr_SetString(PyExc_RuntimeError, "unknown terrain type");
        return NULL;
    }

    // Return
    return Py_BuildValue("s", ttypeString.c_str());
  }
  else
    return NULL;
}

/**
 * Python function for: host
 */
static PyObject *
robot_host(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    // Get host
    std::string host = robot->Host();
    // Return
    return Py_BuildValue("s", host.c_str());
  }
  else
    return NULL;
}

/**
 * Python function for: name
 */
static PyObject *
robot_name(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    // Get host
    std::string name = robot->Name();
    // Return
    return Py_BuildValue("s", name.c_str());
  }
  else
    return NULL;
}

/**
 * Python function for: launch
 */
static PyObject *
robot_launch(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    // Launch
    robot->Launch();
    // Return
    Py_RETURN_NONE;
  }
  else
    return NULL;
}

/**
 * Python function for: dock
 */
static PyObject *
robot_dock(PyObject *, PyObject *args)
{
  char* target;
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "ss", &robot_addr, &target))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    // Launch
    bool res = robot->Dock(std::string(target));
    // Return
    return Py_BuildValue("b", res);
  }
  else
    return NULL;
}

/**
 * Python function for: is docked
 */
static PyObject *
robot_is_docked(PyObject *, PyObject *args)
{
  char* robot_addr;
  if(!PyArg_ParseTuple(args, "s", &robot_addr))
    return NULL;

  RobotPlugin* robot = get_robot_pointer(std::string(robot_addr));
  if(robot)
  {
    // Launch
    bool docked = robot->IsDocked();
    // Return
    return Py_BuildValue("b", docked);
  }
  else
    return NULL;
}

/**
 * Python function for: print gazebo logging messages.
 */
static PyObject *
robot_gzmsg(PyObject *, PyObject *args)
{
  char *message;

  if(!PyArg_ParseTuple(args, "s", &message))
    return NULL;

  // Print
  gzmsg << message << std::endl;
  Py_RETURN_NONE;
}

/**
 * Python function for: print gazebo logging messages.
 */
static PyObject *
robot_gzerr(PyObject *, PyObject *args)
{
  char *message;

  if(!PyArg_ParseTuple(args, "s", &message))
    return NULL;

  // Print
  gzerr << message << std::endl;
  Py_RETURN_NONE;
}

/**
 * Python function for: print gazebo logging messages.
 */
static PyObject *
robot_gzlog(PyObject *, PyObject *args)
{
  char *message;

  if(!PyArg_ParseTuple(args, "s", &message))
    return NULL;

  // Print
  gzlog << message << std::endl;
  Py_RETURN_NONE;
}

/**
 * Python methods to call c++.
 */
PyMethodDef EmbMethods[] = {
        {"bind",                 robot_bind,                 METH_VARARGS, "Bind."},
        {"set_linear_velocity",  robot_set_linear_velocity,  METH_VARARGS, "Linear velocity."},
        {"set_angular_velocity", robot_set_angular_velocity, METH_VARARGS, "Angular velocity."},
        {"send_to",              robot_send_to,              METH_VARARGS, "Send message to."},
        {"neighbors",            robot_neighbors,            METH_VARARGS, "Neighbors."},
        {"pose",                 robot_pose,                 METH_VARARGS, "Robot pose using GPS."},
        {"imu",                  robot_imu,                  METH_VARARGS, "Robot IMU."},
        {"bearing",              robot_bearing,              METH_VARARGS, "Robot bearing."},
        {"search_area",          robot_search_area,          METH_VARARGS, "Search area for GPS."},
        {"image",                robot_image,                METH_VARARGS, "Logic camera."},
        {"type",                 robot_type,                 METH_VARARGS, "Vehicle type."},
        {"host",                 robot_host,                 METH_VARARGS, "Vehicle host."},
        {"name",                 robot_name,                 METH_VARARGS, "Vehicle name."},
        {"launch",               robot_launch,               METH_VARARGS, "Launch vehicle."},
        {"dock",                 robot_dock,                 METH_VARARGS, "Dock vehicle."},
        {"is_docked",            robot_is_docked,            METH_VARARGS, "Is vehicle docked."},
        {"boo_pose",             robot_boo_pose,             METH_VARARGS, "BOO pose."},
        {"terrain_type",         robot_terrain_type,         METH_VARARGS, "Terrain type."},
        {"lost_person_dir",      robot_lost_person_dir,      METH_VARARGS, "Lost person direction."},
        {"gzmsg",                robot_gzmsg,                METH_VARARGS, "Print gazebo message."},
        {"gzerr",                robot_gzerr,                METH_VARARGS, "Print gazebo error."},
        {"gzlog",                robot_gzlog,                METH_VARARGS, "Gazebo log."},
        {NULL, NULL, 0, NULL}
};
