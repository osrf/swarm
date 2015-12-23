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

RobotPlugin *tcplugins[1000];


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
    robot->Bind(&RobotPlugin::OnDataReceivedPython,
                robot, std::string(addr), port);
    Py_RETURN_NONE;
  }
  else
    return NULL;
}

/**
 * Python function for: Set linear velocity
 */
static PyObject *
robot_set_linear_velocity(PyObject *, PyObject *args) {
  int robot_id;
  float x, y, z;
  PyArg_ParseTuple(args, "ifff", &robot_id, &x, &y, &z);

  // Send to the right controller.
  tcplugins[robot_id]->SetLinearVelocity(ignition::math::Vector3d(x, y, z));
  return Py_BuildValue("i", robot_id);
}

/**
 * Python function for: Set angular velocity
 */
static PyObject *
robot_set_angular_velocity(PyObject *, PyObject *args) {
  int robot_id;
  float x, y, z;
  PyArg_ParseTuple(args, "ifff", &robot_id, &x, &y, &z);

  // Send to the right controller.
  tcplugins[robot_id]->SetAngularVelocity(ignition::math::Vector3d(x, y, z));
  return Py_BuildValue("i", robot_id);
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
robot_pose(PyObject *, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);

  // Get pose and altitude.
  double latitude, longitude, altitude;
  tcplugins[robot_id]->Pose(latitude, longitude, altitude);

  PyObject *pArgs = PyTuple_New(3);
  PyTuple_SetItem(pArgs, 0, Py_BuildValue("d", latitude));
  PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", longitude));
  PyTuple_SetItem(pArgs, 2, Py_BuildValue("d", altitude));


  return pArgs;
}

/**
 * Python function for: ask for GPS localization.
 */
static PyObject *
robot_gazebo_pose(PyObject *, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);

  // For localization MODEL Type
  std::string model_type = "rotor_";

  switch (tcplugins[robot_id]->Type()) {
    case RobotPlugin::VehicleType::GROUND: {  // GROUND
      model_type = "ground_";
      break;
    }
    case  RobotPlugin::VehicleType::ROTOR: {   //ROTOR
      model_type = "rotor_";
      break;
    }
    case  RobotPlugin::VehicleType::FIXED_WING: {   //ROTOR
      model_type = "fixed_wing_";
      break;
    }
    default: {
      gzerr << "Unknown vehicle type[" << tcplugins[robot_id]->Type() << "]\n";
      break;
    }
  };


  std::string model_id = model_type + std::to_string(robot_id);

  gazebo::physics::ModelPtr gazebo_model = gazebo::physics::get_world()->GetModel(model_id);
  double rx = gazebo_model->GetWorldPose().pos.x;
  double ry = gazebo_model->GetWorldPose().pos.y;
  double rz = gazebo_model->GetWorldPose().pos.z;
//
//
  double rrx = gazebo_model->GetWorldPose().rot.GetAsEuler().x;
  double rry = gazebo_model->GetWorldPose().rot.GetAsEuler().y;
  double rrz = gazebo_model->GetWorldPose().rot.GetAsEuler().z;
//
//
  PyObject *pArgs = PyTuple_New(6);
  PyTuple_SetItem(pArgs, 0, Py_BuildValue("d", rx));
  PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", ry));
  PyTuple_SetItem(pArgs, 2, Py_BuildValue("d", rz));
  PyTuple_SetItem(pArgs, 3, Py_BuildValue("d", rrx));
  PyTuple_SetItem(pArgs, 4, Py_BuildValue("d", rry));
  PyTuple_SetItem(pArgs, 5, Py_BuildValue("d", rrz));


  return pArgs;
}

/**
 * Python function for: ask for IMU.
 */
static PyObject *
robot_imu(PyObject *, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);


  // Get IMU information.
  ignition::math::Vector3d linVel, angVel;
  ignition::math::Quaterniond orient;
  tcplugins[robot_id]->Imu(linVel, angVel, orient);

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


/**
 * Python function for: ask for IMU.
 */
static PyObject *
robot_bearing(PyObject *, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);


  // Get IMU information.
  ignition::math::Angle bearing;
  tcplugins[robot_id]->Bearing(bearing);

  // Return
  return Py_BuildValue("f", bearing.Radian());
}


/**
 * Python function for: SearchArea function.
 */
static PyObject *
robot_search_area(PyObject *, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);

  // Get pose and altitude.
  double minLatitude, maxLatitude, minLongitude, maxLongitude;
  tcplugins[robot_id]->SearchArea(minLatitude, maxLatitude, minLongitude, maxLongitude);

  PyObject *pArgs = PyTuple_New(4);
  PyTuple_SetItem(pArgs, 0, Py_BuildValue("d", minLatitude));
  PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", maxLatitude));
  PyTuple_SetItem(pArgs, 2, Py_BuildValue("d", minLongitude));
  PyTuple_SetItem(pArgs, 3, Py_BuildValue("d", maxLongitude));

  return pArgs;
}


/**
 * Python function for: SearchArea function.
 */
static PyObject *
robot_camera(PyObject *, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);

  // Get the camera information
  ImageData img;
  if (tcplugins[robot_id]->Image(img)) {
    PyObject *camera_locs = PyTuple_New(img.objects.size());
    int i = 0;
    for (auto const obj : img.objects) {
      PyObject *camera_obj = PyTuple_New(1);
      PyTuple_SetItem(camera_obj, 0, Py_BuildValue("s", obj.first.c_str()));
      // TODO obtain pose
      //double x, y, z, p, r, ya;
//      obj.second.Pose3(x, y, z, p, r, ya);
//      PyTuple_SetItem(camera_obj, 1, Py_BuildValue("d", x));
//      PyTuple_SetItem(camera_obj, 2, Py_BuildValue("d", y));
//      PyTuple_SetItem(camera_obj, 3, Py_BuildValue("d", z));
//      PyTuple_SetItem(camera_obj, 4, Py_BuildValue("d", p));
//      PyTuple_SetItem(camera_obj, 5, Py_BuildValue("d", r));
//      PyTuple_SetItem(camera_obj, 6, Py_BuildValue("d", ya));
//

      PyTuple_SetItem(camera_locs, i++, camera_obj);
    }
    return camera_locs;
  }


  return Py_BuildValue("i", -1);
}


/**
 * Python function for: print gazebo logging messages.
 */
static PyObject *
robot_gzmsg(PyObject *, PyObject *args) {
  int robot_id;
  char *message;

  // Get arguments
  PyArg_ParseTuple(args, "is", &robot_id, &message);

  // Print
  gzmsg << message << "\n";
  return Py_BuildValue("i", -1);
}

/**
 * Python function for: print gazebo logging messages.
 */
static PyObject *
robot_gzerr(PyObject *, PyObject *args) {
  int robot_id;
  char *message;

  // Get arguments
  PyArg_ParseTuple(args, "is", &robot_id, &message);

  // Print
  gzerr << message << "\n";
  return Py_BuildValue("i", -1);
}

/**
 * Python function for: print gazebo logging messages.
 */
static PyObject *
robot_gzlog(PyObject *, PyObject *args) {
  int robot_id;
  char *message;

  // Get arguments
  PyArg_ParseTuple(args, "is", &robot_id, &message);

  // Print
  gzlog << message << "\n";
  return Py_BuildValue("i", -1);
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
        {"camera",               robot_camera,               METH_VARARGS, "Logic camera."},
        {"gazebo_pose",          robot_gazebo_pose,          METH_VARARGS, "Get pose from gazebo."},
        {"gzmsg",                robot_gzmsg,                METH_VARARGS, "Print gazebo message."},
        {"gzerr",                robot_gzerr,                METH_VARARGS, "Print gazebo error."},
        {"gzlog",                robot_gzlog,                METH_VARARGS, "Gazebo log."},
        {NULL, NULL, 0, NULL}
};
