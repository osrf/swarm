cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#################################################
# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#################################################
# Find the Swarm library
find_package(swarm QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SWARM_CXX_FLAGS}")
include_directories(${SWARM_INCLUDE_DIRS})
link_directories(${SWARM_LIBRARY_DIRS})

set (EXAMPLES_LIBS ${IGNITION-TRANSPORT_LIBRARIES}
                   ${ZeroMQ_LIBRARIES}
                   ${PROTOBUF_LIBRARY})

add_library(SwarmRobotPlugin SHARED SwarmRobotPlugin.cc)
target_link_libraries(SwarmRobotPlugin ${SWARM_LIBRARIES})
