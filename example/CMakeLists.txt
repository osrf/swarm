cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#################################################
# Find Gazebo.
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#################################################
# Find the Swarm library.
find_package(swarm QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SWARM_CXX_FLAGS}")
include_directories(${SWARM_INCLUDE_DIRS})
link_directories(${SWARM_LIBRARY_DIRS})

########################################
# Find Boost, if not specified manually
include(FindBoost)
find_package(Boost REQUIRED program_options)

if (NOT Boost_FOUND)
  BUILD_ERROR ("Boost not found. Please install program_options.")
endif()

#################################################
# Generate the team's plugin for controlling a robot.
add_library(TeamControllerPlugin SHARED TeamControllerPlugin.cc)
target_link_libraries(TeamControllerPlugin ${SWARM_LIBRARIES})

#################################################
# Install the library so that it can be found at run-time by Gazebo.
# GNUInstallDirst defines CMAKE_INSTALL_LIBDIR, among other things.
include(GNUInstallDirs)
install(TARGETS TeamControllerPlugin DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib)

#################################################
# Generate a sample program that shows how to parse Swarm log files.
add_executable(swarmlog swarmlog.cc)
target_link_libraries(swarmlog ${SWARM_LIBRARIES} ${PROTOBUF_LIBRARY}
                      ${Boost_LIBRARIES})
