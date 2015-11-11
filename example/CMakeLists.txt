cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#################################################
# Find the Swarm library.
find_package(swarm QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SWARM_CXX_FLAGS}")
include_directories(${SWARM_INCLUDE_DIRS} ${GAZEBO_PROTO_INCLUDE_DIRS})
link_directories(${SWARM_LIBRARY_DIRS})

#################################################
# Generate the terrain test plugin.
add_library(TerrainTestPlugin SHARED TerrainTestPlugin.cc)
target_link_libraries(TerrainTestPlugin ${SWARM_LIBRARIES})

#################################################
# Generate the team's plugin for controlling a robot.
add_library(TeamControllerPlugin SHARED TeamControllerPlugin.cc)
target_link_libraries(TeamControllerPlugin ${SWARM_LIBRARIES})

#################################################
# Generate the plugin for controlling a lost person.
add_library(LostPersonControllerPlugin SHARED LostPersonControllerPlugin.cc)
target_link_libraries(LostPersonControllerPlugin ${SWARM_LIBRARIES})

#################################################
# Install the library so that it can be found at run-time by Gazebo.
# GNUInstallDirst defines CMAKE_INSTALL_LIBDIR, among other things.
include(GNUInstallDirs)
install(TARGETS TeamControllerPlugin DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib)
install(TARGETS LostPersonControllerPlugin DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib)
install(TARGETS TerrainTestPlugin DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib)
