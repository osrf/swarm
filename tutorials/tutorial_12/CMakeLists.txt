cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#################################################
# Find the Swarm library.
find_package(swarm QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SWARM_CXX_FLAGS}")
include_directories(${SWARM_INCLUDE_DIRS})
link_directories(${SWARM_LIBRARY_DIRS})

#################################################
# Generate the team's plugin for controlling the BOO and install the library.
add_library(BooControllerPlugin SHARED BooControllerPlugin.cc)
target_link_libraries(BooControllerPlugin ${SWARM_LIBRARIES})
# GNUInstallDirst defines CMAKE_INSTALL_LIBDIR, among other things.
include(GNUInstallDirs)
install (TARGETS BooControllerPlugin DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib)
