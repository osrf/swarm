include (${project_cmake_dir}/Utils.cmake)

set (gazebo_visualize_sources 
)

set (common_sources ${common_sources}
  Common.cc
  Broker.cc
  Logger.cc
)

set (broker_plugin_sources
  BrokerPlugin.cc
  CommsModel.cc
  VisibilityTable.cc
)

set (robot_plugin_sources
  RobotPlugin.cc
)
if (PYTHONLIBS_FOUND)
  set (robot_plugin_sources ${robot_plugin_sources} python_api.cc)
endif()

set (lost_person_plugin_sources
  LostPersonPlugin.cc
)

set (lost_person_controller_sources
  LostPersonControllerPlugin.cc
)

set (boo_plugin_sources
  BooPlugin.cc
)

set (gtest_sources
  Broker_TEST.cc
  BrokerPlugin_TEST.cc
  Logger_TEST.cc
  RobotPlugin_TEST.cc
)

set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES
                            GENERATED TRUE)
include_directories(${CMAKE_BINARY_DIR}/)
include_directories(${CMAKE_BINARY_DIR}/test/)

ign_build_tests(${gtest_sources})

# Create the libSwarmBrokerPlugin.so library.
ign_add_library(${PROJECT_LIB_BROKER_NAME}
                ${broker_plugin_sources}
                ${common_sources})
target_link_libraries(${PROJECT_LIB_BROKER_NAME}
                      ${PROJECT_LIB_MSGS_NAME}
                      ${PROTOBUF_LIBRARY}
                      ${IGNITION-TRANSPORT_LIBRARIES})
ign_install_library(${PROJECT_LIB_BROKER_NAME})

# Create the libSwarmRobotPlugin.so library.
ign_add_library(${PROJECT_LIB_ROBOT_NAME}
                ${robot_plugin_sources}
                ${common_sources})
set(_libs_tmp ${PROJECT_LIB_MSGS_NAME}
              ${PROTOBUF_LIBRARY}
              ${IGNITION-TRANSPORT_LIBRARIES})
if (PYTHONLIBS_FOUND)
  set(_libs_tmp ${_libs_tmp} ${PYTHON_LIBRARIES})
  target_include_directories(${PROJECT_LIB_ROBOT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
  target_compile_definitions(${PROJECT_LIB_ROBOT_NAME} PRIVATE -DSWARM_PYTHON_API)
endif()
target_link_libraries(${PROJECT_LIB_ROBOT_NAME} ${_libs_tmp})
ign_install_library(${PROJECT_LIB_ROBOT_NAME})

# Create the libSwarmBooPlugin.so library.
ign_add_library(${PROJECT_LIB_BOO_NAME}
                ${boo_plugin_sources}
                ${common_sources})
target_link_libraries(${PROJECT_LIB_BOO_NAME}
                      ${PROJECT_LIB_ROBOT_NAME}
                      ${PROJECT_LIB_MSGS_NAME}
                      ${PROTOBUF_LIBRARY}
                      ${IGNITION-TRANSPORT_LIBRARIES})
ign_install_library(${PROJECT_LIB_BOO_NAME})

# Create the libSwarmLostPersonPlugin.so library.
ign_add_library(${PROJECT_LIB_LOST_PERSON_NAME} ${lost_person_plugin_sources})
target_link_libraries(${PROJECT_LIB_LOST_PERSON_NAME}
                      ${PROTOBUF_LIBRARY}
                      ${IGNITION-TRANSPORT_LIBRARIES})
ign_install_library(${PROJECT_LIB_LOST_PERSON_NAME})

# Create the libLostPersonControllerPlugin.so library.
ign_add_library(${PROJECT_LIB_LOST_PERSON_CONTROLLER_NAME} ${lost_person_controller_sources})
target_link_libraries(${PROJECT_LIB_LOST_PERSON_CONTROLLER_NAME}
                      ${PROJECT_LIB_LOST_PERSON_NAME}
                      ${PROTOBUF_LIBRARY}
                      ${IGNITION-TRANSPORT_LIBRARIES})
ign_install_library(${PROJECT_LIB_LOST_PERSON_CONTROLLER_NAME})

ign_add_library(VisibilityPlugin VisibilityPlugin.cc VisibilityTable.cc)
target_link_libraries(VisibilityPlugin 
  ${PROJECT_LIB_MSGS_NAME}
  ${PROTOBUF_LIBRARY}
  ${IGNITION-TRANSPORT_LIBRARIES})
ign_install_library(VisibilityPlugin)

# ign_add_library(GazeboVisualizePlugin GazeboVisualizePlugin.cc)
# target_link_libraries(GazeboVisualizePlugin 
#   ${PROJECT_LIB_MSGS_NAME}
#   ${PROTOBUF_LIBRARY}
#   ${IGNITION-TRANSPORT_LIBRARIES})
# ign_install_library(GazeboVisualizePlugin )
