include (${project_cmake_dir}/Utils.cmake)

set (broker_plugin_sources
  BrokerPlugin.cc
)

set (robot_plugin_sources
  RobotPlugin.cc
)

set (gtest_sources
  BrokerPlugin_TEST.cc
  RobotPlugin_TEST.cc
)

set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES
                            GENERATED TRUE)
include_directories(${CMAKE_BINARY_DIR}/)
include_directories(${CMAKE_BINARY_DIR}/test/)

ign_build_tests(${gtest_sources})

# Create the libSwarmBrokerPlugin.so library.
ign_add_library(${PROJECT_LIB_BROKER_NAME} ${broker_plugin_sources})
target_link_libraries(${PROJECT_LIB_BROKER_NAME}
                      ${PROJECT_LIB_MSGS_NAME}
                      ${PROTOBUF_LIBRARY}
                      ${IGNITION-TRANSPORT_LIBRARIES})
ign_install_library(${PROJECT_LIB_BROKER_NAME})

# Create the libSwarmRobotPlugin.so library.
ign_add_library(${PROJECT_LIB_ROBOT_NAME} ${robot_plugin_sources})
target_link_libraries(${PROJECT_LIB_ROBOT_NAME}
                      ${PROJECT_LIB_MSGS_NAME}
                      ${PROTOBUF_LIBRARY}

                      ${IGNITION-TRANSPORT_LIBRARIES})
ign_install_library(${PROJECT_LIB_ROBOT_NAME})
