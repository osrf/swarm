include_directories(${CMAKE_BINARY_DIR}/)

add_subdirectory(report)

#################################################
# Generate a tool for introspecting Swarm log files.
add_executable(swarmlog swarmlog.cc)
target_link_libraries(swarmlog ${SWARM_LIBRARIES} ${PROTOBUF_LIBRARY}
                      ${Boost_LIBRARIES}
                      ${PROJECT_LIB_MSGS_NAME})

install (PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/swarmlog ${CMAKE_CURRENT_SOURCE_DIR}/swarmlog_to_csv_comms.py  DESTINATION ${BIN_INSTALL_DIR})
