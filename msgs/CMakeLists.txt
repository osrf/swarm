set(msgs
  datagram.proto
  log_entry.proto
  log_header.proto
)

add_executable(ignmsgs_out generator/IgnGenerator.cc generator/ign_generator.cc)

target_link_libraries(ignmsgs_out ${PROTOBUF_LIBRARY} ${PROTOBUF_PROTOC_LIBRARY})

if (UNIX)
  target_link_libraries(ignmsgs_out pthread)
endif()


# Need to add .exe to executable in windows
if (WIN32)
   set(ignmsgs_out_binary "ignmsgs_out.exe")
else()
   set(ignmsgs_out_binary "ignmsgs_out")
endif()

set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(FIL ${msgs})
  get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
  get_filename_component(FIL_WE ${FIL} NAME_WE)

  list(APPEND PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc")
  list(APPEND PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h")
  list(APPEND PROTO_PY "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}_pb2.py")
  list(APPEND PROTO_RB "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.rb")

  add_custom_command(
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc"
      "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h"
      "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}_pb2.py"
      "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.rb"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --plugin=protoc-gen-ignmsgs=${CMAKE_CURRENT_BINARY_DIR}/${ignmsgs_out_binary} --cpp_out=dllexport_decl=IGNITION_MSGS_VISIBLE:${CMAKE_CURRENT_BINARY_DIR} --python_out=${CMAKE_CURRENT_BINARY_DIR} --ruby_out=${CMAKE_CURRENT_BINARY_DIR} --ignmsgs_out ${CMAKE_CURRENT_BINARY_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR} --proto_path ${GAZEBO_PROTO_PATH} ${ABS_FIL}
    DEPENDS ${ABS_FIL} ignmsgs_out
    COMMENT "Running C++, Python, and Ruby protocol buffer compiler on ${FIL}"
    VERBATIM )
endforeach()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_IGNITION_MSGS")

set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)
ign_add_library(${PROJECT_LIB_MSGS_NAME} ${PROTO_SRCS})
if (UNIX)
  # Need to add default visibility to ignition-transport-msgs
  get_target_property(current_property ${PROJECT_LIB_MSGS_NAME} COMPILE_FLAGS)
  if(NOT current_property) # property non-existent or empty
    set_target_properties(${PROJECT_LIB_MSGS_NAME} PROPERTIES GENERATED TRUE COMPILE_FLAGS "-fvisibility=default")
  else()
    set_target_properties(${PROJECT_LIB_MSGS_NAME} PROPERTIES COMPILE_FLAGS "${current_property} -fvisibility=default")
  endif()
endif()

target_link_libraries(${PROJECT_LIB_MSGS_NAME} ${PROTOBUF_LIBRARY} ${PROTOBUF_PROTOC_LIBRARY} ${GAZEBO_PROTO_LIBRARIES})

ign_install_library(${PROJECT_LIB_MSGS_NAME})
add_dependencies(${PROJECT_LIB_MSGS_NAME} ignmsgs_out)

# Install the generated protobuf messages.
install(FILES ${PROTO_HDRS} DESTINATION ${INCLUDE_INSTALL_DIR}/msgs)

# Also compile the underlying Gazebo messages to produce Python code
if(PYTHON_INSTALL_DIR)
  file(GLOB _gazebo_protos "${GAZEBO_PROTO_PATH}/*.proto")
  foreach(FIL ${_gazebo_protos})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
  
    list(APPEND PROTO_PY "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}_pb2.py")
    list(APPEND PROTO_RB "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.rb")
 
    add_custom_command(
      OUTPUT
        "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}_pb2.py"
        "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.rb"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --python_out=${CMAKE_CURRENT_BINARY_DIR} --ruby_out=${CMAKE_CURRENT_BINARY_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR} --proto_path ${GAZEBO_PROTO_PATH} ${ABS_FIL}
      DEPENDS ${ABS_FIL}
      COMMENT "Running Python and Ruby protocol buffer compiler on ${FIL}"
      VERBATIM )
  endforeach()
  add_custom_target(gazebo_proto_pb ALL DEPENDS ${PROTO_PY} ${PROTO_RB})

  # Create an __init__.py file to import all the messages
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "# Auto-generated")
  foreach(_pyfile ${PROTO_PY})
    get_filename_component(_module ${_pyfile} NAME_WE)
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "import ${_module}\n")
  endforeach()

  # Install the resulting files
  message(STATUS "Installing Python message code to ${PYTHON_INSTALL_DIR}/swarm")

  install(FILES ${PROTO_PY} ${CMAKE_CURRENT_BINARY_DIR}/__init__.py DESTINATION ${PYTHON_INSTALL_DIR}/swarm)

  message(STATUS "Installing Ruby message code to ${RUBY_INSTALL_DIR}/swarm")
  install(FILES ${PROTO_RB} DESTINATION ${RUBY_INSTALL_DIR}/swarm)
endif()

