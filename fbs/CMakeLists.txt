set (msgs
  gps.fbs
)

set (generated
  gps_generated.h
  PARENT_SCOPE
)

function(compile_flatbuffers_schema_to_cpp SRC_FBS)
  get_filename_component(SRC_FBS_DIR ${SRC_FBS} PATH)
  string(REGEX REPLACE "\\.fbs$" "_generated.h" GEN_HEADER ${SRC_FBS})
  add_custom_command(
    OUTPUT ${GEN_HEADER}
    COMMAND flatc -c --no-includes --gen-mutable -o "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/${SRC_FBS}"
    DEPENDS flatc)
endfunction()

foreach(FB ${msgs})
  message(STATUS "Generating cpp code for: ${FB}")
  compile_flatbuffers_schema_to_cpp(${FB})
endforeach()

set_source_files_properties(${generated} PROPERTIES GENERATED TRUE)

message(STATUS "All generated flatbuffers: " ${generated})
# Install the generated protobuf messages.
install(FILES ${generated} DESTINATION ${INCLUDE_INSTALL_DIR}/fbs)