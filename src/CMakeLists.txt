include (${project_cmake_dir}/Utils.cmake)

set (sources
  swarm.cc
)

set (gtest_sources
  swarm_TEST.cc
)

include_directories(${CMAKE_BINARY_DIR})

ign_build_tests(${gtest_sources})
if (UNIX)
  ign_add_library(${PROJECT_NAME_LOWER} ${sources})
elseif(WIN32)
  # Probotuf compilation is recommended and configured for a STATIC .lib
  # Not mixing static and shared and just build ignition transport as STATIC
  # on Windows
  add_library(${PROJECT_NAME_LOWER} STATIC ${sources})
endif()
target_link_libraries(${PROJECT_NAME_LOWER} ${IGNITION-TRANSPORT_LIBRARIES}
                      ${PROJECT_MSGS_NAME})

if(WIN32)
  target_link_libraries(${PROJECT_NAME_LOWER} ws2_32 Iphlpapi)
endif()
ign_install_library(${PROJECT_NAME_LOWER})

add_dependencies(${PROJECT_NAME_LOWER}
  gtest gtest_main
)
