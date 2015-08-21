include (${project_cmake_dir}/Utils.cmake)

set (broker_plugin_sources ${broker_plugin_sources}
  comms/CommsModel.cc
  PARENT_SCOPE
)

set (gtest_sources
)

ign_build_tests(${gtest_sources})
