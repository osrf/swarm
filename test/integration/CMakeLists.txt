set(TEST_TYPE "INTEGRATION")

set(tests
  battery.cc
)

ign_build_tests(${tests})

add_library(battery_plugin SHARED battery_plugin.cc)
target_link_libraries(battery_plugin
  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_LIB_ROBOT_NAME}
  ${PROJECT_LIB_MSGS_NAME} gtest gtest_main)

