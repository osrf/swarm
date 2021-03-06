# Non-erb world files
set (world_files
)

# World files that need to be processed with erb
set (world_erb_files
  battery_00.world.erb
  battery_01.world.erb
  battery_02.world.erb
  boo_00.world.erb
  boo_01.world.erb
  boo_02.world.erb
  boo_03.world.erb
  boo_04.world.erb
  boo_05.world.erb
  boo_06.world.erb
  boo_07.world.erb
  boo_08.world.erb
  boo_09.world.erb
  comms_00.world.erb
  comms_01.world.erb
  comms_02.world.erb
  comms_03.world.erb
  comms_04.world.erb
  comms_05.world.erb
  comms_06.world.erb
  comms_07.world.erb
  comms_08.world.erb
  comms_09.world.erb
  comms_10.world.erb
  comms_11.world.erb
  comms_12.world.erb
  logical_camera_00.world.erb
  logical_camera_01.world.erb
  logical_camera_02.world.erb
  map_00.world.erb
  map_01.world.erb
  map_02.world.erb
)

# Process the erb files
foreach(_erb ${world_erb_files})
  string(REGEX REPLACE ".world.erb" ".world" _world ${_erb})
  set(_world ${CMAKE_CURRENT_BINARY_DIR}/${_world})
  add_custom_command(OUTPUT ${_world}
                     COMMAND ${ERB_EXE_PATH} ${_erb} > ${_world}
                     DEPENDS ${_erb}
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND world_files ${_world})
endforeach()
add_custom_target(erb_test_generation ALL DEPENDS ${world_files})
