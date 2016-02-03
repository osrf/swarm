add_subdirectory(terrains)

# Non-erb world files
set (world_files
  swarm_vis.world
)

# Non-world erb files
set (other_erb_files
  boo.sdf.erb
  common.sdf.erb
  fixedwing_vehicle.sdf.erb
  ground_vehicle.sdf.erb
  lost_person_simple.sdf.erb
  rotor_vehicle.sdf.erb
  terrain_camp_roberts.sdf.erb
  terrain_model_1.sdf.erb
  terrain_test_model.sdf.erb
  template.world.erb
  vegetation_01.sdf.erb
  vegetation_camp_roberts_low.sdf.erb
  vegetation_camp_roberts_med.sdf.erb
  vegetation_camp_roberts_high.sdf.erb
  vegetation_flat_low.sdf.erb
  vegetation_flat_med.sdf.erb
  vegetation_flat_high.sdf.erb
  buildings_camp_roberts.sdf.erb
  buildings_flat.sdf.erb
)

# World files that need to be processed with erb
set (world_erb_files
  fixed_simple_36.world.erb
  ground_easy_4.world.erb
  ground_rotor.world.erb
  ground_simple_2.world.erb
  ground_simple_36.world.erb
  lost_person.world.erb
  rotor_simple_36.world.erb
  swarm_empty.world.erb
  terrain_test.world.erb
  terrain_test_roberts.world.erb
)

set (master_erb_file master.erb)

# Number of robots per type for each world file.
set (_robot_members
  10
  25
  50
  100
  200
  400
  800
)

# World files that are automatically generated based on master.erb
set (generated_master_world_files
  complete.world
  final_01_02_low.world
  final_01_02_low_flat.world
  final_01_02_low_ground.world
  final_01_02_medium.world
  final_01_02_medium_flat.world
  final_01_02_medium_ground.world
  final_01_02_high.world
  final_01_02_high_flat.world
  final_01_02_high_ground.world
  final_01_10_low.world
  final_01_10_low_flat.world
  final_01_10_low_ground.world
  final_01_10_medium.world
  final_01_10_medium_flat.world
  final_01_10_medium_ground.world
  final_01_10_high.world
  final_01_10_high_flat.world
  final_01_10_high_ground.world
  final_01_20_low.world
  final_01_20_low_flat.world
  final_01_20_low_ground.world
  final_01_20_medium.world
  final_01_20_medium_flat.world
  final_01_20_medium_ground.world
  final_01_20_high.world
  final_01_20_high_flat.world
  final_01_20_high_ground.world
)

# Process the erb files
foreach(_erb ${world_erb_files})
  string(REGEX REPLACE ".world.erb" ".world" _world ${_erb})
  set(_world ${CMAKE_CURRENT_BINARY_DIR}/${_world})
  add_custom_command(OUTPUT ${_world}
                     COMMAND ${ERB_EXE_PATH} ${_erb} > ${_world}
                     DEPENDS ${_erb} ${other_erb_files}
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND world_files ${_world})
endforeach()
add_custom_target(erb_generation ALL DEPENDS ${world_files})

# Process the master.erb
foreach(_world ${generated_master_world_files})
  foreach(_num_robots ${_robot_members})
    string(REGEX REPLACE ".world" "_${_num_robots}.world" world ${_world})
    MESSAGE(STATUS "Processing world: ${world}")
    set(_target_world ${CMAKE_CURRENT_BINARY_DIR}/${world})
    add_custom_command(OUTPUT ${_target_world}
                       COMMAND erb_gazebo_world_file=${_world} erb_num_robots=${_num_robots} ${ERB_EXE_PATH} ${master_erb_file}  > ${_target_world}
                       DEPENDS ${master_erb_file} ${other_erb_files}
                       WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    list(APPEND world_files ${_target_world})
  endforeach()
endforeach()
add_custom_target(erb_master_generation ALL DEPENDS ${world_files})

# Install all the world files
if (${gazebo_VERSION_MAJOR} GREATER 6)
  install(FILES ${world_files}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}/worlds)
else()
  install(FILES ${world_files}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}/worlds)
endif()
