add_subdirectory(terrains)

# Non-erb world files
set (world_files
)

# Non-world erb files
set (other_erb_files
  boo.sdf.erb
  common.sdf.erb
  fixedwing_vehicle.sdf.erb
  ground_vehicle.sdf.erb
  lost_person_simple.sdf.erb
  rotor_vehicle.sdf.erb
  terrain_model_1.sdf.erb
  template.world.erb
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
)

set (master_erb_file master.erb)

# World files that are automatically generated based on master.erb
set (generated_master_world_files
  complete.world
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
  set(_target_world ${CMAKE_CURRENT_BINARY_DIR}/${_world})
  add_custom_command(OUTPUT ${_target_world}
                     COMMAND erb_gazebo_world_file=${_world} ${ERB_EXE_PATH} ${master_erb_file}  > ${_target_world}
                     DEPENDS ${master_erb_file} ${other_erb_files}
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND world_files ${_target_world})
endforeach()
add_custom_target(erb_master_generation ALL DEPENDS ${world_files})

# Install all the world files
install(FILES ${world_files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}/worlds)
