add_subdirectory(terrains)

# Non-erb world files
set (world_files
)

# World files that need to be processed with erb
set (world_erb_files
  fixed_simple_36.world.erb
  ground_simple_2.world.erb
  ground_simple_36.world.erb
  rotor_simple_36.world.erb
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
add_custom_target(erb_generation ALL DEPENDS ${world_files})

# Install all the world files
install(FILES ${world_files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}/worlds)
