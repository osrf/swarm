set (terrains
  swarm_terrain_257_random.png
  swarm_terrain_camp_roberts.png
)

if (${gazebo_VERSION_MAJOR} GREATER 6)
  install(FILES ${terrains}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}/media/materials/textures)
elseif()
  install(FILES ${terrains}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}/media/materials/textures)
endif()
