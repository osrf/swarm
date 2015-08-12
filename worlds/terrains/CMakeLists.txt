set (terrains
  swarm_terrain_257_random.png
)

install(FILES ${terrains}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}/media/materials/textures)
