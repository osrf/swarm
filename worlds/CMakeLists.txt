set (files
  swarm_empty.world
)

install(FILES ${files}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds)
