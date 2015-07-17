include (${project_cmake_dir}/Utils.cmake)

set (headers
  SwarmPlugin.hh
)

ign_install_includes(${IGN_PROJECT_NAME} ${headers})
