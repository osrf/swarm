include (${project_cmake_dir}/Utils.cmake)

set (headers
  BooPlugin.hh
  Broker.hh
  BrokerPlugin.hh
  Common.hh
  CommsModel.hh
  Helpers.hh
  Logger.hh
  LogParser.hh
  LostPersonControllerPlugin.hh
  LostPersonPlugin.hh
  RobotPlugin.hh
  SwarmTypes.hh
)

#################################################
### Do not change the following
set (ign_headers "" CACHE INTERNAL "Ignition headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(ign_headers
    "Swarm headers" "#include <${IGN_PROJECT_NAME}/${hdr}>\n")
endforeach()

ign_install_includes("" ${headers})
