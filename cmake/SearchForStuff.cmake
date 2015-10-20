include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

#################################################
# Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#################################################
# Ignition math
find_package(ignition-math2 QUIET REQUIRED)
include_directories(${IGNITION-MATH_INCLUDE_DIRS})
link_directories(${IGNITION-MATH_LIBRARY_DIRS})

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()

# gtest apparently needs boost::system. Boost_LIBRARIES is added to the link
# line for tests in TestUtils.cmake.
include(FindBoost)
find_package(Boost REQUIRED system thread filesystem program_options)

# We need erb to process the .world.erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

#################################################
# Determine Python version, for installation of
# Python code.  Not the prettiest technique
# (e.g., it's very Linux specific).
execute_process(COMMAND python -c "import sys; print(\"%d.%d\"%(sys.version_info.major,sys.version_info.minor))"
                RESULT_VARIABLE _python_result
                OUTPUT_VARIABLE _python_stdout
                OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT ${_python_result})
  SET(PYTHON_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib/python${_python_stdout}/dist-packages)
else()
  SET(PYTHON_INSTALL_DIR "")
endif()
