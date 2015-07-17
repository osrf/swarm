# Swarm API

** SWARM classes and functions.**

  [https://bitbucket.org/osrf/swarm(https://bitbucket.org/osrf/swarm)

## Dependencies

The following dependencies are required to compile Swarm from source:

 - cmake
 - mercurial
 - C++ compiler (with C++11 support).

Installation of dependencies:

 - sudo apt-get install build-essential cmake mercurial

## Installation

Standard installation can be performed in UNIX systems using the following
steps:

 - mkdir build/
 - cd build/
 - cmake ..
 - sudo make install

## Uninstallation

To uninstall the software installed with the previous steps:

 - cd build/
 - sudo make uninstall

## Documentation

### Update Matlab documentation

 - Run `make doc`
 - Upload content of `build/doxygen/html/*` to s3.
