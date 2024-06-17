# Install script for directory: /home/danielchoate/colmap/src/colmap

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/danielchoate/colmap/build/src/colmap/controllers/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/estimators/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/exe/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/feature/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/geometry/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/image/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/math/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/mvs/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/optim/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/retrieval/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/scene/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/sensor/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/sfm/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/tools/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/util/cmake_install.cmake")
  include("/home/danielchoate/colmap/build/src/colmap/ui/cmake_install.cmake")

endif()

