# Install script for directory: /home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_config/include/ecl/config

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
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  set(CMAKE_INSTALL_SO_NO_EXE "0")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ecl/config" TYPE FILE FILES "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_config/include/ecl/config/ecl.hpp")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ecl/config" TYPE FILE FILES
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_config/include/ecl/config/char_sign.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_config/include/ecl/config/ecl.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_config/include/ecl/config/ecl_unknown.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_config/include/ecl/config/endianness.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_config/include/ecl/config/macros.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_config/include/ecl/config/portable_types.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_config/include/ecl/config/windows.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_config/include/ecl/config/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
