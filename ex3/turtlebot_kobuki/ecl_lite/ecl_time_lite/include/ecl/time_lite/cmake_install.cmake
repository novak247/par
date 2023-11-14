# Install script for directory: /home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ecl/time_lite" TYPE FILE FILES
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/config.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/cpu_time.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/date.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/errors.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/functions.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/functions_mac.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/functions_pos.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/functions_rt.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/functions_win.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/macros.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/types.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/types_pos.hpp"
    "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/types_win.hpp"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ecl/time_lite" TYPE FILE FILES "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/config.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/chudoba/work/3rdparty/kobuki_core2/src/ecl_lite/ecl_time_lite/include/ecl/time_lite/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
