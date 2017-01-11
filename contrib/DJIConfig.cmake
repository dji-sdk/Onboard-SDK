#Detecting target architecture to decide which precompiled library to link against
if((CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64") AND (CMAKE_SIZEOF_VOID_P EQUAL 8))
  set(TARGET_ARCH "x86_64")
elseif((CMAKE_SYSTEM_PROCESSOR MATCHES "i386|i686|x86|AMD64") AND (CMAKE_SIZEOF_VOID_P EQUAL 4))
  set(TARGET_ARCH "x86")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm*")
  set(TARGET_ARCH "arm")
endif()

#Detect platform - from https://gist.github.com/CoolerVoid/1781717
if(CMAKE_SYSTEM_NAME MATCHES Linux)
  if(${TARGET_ARCH} MATCHES "arm" ) 
    set(RELEASE_FILE /etc/os-release)
    set(VERSION_FIELD VERSION_ID)
    set(COMPILER "-std=c++0x")
  else()
    set(RELEASE_FILE "/etc/lsb-release")
    set(VERSION_FIELD DISTRIB_RELEASE)
    set(COMPILER "-std=c++11")
  endif()
endif()

if(EXISTS ${RELEASE_FILE})
  #Detect platform - from https://gist.github.com/CoolerVoid/1781717
  EXECUTE_PROCESS(
    COMMAND cat ${RELEASE_FILE}
    COMMAND grep ${VERSION_FIELD}
    COMMAND awk -F= "{ print $2 }"
    COMMAND tr "\n" " "
    COMMAND sed "s/ //"
    OUTPUT_VARIABLE VER)
else()
  set(VER UNKNOWN)
endif()

# External DJI Modules: Precision Missions 1.0.2, Collision Avoidance and Pointcloud2LAS
# BETA supported only on Ubuntu 16.04. 
if( ${VER} STREQUAL "16.04")
  set(DISTRO_VERSION 1604)
elseif(${VER} STREQUAL "14.04")
  set(DISTRO_VERSION 1404)
elseif(${VER} STREQUAL "18")
  set(DISTRO_VERSION 1604)
else()
  set(DISTRO_VERSION UNSUPPORTED)
endif()

