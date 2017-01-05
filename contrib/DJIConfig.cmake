#Detecting target architecture to decide which precompiled library to link against
if((CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64") AND (CMAKE_SIZEOF_VOID_P EQUAL 8))
  set(TARGET_ARCH "x86_64")
elseif((CMAKE_SYSTEM_PROCESSOR MATCHES "i386|i686|x86|AMD64") AND (CMAKE_SIZEOF_VOID_P EQUAL 4))
  set(TARGET_ARCH "x86")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm*")
  set(TARGET_ARCH "arm")
endif()

#Detect platform - from https://gist.github.com/CoolerVoid/1781717
EXECUTE_PROCESS(
  COMMAND cat /etc/lsb-release
  COMMAND grep DISTRIB_DESCRIPTION
  COMMAND awk -F= "{ print $2 }"
  COMMAND tr "\n" " "
  COMMAND sed "s/ //"
  OUTPUT_VARIABLE LSB_VER
  )

if( ${LSB_VER} MATCHES "Ubuntu16.04 LTS")
  set(DISTRO_VERSION 1604)
elseif(${LSB_VER} MATCHES "Ubuntu14.04 LTS")
  set(DISTRO_VERSION 1404)
elseif(${LSB_VER} MATCHES "Linux Mint 18 Sarah")
  set(DISTRO_VERSION 1604)
endif()

