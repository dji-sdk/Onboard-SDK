cmake_minimum_required(VERSION 2.8)
project(djiosdk-linux-sample)
include(ExternalProject)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

if (COVERAGE)
  set(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage -fPIC")
  set(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
endif ()

include_directories(../../../osdk-core/inc)
include_directories(../../../osdk-wrapper/inc)
include_directories(../../../platform/linux/inc)
include_directories(inc)
if (LIDAR_LOGGING)
  include_directories(../../../sensor-integration/velodyne_vlp16puck_lite_API_example/vlp16lidar-driver/inc)
endif ()

# Local Mission Planning Includes
include_directories(../../../tools/localMissionPlanning/inc)
include_directories(../../../tools/localMissionPlanning/inc/gps2local)
include_directories(../../../tools/localMissionPlanning/inc/trajectory-generator)
include_directories(../../../tools/localMissionPlanning/inc/trajectory-recorder)
include_directories(../../../tools/localMissionPlanning/inc/trajectory-follower)
include_directories(../../../tools/localMissionPlanning/inc/se3controller)
include_directories(../../../tools/localMissionPlanning/inc/twopoint-trajectory-generator)
include_directories(../../../tools/localMissionPlanning/inc/trajectory-infrastructure)

# Local Mission Planning libraries
include_directories(../../../tools/localMissionPlanning/dependencies)
include_directories(../../../tools/localMissionPlanning/dependencies/csv)

if(NOT TARGET rapidjson)
  ExternalProject_Add(
          rapidjson
          # Disable update step
          UPDATE_COMMAND ""
          GIT_REPOSITORY https://github.com/miloyip/rapidjson.git
          CMAKE_ARGS -DRAPIDJSON_BUILD_TESTS=OFF
          -DRAPIDJSON_BUILD_DOC=OFF
          -DRAPIDJSON_BUILD_EXAMPLES=OFF
          PREFIX ${CMAKE_CURRENT_BINARY_DIR}
          # Disable install step
          INSTALL_COMMAND ""
          TEST_COMMAND ""
          # UPDATE_DISCONNECTED
  )
endif()
ExternalProject_Get_Property(rapidjson source_dir)
include_directories("${source_dir}/include")

#Detecting target architecture to decide which precompiled library to link against
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  set (X86 TRUE)
else ()
  set (X86 FALSE)
endif ()
#Detect platform - from https://gist.github.com/CoolerVoid/1781717
EXECUTE_PROCESS(
        COMMAND cat /etc/lsb-release
        COMMAND grep DISTRIB_RELEASE
        COMMAND awk -F= "{ print $2 }"
        COMMAND tr "\n" " "
        COMMAND sed "s/ //"
        OUTPUT_VARIABLE LSB_VER
)
if( ${LSB_VER} STREQUAL "16.04")
  if (X86)
    find_library(MISSIONPLAN missionplan HINTS ../tools/localMissionPlanning/lib/x86_64/1604)
  else ()
    message("ARM running ubuntu 16.04 is not supported.")
  endif()
elseif(${LSB_VER} STREQUAL "14.04")
  if (X86)
    message("Warning! Precision Trajectory Features do not work in Ubuntu 14.04 on x86. Please upgrade to Ubuntu 16.04 to use these features. Your program WILL SEGFAULT if you try to run precision trajectories.")
    find_library(MISSIONPLAN missionplan HINTS ../tools/localMissionPlanning/lib/x86_64/1404)
  else ()
    find_library(MISSIONPLAN missionplan HINTS ../tools/localMissionPlanning/lib/arm)
  endif()
endif()

FILE(GLOB SOURCE_FILES inc/*.h src/*.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
if(LIDAR_LOGGING)
target_link_libraries(${PROJECT_NAME} vlp16lidar-driver djiosdk-core djiosdk-platform djiosdk-wrapper ${MISSIONPLAN})
else()
target_link_libraries(${PROJECT_NAME} djiosdk-core djiosdk-platform djiosdk-wrapper ${MISSIONPLAN})
endif()
