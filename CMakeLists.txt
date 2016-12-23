cmake_minimum_required(VERSION 2.8)
project(onboardsdk)

set(CMAKE_VERBOSE_MAKEFILE OFF)

# Add module path for <name>.cmake conf files
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/contrib/)

include(${CMAKE_MODULE_PATH}/DJIConfig.cmake)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/libs)

# Set test related flags
option(GTEST "Build with Google Test framework" OFF)

# Allow to build additional modules
option(LIDAR_LOGGING "Build VLP16 Puck LiDAR logging" OFF)

if(NOT ${DISTRO_VERSION} STREQUAL "1604")
  message("Precision missions, collision avoidance and pointcloud to LAS converter BETA works only on Ubuntu 16.04")
else()
  option(USE_PRECISION_MISSIONS "Use DJI precision-missions library" OFF)
  if(USE_PRECISION_MISSIONS)
    add_definitions(-DUSE_PRECISION_MISSIONS)
    include(${CMAKE_MODULE_PATH}/External_PrecisionMissions.cmake)
  endif()

  option(USE_COLLISION_AVOIDANCE "Use DJI collision avoidance library" OFF)
  if(USE_COLLISION_AVOIDANCE)
    include(${CMAKE_MODULE_PATH}/External_CollisionAvoidance.cmake)
  endif()

  option(USE_POINTCLOUD2LAS "Use DJI pointcloud to LAS converter library" OFF)
  if(USE_POINTCLOUD2LAS)
    include(${CMAKE_MODULE_PATH}/External_Pointcloud2Las.cmake)
  endif()
endif()

# Install VTK external dependency
if(LIDAR_LOGGING)
  include(${CMAKE_MODULE_PATH}/External_VTK.cmake)
endif()

# Install GTest and RapidJSON external dependencies for tests
if(GTEST)
  include(${CMAKE_MODULE_PATH}/External_GTest.cmake)
endif()

if(GTEST OR USE_PRECISION_MISSIONS)
  include(${CMAKE_MODULE_PATH}/External_RapidJSON.cmake)
endif()

add_subdirectory(osdk-core)
add_subdirectory(osdk-wrapper)
add_subdirectory(platform/linux)
add_subdirectory(sample/Linux/Non-Blocking)
add_subdirectory(sample/Linux/Blocking)
if (LIDAR_LOGGING)
  add_subdirectory(sensor-integration/velodyne_vlp16puck_lite_API_example)
endif ()

