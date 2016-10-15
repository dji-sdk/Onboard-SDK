cmake_minimum_required(VERSION 2.8)
project(onboardsdk)

set(CMAKE_VERBOSE_MAKEFILE OFF)

if (LIDAR_LOGGING)
  # Try to find VTK in your system (install if does not exist)
set(VTK_CMAKE_PATHS /lib/cmake /usr/lib/cmake /usr/local/lib/cmake /opt/VTK-7.0.0/lib/cmake)
find_package(VTK 7.0 QUIET 
  HINTS ${VTK_CMAKE_PATHS}
  NO_DEFAULT_PATH)

if(NOT VTK_FOUND)
  # Download and installation will take up to 15min
  include(External_VTK.cmake)
  find_package(VTK 7.0 REQUIRED)
endif()
include(${VTK_USE_FILE})
message(STATUS " VTK include dir set to ${VTK_INCLUDE_DIRS}")
add_definitions(-DLIDAR_LOGGING)
endif()
# Add module path for <name>.cmake conf files
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/contrib/)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/libs)

# Set test related flags
option(GTEST "Build with Google Test framework" OFF)
option(RapidJSON "Build with JSON support to complement GTest" OFF)

# Allow to build additional modules
#option(LIDAR_LOGGING "Build VLP16 Puck LiDAR logging" OFF)

# Find test related libraries
if(GTEST)
  include(${CMAKE_MODULE_PATH}/External_GTest.cmake)
  if(RapidJSON)
    include(${CMAKE_MODULE_PATH}/External_RapidJSON.cmake)
  endif()
endif()

add_subdirectory(osdk-core)
add_subdirectory(osdk-wrapper)
add_subdirectory(platform/linux)
add_subdirectory(sample/Linux/Non-Blocking)
add_subdirectory(sample/Linux/Blocking)
if (LIDAR_LOGGING)
  add_subdirectory(sensor-integration/velodyne_vlp16puck_lite_API_example)
endif ()
