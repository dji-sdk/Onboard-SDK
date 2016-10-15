cmake_minimum_required(VERSION 2.8)
project(vlp16lidar-driver)

if (COVERAGE)
  set(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage -fPIC")
  set(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
endif ()

# Add module path for Find<name>.cmake conf files
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/contrib)

option(BUILD_SIMULATOR "Build Velodyne LiDAR Simulator" ON)
option(BUILD_POINTCLOUD_GRABBER "Build PointClout Grabber app sample" ON)

add_subdirectory(vlp16lidar-driver)

if(BUILD_SIMULATOR)
  add_subdirectory(sim)
endif()

if(BUILD_POINTCLOUD_GRABBER)
  add_subdirectory(pointcloud_grabber)
endif()

if(GTEST)
  add_subdirectory(test)
endif()

