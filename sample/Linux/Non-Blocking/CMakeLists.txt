cmake_minimum_required(VERSION 2.8)
project(djiosdk-linux-sample-nonblock)
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

FILE(GLOB SOURCE_FILES inc/*.h src/*.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} djiosdk-core djiosdk-platform djiosdk-wrapper)
