cmake_minimum_required(VERSION 2.8)
project(djiosdk-linux-sample-nonblock)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILER} -pthread -g -O0")

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

if (USE_PRECISION_MISSIONS)
  include_directories("${precision_missions_source_dir}/inc")

  # Local Mission Planning libraries
  include_directories("${precision_missions_source_dir}/dependencies")
  include_directories("${precision_missions_source_dir}/dependencies/csv")
endif()

FILE(GLOB SOURCE_FILES inc/*.h src/*.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

add_dependencies(${PROJECT_NAME} djiosdk-wrapper)

target_link_libraries(${PROJECT_NAME} 
  djiosdk-core 
  djiosdk-platform 
  djiosdk-wrapper)

