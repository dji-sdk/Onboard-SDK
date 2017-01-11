cmake_minimum_required(VERSION 2.8)
project(djiosdk-wrapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILER} -pthread -g -O0")

if (COVERAGE)
  set(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage -fPIC")
  set(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
endif ()

include_directories(../osdk-core/inc)
include_directories(../platform/linux/inc)
include_directories(inc)
if (LIDAR_LOGGING)
  include_directories(../sensor-integration/velodyne_vlp16puck_lite_API_example/vlp16lidar-driver/inc)
endif ()

FILE(GLOB SOURCE_FILES inc/*.h src/*.cpp)
add_library(${PROJECT_NAME} ${SOURCE_FILES})

if (USE_PRECISION_MISSIONS AND (${DISTRO_VERSION} STREQUAL "1604"))
  add_dependencies(${PROJECT_NAME} rapidjson)
  include_directories("${rapidjson_source_dir}/include")

  add_dependencies(${PROJECT_NAME} PrecisionMissions)
  include_directories("${precision_missions_source_dir}/inc")

  # Local Mission Planning libraries
  include_directories("${precision_missions_source_dir}/dependencies")
  include_directories("${precision_missions_source_dir}/dependencies/csv")

  set(MISSIONPLAN_LIBRARY 
   "${precision_missions_source_dir}/lib/${TARGET_ARCH}/${DISTRO_VERSION}/libmissionplan.a")
endif()

if (USE_COLLISION_AVOIDANCE)
  add_dependencies(${PROJECT_NAME} dji-ros-collision-avoidance)
endif()

if (USE_POINTCLOUD2LAS)
  add_dependencies(${PROJECT_NAME} dji-ros-pointcloud2las)
endif()


set (WRAPPER_LINK_LIBRARIES djiosdk-core djiosdk-platform)

if (LIDAR_LOGGING)
  set(WRAPPER_LINK_LIBRARIES ${WRAPPER_LINK_LIBRARIES} vlp16lidar-driver)
  #target_link_libraries(${PROJECT_NAME}  djiosdk-core vlp16lidar-driver djiosdk-platform ${MISSIONPLAN})
endif()

if (USE_PRECISION_MISSIONS)
  set(WRAPPER_LINK_LIBRARIES ${WRAPPER_LINK_LIBRARIES} ${MISSIONPLAN_LIBRARY})
endif ()

target_link_libraries(${PROJECT_NAME}  ${WRAPPER_LINK_LIBRARIES})

