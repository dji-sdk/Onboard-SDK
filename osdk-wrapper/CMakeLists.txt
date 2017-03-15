cmake_minimum_required(VERSION 2.8)
project(djiosdk-wrapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

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
  # Precision-Missions library requiers RapidJSON
  add_dependencies(${PROJECT_NAME} rapidjson)
  include_directories("${rapidjson_source_dir}/include")

  # If you are not using cutom binaries, CMake will download
  # pre-duilt binaries and place them in the CMake build directory
  if(NOT EXISTS ${PRECISION_MISSIONS_INCLUDE_DIR})
    add_dependencies(${PROJECT_NAME} PrecisionMissions)
    set(PRECISION_MISSIONS_INCLUDE_DIR "${precision_missions_source_dir}/inc" CACHE INTERNAL "" FORCE)
    set(PRECISION_MISSIONS_DEPENDENCIES_DIR "${precision_missions_source_dir}/dependencies" CACHE INTERNAL "" FORCE)
 endif()

  if(NOT EXISTS ${PRECISION_MISSIONS_LIBRARY})
     set(PRECISION_MISSIONS_LIBRARY
     "${precision_missions_source_dir}/lib/${TARGET_ARCH}/${DISTRO_VERSION}/libmissionplan.a")
  endif()

  include_directories("${PRECISION_MISSIONS_INCLUDE_DIR}")
  include_directories("${PRECISION_MISSIONS_DEPENDENCIES_DIR}")
  include_directories("${PRECISION_MISSIONS_DEPENDENCIES_DIR}/csv")
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
  #target_link_libraries(${PROJECT_NAME}  djiosdk-core vlp16lidar-driver djiosdk-platform ${PRECISION_MISSIONS_LIBRARY})
endif()

if (USE_PRECISION_MISSIONS)
  set(WRAPPER_LINK_LIBRARIES ${WRAPPER_LINK_LIBRARIES} ${PRECISION_MISSIONS_LIBRARY})
endif ()

target_link_libraries(${PROJECT_NAME}  ${WRAPPER_LINK_LIBRARIES})

