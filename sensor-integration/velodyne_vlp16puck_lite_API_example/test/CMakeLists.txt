cmake_minimum_required(VERSION 2.8)
project(VLP16LiDAR_Test)

if (COVERAGE)
  set(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage -fPIC")
  set(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
endif ()

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

# Find VTK in your system
set(VTK_CMAKE_PATHS
  /lib/cmake
  /usr/lib/cmake
  /usr/local/lib/cmake
  /opt/VTK-7.0.0/lib/cmake)

include_directories(../vlp16lidar-driver/inc)

# Test with GTest framework
if(CMAKE_SYSTEM_NAME MATCHES Linux)
  include_directories(test ${Boost_INCLUDE_DIRS} ${LIBLAS_INCLUDE_DIR})
  FILE(GLOB TEST_SOURCE_FILES *.cpp)
  add_executable(${PROJECT_NAME} ${TEST_SOURCE_FILES})
  add_dependencies(${PROJECT_NAME} googletest)
  
  add_dependencies(${PROJECT_NAME} VTK)
  if(VTK_FOUND)
    include(${VTK_USE_FILE})
    message(STATUS " VTK include dir set to ${VTK_INCLUDE_DIRS}")
  endif()

  target_link_libraries(${PROJECT_NAME}
    boost_thread
    boost_system
    boost_iostreams
    boost_filesystem
    vlp16lidar-driver
    ${gtest_binary_dir}/googlemock/gtest/libgtest.a
    ${gtest_binary_dir}/googlemock/gtest/libgtest_main.a
    pthread)
  add_test(${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME} ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME})
endif()

