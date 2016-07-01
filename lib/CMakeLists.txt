cmake_minimum_required(VERSION 2.8.3)

project(dji_sdk_lib)

if(NOT CMAKE_BUILD_TYPE)
  set( CMAKE_BUILD_TYPE Debug CACHE STRING
    "Choose the type of build Debug/Release:"
    FORCE)
endif()

option(GTEST "Enable GoogleTest" OFF)
if(GTEST)
  enable_testing()
  find_package(GTest)
  find_package(Threads)
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

include_directories(
  ${GTEST_INCLUDE_DIRS})

## Set compiler flags accordingly to the build type
set(COMMON_CXX_FLAGS "-std=c++11 -pthread")

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
endif()

if(CMAKE_BUILD_TYPE MATCHES "Release")
  set(CMAKE_CXX_FLAGS_RELASE "${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  inc
  test)

## Declare a C++ library
FILE(GLOB DJI_SDK_LIB_SOURCES src/*.cpp)

add_library(dji_sdk_lib
  STATIC 
  ${DJI_SDK_LIB_SOURCES})

INSTALL(TARGETS dji_sdk_lib
    RUNTIME DESTINATION ${EXECUTABLE_OUTPUT_PATH}
    LIBRARY DESTINATION ${EXECUTABLE_OUTPUT_PATH}
    ARCHIVE DESTINATION ${EXECUTABLE_OUTPUT_PATH}
    CONFIGURATIONS Debug
    COMPONENT library)

INSTALL(TARGETS dji_sdk_lib
    RUNTIME DESTINATION ${EXECUTABLE_OUTPUT_PATH}
    LIBRARY DESTINATION ${EXECUTABLE_OUTPUT_PATH}
    ARCHIVE DESTINATION ${EXECUTABLE_OUTPUT_PATH}
    CONFIGURATIONS Release
    COMPONENT library)

## Add gtest based cpp test target and link libraries
if(GTEST_FOUND)
  add_executable(DJI_APITest test/DJI_APITest.cpp)

  if(TARGET DJI_APITest)
    target_link_libraries(DJI_APITest 
      dji_sdk_lib 
      ${GTEST_LIBRARIES} 
      pthread
    )
  endif()

  add_test(${EXECUTABLE_OUTPUT_PATH}/DJI_APITest ${EXECUTABLE_OUTPUT_PATH}/DJI_APITest)
endif()
