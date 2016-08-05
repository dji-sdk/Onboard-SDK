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
  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    find_package(Threads)
  endif()
  find_package(GTest REQUIRED) 
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

include_directories(
  ${GTEST_INCLUDE_DIRS})

## Set compiler flags accordingly to the build type
if(MSVC)
  include_directories(${Qt5Widgets_INCLUDES})
  add_definitions(${Qt5Widgets_DEFINITIONS})
  set(COMMON_CXX_FLAGS "-std=c++11 ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
endif(MSVC)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
  set(COMMON_CXX_FLAGS "-std=c++11 -pthread")
endif()

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
  json11
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
  set(CMAKE_PREFIX_PATH "~/Documents/json11/")
  find_package(json11)
  add_executable(DJI_APITest test/DJI_APITest.cpp test/DJI_FlightTest.cpp test/DJI_VirtualRCTest.cpp test/DJI_HotPointTest.cpp test/DJI_WayPointTest.cpp )
  if(TARGET DJI_APITest)
    if(MSVC)
      qt5_use_modules(DJI_APITest Core SerialPort)
      target_link_libraries(DJI_APITest 
        dji_sdk_lib 
        ${GTEST_LIBRARIES} 
        ${Qt5Widgets_LIBRARIES}
      )
    endif(MSVC)
     
    if(CMAKE_SYSTEM_NAME MATCHES Linux)
      target_link_libraries(DJI_APITest 
        json11
        dji_sdk_lib 
        ${GTEST_LIBRARIES} 
        pthread)
    endif()
  endif()

  add_test(${EXECUTABLE_OUTPUT_PATH}/DJI_APITest ${EXECUTABLE_OUTPUT_PATH}/DJI_APITest)
endif()
