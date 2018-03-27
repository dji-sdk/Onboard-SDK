# *  @Copyright (c) 2016-2017 DJI
# *
# * Permission is hereby granted, free of charge, to any person obtaining a copy
# * of this software and associated documentation files (the "Software"), to deal
# * in the Software without restriction, including without limitation the rights
# * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# * copies of the Software, and to permit persons to whom the Software is
# * furnished to do so, subject to the following conditions:
# *
# * The above copyright notice and this permission notice shall be included in
# * all copies or substantial portions of the Software.
# *
# * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# * SOFTWARE.
# *
# *

cmake_minimum_required(VERSION 2.8)
project(djiosdk-core)

# We want to use INTERFACE_LINK_LIBRARIES property to 
# propagate link dependencies
cmake_policy(SET CMP0022 NEW)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
          "Choose the type of build Debug/Release/Coverage:"
          FORCE)
endif ()

# Add module path for <name>.cmake conf files
set(CURRENT_CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

## Set compiler flags accordingly to the build type
if (MSVC)
  include_directories(${Qt5Widgets_INCLUDES})
  add_definitions(${Qt5Widgets_DEFINITIONS})
  set(COMMON_CXX_FLAGS "-std=c++11 ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
endif (MSVC)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  set(COMMON_CXX_FLAGS "-std=c++11 -pthread")
endif ()

if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0")
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
endif ()

if (COVERAGE)
  SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage -fPIC")
  SET(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${GCC_COVERAGE_COMPILE_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
endif ()

if (CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif ()

include (${CMAKE_CURRENT_SOURCE_DIR}/../contrib/DJIConfig.cmake)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        api/inc
        protocol/inc
        hal/inc
        utility/inc
        platform/default/inc)

# Use this if more platform is supported
if (CMAKE_SYSTEM_NAME MATCHES Linux)
  include_directories(
          platform/linux/inc)
endif()

## Source code for OSDK CORE
FILE(GLOB OSDK_LIB_SRCS
        api/src/*.cpp
        protocol/src/*.cpp
        hal/src/*.cpp
        utility/src/*.cpp
        platform/default/src/*.cpp)

# Use this if more platform is supported
if (CMAKE_SYSTEM_NAME MATCHES Linux)
  FILE(GLOB OSDK_LIB_SRCS ${OSDK_LIB_SRCS} platform/linux/src/*.cpp)
endif()

add_library(${PROJECT_NAME}
        STATIC
        ${OSDK_LIB_SRCS})

## Libraries to propagate as dependencies to third-party code depending on osdk-core
## Append to this variable when you want a dependency to get propagated
SET(OSDK_INTERFACE_LIBS pthread) # pthread is assumed available on linux

## Modules to link/build against
## Append to these variables with build/install locations when adding new OSDK modules
SET(MODULE_BUILD_INTERFACE "")
SET(MODULE_INSTALL_INTERFACE "")

## Advanced Sensing
if(ADVANCED_SENSING) 
  # Add a cmake file to find libusb
  set(CMAKE_MODULE_PATH ${CURRENT_CMAKE_MODULE_PATH})
  
  find_package(LibUSB REQUIRED)
  find_package(AdvancedSensing QUIET)
  if(NOT ADVANCED_SENSING_FOUND)
    include(${CURRENT_CMAKE_MODULE_PATH}/External_AdvancedSensing.cmake)
    add_dependencies(${PROJECT_NAME} advanced-sensing)
  endif()

  find_package(FFMPEG REQUIRED)

  if(FFMPEG_FOUND)
      message( STATUS "Found FFmpeg ${FFMPEG_VERSION} installed in the system.")
      message( STATUS " - Includes: ${FFMPEG_INCLUDE_DIRS}")
      message( STATUS " - Libraries: ${FFMPEG_LIBRARIES}")
  else()
    message("Cannot Find FFMPEG")
  endif(FFMPEG_FOUND)

  target_include_directories(${PROJECT_NAME} PUBLIC
                             $<BUILD_INTERFACE:${ADVANCED_SENSING_INCLUDE_DIRS}>)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${ADVANCED_SENSING_LIBRARY})


  target_include_directories(${PROJECT_NAME} PUBLIC ${LIBUSB_1_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBUSB_1_LIBRARIES})

  target_include_directories(${PROJECT_NAME} PUBLIC ${FFMPEG_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${FFMPEG_LIBRARIES})

  set(OSDK_INTERFACE_LIBS ${OSDK_INTERFACE_LIBS} ${LIBUSB_1_LIBRARIES} ${FFMPEG_LIBRARIES})
  set(MODULE_BUILD_INTERFACE ${MODULE_BUILD_INTERFACE} ${ADVANCED_SENSING_LIBRARY})
  set(MODULE_INSTALL_INTERFACE ${MODULE_INSTALL_INTERFACE} $<INSTALL_PREFIX>/lib/libadvanced-sensing.a)

  target_compile_definitions(${PROJECT_NAME} PUBLIC -DADVANCED_SENSING)

endif()

# If there are more modules, add them here, BEFORE we set INTERFACE_LINK_LIBRARIES property.

## Once all the modules are done, set the interface_link_libraries property
set_property(TARGET ${PROJECT_NAME}
        PROPERTY INTERFACE_LINK_LIBRARIES
        $<BUILD_INTERFACE:${MODULE_BUILD_INTERFACE}>
        $<INSTALL_INTERFACE:${MODULE_INSTALL_INTERFACE}>
        ${OSDK_INTERFACE_LIBS}
        )

################
# Installation #
################

set(INSTALL_LIB_DIR lib)
set(INSTALL_BIN_DIR bin)
set(INSTALL_INCLUDE_DIR include/djiosdk)
set(DEF_INSTALL_CMAKE_DIR lib/cmake/djiosdk)
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR})
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# Add all targets to the build-tree export set
export(TARGETS djiosdk-core
        FILE "${PROJECT_BINARY_DIR}/djiosdkTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE djiosdk)

# Create the DJIOSDKConfig.cmake and DJIOSDKConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
        "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfig.cmake.in
        "${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfig.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "\${DJIOSDK_CMAKE_DIR}/${REL_INCLUDE_DIR}")

configure_file(${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfig.cmake.in
        "${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfig.cmake" @ONLY)

# ... for both
configure_file(${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfigVersion.cmake.in
        "${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfigVersion.cmake" @ONLY)

# Install the DJIOSDKConfig.cmake and DJIOSDKConfigVersion.cmake
install(FILES
        "${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfig.cmake"
        "${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfigVersion.cmake"
        DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

FILE(GLOB OSDK_LIB_HEADERS
        api/inc/*.h*
        protocol/inc/*.h*
        hal/inc/*.h*
        utility/inc/*.h*
        default/inc/*.h*
        platform/linux/inc/*.h*
        )

# Append advanced sensing headers
if(ADVANCED_SENSING)
  install(FILES ${ADVANCED_SENSING_LIBRARY}
        DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib)

  set(OSDK_LIB_HEADERS 
      ${OSDK_LIB_HEADERS} 
      ${ADVANCED_SENSING_INCLUDE_DIRS}/dji_advanced_sensing.hpp
      ${ADVANCED_SENSING_INCLUDE_DIRS}/dji_advanced_sensing_protocol.hpp
      ${ADVANCED_SENSING_INCLUDE_DIRS}/linux_usb_device.hpp
      ${ADVANCED_SENSING_INCLUDE_DIRS}/dji_camera_image.hpp
      ${ADVANCED_SENSING_INCLUDE_DIRS}/dji_camera_stream.hpp
      )
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
        PUBLIC_HEADER "${OSDK_LIB_HEADERS}")

install(TARGETS ${PROJECT_NAME}
        EXPORT djiosdkTargets
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT dev
        )

# Install the export set for use with the install-tree
install(EXPORT djiosdkTargets DESTINATION
        "${INSTALL_CMAKE_DIR}" COMPONENT dev)
