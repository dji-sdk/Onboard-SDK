cmake_minimum_required(VERSION 2.8)
project(djiosdk-core)

cmake_policy(SET CMP0022 NEW)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
          "Choose the type of build Debug/Release/Coverage:"
          FORCE)
endif ()

# Add module path for <name>.cmake conf files
set(CURRENT_CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

if (MSVC)
  include_directories(${Qt5Widgets_INCLUDES})
  add_definitions(${Qt5Widgets_DEFINITIONS})
  set(COMMON_CXX_FLAGS "-std=c++11 ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
endif (MSVC)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  set(COMMON_CXX_FLAGS "-std=c++11 -pthread -fPIC")
endif ()

if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0 -Wall -Werror")
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

include_directories(
        api/inc
        platform/inc
        logger/inc
        utility/inc
        utils/inc
        linker
        linker/channel
        linker/command
        linker/logger
        linker/protocol
        linker/root_task)

## Source code for OSDK CORE
FILE(GLOB OSDK_LIB_SRCS
        api/src/*.cpp
        platform/src/*.c*
        logger/src/*.cpp
        utils/src/*.c*
        utility/src/*.cpp
        linker/channel/*.c
        linker/command/*.c
        linker/logger/*.c
        linker/protocol/*.c
        linker/root_task/*.c)

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
        platform/inc/*.h*
        logger/inc/*.h*
        utility/inc/*.h*
        utils/inc/*.h*
        linker/*.h*
        linker/channel/*.h*
        linker/command/*.h*
        linker/logger/*.h*
        linker/protocol/*.h*
        linker/root_task/*.h*)

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
