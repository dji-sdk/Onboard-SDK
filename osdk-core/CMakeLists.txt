cmake_minimum_required(VERSION 2.8)
project(djiosdk-core)


if(CMAKE_SYSTEM_PROCESSOR MATCHES "i386|i686|x86|AMD64|x86_64")
  set(ARCH x86)

elseif((CMAKE_SYSTEM_PROCESSOR MATCHES "^arm*") OR (CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch"))
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(ARCH armv7)
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ARCH armv8)
  endif()
endif()
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
  set(COMMON_CXX_FLAGS "-std=c++11 -pthread -g -fPIC")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
  add_definitions(-DDJIOSDK_OPERATOR_TYPE=1)
else()
  add_definitions(-DDJIOSDK_OPERATOR_TYPE=0)
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

add_definitions(-DPLATFORM_OSDK)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "i386|i686|x86|AMD64|x86_64")
  set(ARCH x86)
  add_definitions(-DDJIOSDK_HARDWARE_TYPE=3)
elseif((CMAKE_SYSTEM_PROCESSOR MATCHES "^arm*") OR (CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch"))
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(ARCH armv7)
    add_definitions(-DDJIOSDK_HARDWARE_TYPE=1)
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ARCH armv8)
    add_definitions(-DDJIOSDK_HARDWARE_TYPE=2)
  endif()
elseif()
  add_definitions(-DDJIOSDK_HARDWARE_TYPE=0)
endif()

include (${CMAKE_CURRENT_SOURCE_DIR}/contrib/DJIConfig.cmake)
include_directories(
        api/inc
        modules/inc/firewall
        modules/inc/filemgr
        modules/inc/filemgr/impl
        modules/inc/payload
        modules/inc/flight
        modules/inc/mop
        platform/inc
        logger/inc
        utility/inc
        linker/${ARCH}/inc)

## Source code for OSDK CORE
FILE(GLOB OSDK_LIB_SRCS
        api/src/*.cpp
        modules/src/firewall/*.cpp
        modules/src/filemgr/*.cpp
        modules/src/filemgr/impl/*.cpp
        modules/src/payload/*.cpp
        modules/src/flight/*.cpp
        modules/src/mop/*.cpp
        platform/src/*.c*
        logger/src/*.cpp
        utility/src/*.cpp)

link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/linker/${ARCH}/lib/libdji-linker.a)

add_library(${PROJECT_NAME}
        STATIC
        ${OSDK_LIB_SRCS})

add_subdirectory(advanced-sensing)


set(ADVANCED_SENSING_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/advanced-sensing)
set(ADVANCED_SENSING_HEADERS_DIR
    ${ADVANCED_SENSING_SOURCE_ROOT}/api/inc
    ${ADVANCED_SENSING_SOURCE_ROOT}/platform/inc
    ${ADVANCED_SENSING_SOURCE_ROOT}/protocol/inc
    ${ADVANCED_SENSING_SOURCE_ROOT}/camera_stream/src
    ${ADVANCED_SENSING_SOURCE_ROOT}/ori-osdk-core/protocol/inc
    ${ADVANCED_SENSING_SOURCE_ROOT}/ori-osdk-core/hal/inc
    ${ADVANCED_SENSING_SOURCE_ROOT}/ori-osdk-core/platform/linux/inc
    )

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
  #find_package(AdvancedSensing REQUIRED)
  find_package(FFMPEG REQUIRED)

  message(${ADVANCED_SENSING_HEADERS_DIR})
  include_directories(${ADVANCED_SENSING_HEADERS_DIR})



  if(FFMPEG_FOUND)
    message( STATUS "Found FFmpeg ${FFMPEG_VERSION} installed in the system.")
    message( STATUS " - Includes: ${FFMPEG_INCLUDE_DIRS}")
    message( STATUS " - Libraries: ${FFMPEG_LIBRARIES}")
  else()
    message("Cannot Find FFMPEG")
  endif(FFMPEG_FOUND)

  target_include_directories(${PROJECT_NAME} PUBLIC
          $<BUILD_INTERFACE:${ADVANCED_SENSING_HEADERS_DIR}>)
  target_link_libraries(${PROJECT_NAME} PRIVATE advanced-sensing)


  target_include_directories(${PROJECT_NAME} PUBLIC ${LIBUSB_1_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBUSB_1_LIBRARIES})

  target_include_directories(${PROJECT_NAME} PUBLIC ${FFMPEG_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${FFMPEG_LIBRARIES})

  set(OSDK_INTERFACE_LIBS ${OSDK_INTERFACE_LIBS} ${LIBUSB_1_LIBRARIES} ${FFMPEG_LIBRARIES})
  set(MODULE_BUILD_INTERFACE ${MODULE_BUILD_INTERFACE} advanced-sensing)
  set(MODULE_INSTALL_INTERFACE ${MODULE_INSTALL_INTERFACE} advanced-sensing)

  target_compile_definitions(${PROJECT_NAME} PUBLIC -DADVANCED_SENSING)

endif()

# Once all the modules are done, set the interface_link_libraries property
set_property(TARGET ${PROJECT_NAME}
        PROPERTY INTERFACE_LINK_LIBRARIES
        $<BUILD_INTERFACE:${MODULE_BUILD_INTERFACE}>
        $<INSTALL_INTERFACE:${MODULE_INSTALL_INTERFACE}>
        ${OSDK_INTERFACE_LIBS}
        )

################
# Installation #
################

set(OSDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../onboard-sdk/osdk-lib)

set(OSDK_LIB_ARCH_DIR ${OSDK_LIB_DIR}/${ARCH})
set(OSDK_CORE_INSTALL_INCLUDE_DIR ${OSDK_LIB_ARCH_DIR}/osdk-core/inc)
set(OSDK_CORE_INSTALL_LIB_DIR ${OSDK_LIB_ARCH_DIR}/osdk-core/lib)

#[[set(OSDK_LIB_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../osdk-lib)
set(OSDK_CORE_INSTALL_LIB_DIR ${OSDK_LIB_ROOT_DIR}/lib)
set(INSTALL_BIN_DIR ${OSDK_LIB_ROOT_DIR}/bin)
set(OSDK_CORE_INSTALL_INCLUDE_DIR ${OSDK_LIB_ROOT_DIR}/include/djiosdk)
set(DEF_INSTALL_CMAKE_DIR  ${OSDK_LIB_ROOT_DIR}/lib/cmake/djiosdk)
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR})]]

#[[if(OSDK_LIB_REFRESH)
  message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
  set(CMAKE_INSTALL_PREFIX ${OSDK_LIB_ROOT_DIR})
  message("!!! CAUTION : CMAKE_INSTALL_PREFIX -----------> ${CMAKE_INSTALL_PREFIX}")
  message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
else()
  message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
  message("!!! CAUTION : CMAKE_INSTALL_PREFIX -----------> ${CMAKE_INSTALL_PREFIX}")
  message("!!! CAUTION : ALL THE LIB WILL BE INSTALLED HEAR. YOU CAN TYPE : ")
  message("  cmake .. -DCMAKE_INSTALL_PREFIX=${OSDK_LIB_ROOT_DIR}")
  message("      OR  ")
  message("  cmake .. -DOSDK_LIB_REFRESH=ON")
  message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
endif()]]

foreach(p LIB BIN INCLUDE CMAKE)
  #set(var INSTALL_${p}_DIR)
  #if(NOT IS_ABSOLUTE "${${var}}")
  #  set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  #endif()
  #message("*******---===+++---->>> INSTALL_${p}_DIR : ${INSTALL_${p}_DIR}")
endforeach()

#[[# Add all targets to the build-tree export set
export(TARGETS djiosdk-core
        FILE "${PROJECT_BINARY_DIR}/djiosdkTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE djiosdk)

# Create the DJIOSDKConfig.cmake and DJIOSDKConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
        "${OSDK_CORE_INSTALL_INCLUDE_DIR}")
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
        DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)]]

FILE(GLOB OSDK_LIB_HEADERS
        ##aircraft function APIs related header files
        api/inc/*.h*
        modules/inc/payload/*.h*
        modules/inc/flight/*.h*
        modules/inc/mop/*.h*
        ##osdk application related header files
        platform/inc/*.h*
        logger/inc/*.h*
        utility/inc/*.h*
        ##linker layer related header files
        ##linker/core/mop/mop_entry_osdk.h
        ##linker/core/osdk_typedef.h
        ##linker/core/logger/osdk_logger.h
        ##linker/core/platform/osdk_platform.h
        linker/${ARCH}/inc/*.h*
        modules/inc/filemgr/*.h*
        modules/inc/filemgr/impl/*.h*
        )

# Append advanced sensing headers
#if(ADVANCED_SENSING)
#  FILE(GLOB ADVANCED_SENSING_HEADERS
#          ${ADVANCED_SENSING_INCLUDE_DIRS}/*.h*)
#
#  install(FILES ${ADVANCED_SENSING_LIBRARY}
#          DESTINATION "${ADVANCED_SENSING_INSTALL_INCLUDE_DIR}" COMPONENT shlib)
#
#endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
        PUBLIC_HEADER "${OSDK_LIB_HEADERS}")
#[[
install(TARGETS ${PROJECT_NAME}
        EXPORT djiosdkTargets
        ARCHIVE DESTINATION "${OSDK_CORE_INSTALL_LIB_DIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${OSDK_CORE_INSTALL_INCLUDE_DIR}" COMPONENT dev
        )
]]

install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/linker/${ARCH}/lib/libdji-linker.a
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" COMPONENT shlib
        )

install(TARGETS ${PROJECT_NAME}
        EXPORT djiosdkTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include" COMPONENT dev
        )

#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/advanced-sensing-2.0.3
#        DESTINATION ${OSDK_LIB_ARCH_DIR} )
#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/waypointv2-core-1.0.0
#        DESTINATION ${OSDK_LIB_ARCH_DIR} )