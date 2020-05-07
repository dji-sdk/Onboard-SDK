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
project(onboardsdk-linux-samples)

# set(DJIOSDK_DIR ${OSDK_CORE_LIB_DIR}/lib/cmake/djiosdk)
# find_package(DJIOSDK REQUIRED)

include_directories(${DJIOSDK_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hal)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/osal)


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

set(OSDK_CORE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../osdk-core)
set(CMAKE_MODULE_PATH ${OSDK_CORE_PATH}/cmake-modules)

#set(OSDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../../../osdk-lib/${ARCH})
#set(CMAKE_MODULE_PATH ${OSDK_LIB}/../cmake-modules)

#set(OSDK_CORE_LIB ${OSDK_LIB}/osdk-core)
#set(ADVANCED_SENSING_LIB ${OSDK_LIB}/advanced-sensing-2.0.3)
#set(WP2_CORE_LIB  ${OSDK_LIB}/waypointv2-core-1.0.0)

#include_directories(${OSDK_CORE_LIB}/inc)
#include_directories(${ADVANCED_SENSING_LIB}/inc)

message("-----------OSDK_LIB_HEADERS_DIR------------")
set(OSDK_LIB_HEADERS_DIR
        ${OSDK_CORE_PATH}/api/inc
        ${OSDK_CORE_PATH}/modules/inc/payload
        ${OSDK_CORE_PATH}/modules/inc/flight
        ${OSDK_CORE_PATH}/modules/inc/mop
        ${OSDK_CORE_PATH}/platform/inc
        ${OSDK_CORE_PATH}/logger/inc
        ${OSDK_CORE_PATH}/utility/inc
        ${OSDK_CORE_PATH}/linker/${ARCH}/inc
        ${OSDK_CORE_PATH}/modules/inc/filemgr
        )
include_directories(${OSDK_LIB_HEADERS_DIR})
message((${OSDK_LIB_HEADERS_DIR}))


message("-----------ADVANCED_SENSING_HEADERS_DIR------------")
set(ADVANCED_SENSING_SOURCE_ROOT ${OSDK_CORE_PATH}/advanced-sensing)
set(ADVANCED_SENSING_HEADERS_DIR
    ${ADVANCED_SENSING_SOURCE_ROOT}/api/inc
    ${ADVANCED_SENSING_SOURCE_ROOT}/platform/inc
    ${ADVANCED_SENSING_SOURCE_ROOT}/protocol/inc
    ${ADVANCED_SENSING_SOURCE_ROOT}/camera_stream/src
    ${ADVANCED_SENSING_SOURCE_ROOT}/ori-osdk-core/protocol/inc
    ${ADVANCED_SENSING_SOURCE_ROOT}/ori-osdk-core/hal/inc
    ${ADVANCED_SENSING_SOURCE_ROOT}/ori-osdk-core/platform/linux/inc
    )
include_directories(${ADVANCED_SENSING_HEADERS_DIR})
message((${ADVANCED_SENSING_HEADERS_DIR}))

find_package(LibUSB REQUIRED)
find_package(FFMPEG REQUIRED)
#find_package(Waypt2Core REQUIRED)

if(FFMPEG_FOUND)
    message( STATUS "Found FFmpeg ${FFMPEG_VERSION} installed in the system.")
    message( STATUS " - Includes: ${FFMPEG_INCLUDE_DIRS}")
    message( STATUS " - Libraries: ${FFMPEG_LIBRARIES}")
else()
    message("Cannot Find FFMPEG")
endif(FFMPEG_FOUND)


include_directories(${LIBUSB_1_INCLUDE_DIRS})
include_directories(${FFMPEG_INCLUDE_DIR})
#include_directories(${WAYPT2_CORE_INCLUDE_DIRS})

#link libraries

#link_libraries(${OSDK_CORE_LIB}/lib/libdjiosdk-core.a)

if(ADVANCED_SENSING)
    link_libraries(advanced-sensing)
    link_libraries(djiosdk-core)
    link_libraries(${LIBUSB_1_LIBRARIES})
    link_libraries(${FFMPEG_LIBRARIES})
endif()

#if(WAYPT2_CORE)
#endif()

#add_subdirectory(camera-gimbal)
add_subdirectory(flight-control)
add_subdirectory(mfio)
add_subdirectory(missions)
add_subdirectory(mobile)
add_subdirectory(telemetry)
add_subdirectory(logging)
add_subdirectory(time-sync)
add_subdirectory(payload-3rd-party)
add_subdirectory(payloads)
add_subdirectory(advanced-sensing)
add_subdirectory(hms)
add_subdirectory(battery)
add_subdirectory(mop)


