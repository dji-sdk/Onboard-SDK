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

set(OSDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../../../osdk-lib/${ARCH})
set(CMAKE_MODULE_PATH ${OSDK_LIB}/../cmake-modules)

set(OSDK_CORE_LIB ${OSDK_LIB}/osdk-core)
set(ADVANCED_SENSING_LIB ${OSDK_LIB}/advanced-sensing-2.0.3)
#set(WP2_CORE_LIB  ${OSDK_LIB}/waypointv2-core-1.0.0)

include_directories(${OSDK_CORE_LIB}/inc)
include_directories(${ADVANCED_SENSING_LIB}/inc)

find_package(LibUSB REQUIRED)
find_package(FFMPEG REQUIRED)
find_package(Waypt2Core REQUIRED)

if(FFMPEG_FOUND)
    message( STATUS "Found FFmpeg ${FFMPEG_VERSION} installed in the system.")
    message( STATUS " - Includes: ${FFMPEG_INCLUDE_DIRS}")
    message( STATUS " - Libraries: ${FFMPEG_LIBRARIES}")
else()
    message("Cannot Find FFMPEG")
endif(FFMPEG_FOUND)


include_directories(${LIBUSB_1_INCLUDE_DIRS})
include_directories(${FFMPEG_INCLUDE_DIR})
include_directories(${WAYPT2_CORE_INCLUDE_DIRS})

#link libraries

link_libraries(${OSDK_CORE_LIB}/lib/libdjiosdk-core.a)

if(ADVANCED_SENSING)
    link_libraries(${ADVANCED_SENSING_LIB}/lib/libadvanced-sensing.a)
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


