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

if(NOT ONBOARDSDK_SOURCE)
    set(ONBOARDSDK_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../../osdk-core")
endif()

if(NOT MODULES_HEADER_DIR)
    set(MODULES_HEADER_DIR "${ONBOARDSDK_SOURCE}/modules/inc/payload")
endif()

include_directories(${ONBOARDSDK_SOURCE}/linker/command)
include_directories(${ONBOARDSDK_SOURCE}/linker/channel)
include_directories(${ONBOARDSDK_SOURCE}/linker)
include_directories(${ONBOARDSDK_SOURCE}/utils/inc)
include_directories(${ONBOARDSDK_SOURCE}/linker/logger)
include_directories(${ONBOARDSDK_SOURCE}/linker/protocol)
include_directories(${ONBOARDSDK_SOURCE}/linker/root_task)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/logger/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hal)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/osal)

if(NOT LINKER_HEADER_SRC)
    set(LINKER_HEADER_SRC "${ONBOARDSDK_SOURCE}/linker/root_task/*.c \
                            ${ONBOARDSDK_SOURCE}/linker/logger/*.c \
                            ${ONBOARDSDK_SOURCE}/linker/protocol/*.c \
                            ${ONBOARDSDK_SOURCE}/utils/src/*.c* \
                            ${ONBOARDSDK_SOURCE}/linker/channel/*.c \
                            ${ONBOARDSDK_SOURCE}/linker/command/*.c \
                            ${ONBOARDSDK_SOURCE}/platform/src/*.c* \
                            ${ONBOARDSDK_SOURCE}/logger/src/*.c*")
endif()

if(NOT FLIGHT_MODULES_HEADER_DIR)
    set(FLIGHT_MODULES_HEADER_DIR "${ONBOARDSDK_SOURCE}/modules/inc/flight")
endif()

add_subdirectory(camera-gimbal)
#add_subdirectory(flight-control)
add_subdirectory(mfio)
#add_subdirectory(mobile)
add_subdirectory(logging)
add_subdirectory(telemetry)

