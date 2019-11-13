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
project(djiosdk-telemetry-sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/logger/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/inc)
include_directories(${ONBOARDSDK_SOURCE}/linker)
include_directories(${ONBOARDSDK_SOURCE}/linker/core)
include_directories(${ONBOARDSDK_SOURCE}/linker/core/command)
include_directories(${ONBOARDSDK_SOURCE}/linker/core/channel)
include_directories(${ONBOARDSDK_SOURCE}/linker/core/platform)
include_directories(${ONBOARDSDK_SOURCE}/linker/core/utils/inc)
include_directories(${ONBOARDSDK_SOURCE}/linker/core/logger)
include_directories(${ONBOARDSDK_SOURCE}/linker/core/protocol)
include_directories(${ONBOARDSDK_SOURCE}/linker/core/root_task)
include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)
include_directories(${MODULES_HEADER_DIR})
include_directories(${FLIGHT_MODULES_HEADER_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../hal)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../osal)

FILE(GLOB SOURCE_FILES *.hpp *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../hal/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../osal/*.c
        ${ONBOARDSDK_SOURCE}/linker/core/root_task/*.c
        ${ONBOARDSDK_SOURCE}/linker/core/logger/*.c
        ${ONBOARDSDK_SOURCE}/linker/core/protocol/*.c
        ${ONBOARDSDK_SOURCE}/linker/core/utils/src/*.c*
        ${ONBOARDSDK_SOURCE}/linker/core/channel/*.c
        ${ONBOARDSDK_SOURCE}/linker/core/command/*.c
        ${ONBOARDSDK_SOURCE}/linker/core/platform/*.c
        ${ONBOARDSDK_SOURCE}/platform/src/*.c*
        ${ONBOARDSDK_SOURCE}/logger/src/*.c*
        ${ONBOARDSDK_SOURCE}/utility/src/*.c*
        ${ONBOARDSDK_SOURCE}/api/src/*.c*
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} djiosdk-core)
