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
project(djiosdk-activate-sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0 -Wall -Werror")

include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/logger/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/inc)
include_directories(${ONBOARDSDK_SOURCE}/linker/command)
include_directories(${ONBOARDSDK_SOURCE}/linker/channel)
include_directories(${ONBOARDSDK_SOURCE}/linker)
include_directories(${ONBOARDSDK_SOURCE}/utils/inc)
include_directories(${ONBOARDSDK_SOURCE}/linker/logger)
include_directories(${ONBOARDSDK_SOURCE}/linker/protocol)
include_directories(${ONBOARDSDK_SOURCE}/linker/root_task)
include_directories(${MODULES_HEADER_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../hal)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../osal)

FILE(GLOB SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../hal/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../osal/*.c
        ${ONBOARDSDK_SOURCE}/linker/root_task/*.c
        ${ONBOARDSDK_SOURCE}/linker/logger/*.c
        ${ONBOARDSDK_SOURCE}/linker/protocol/*.c
        ${ONBOARDSDK_SOURCE}/utils/src/*.c*
        ${ONBOARDSDK_SOURCE}/linker/channel/*.c
        ${ONBOARDSDK_SOURCE}/linker/command/*.c
        ${ONBOARDSDK_SOURCE}/platform/src/*.c*
        ${ONBOARDSDK_SOURCE}/logger/src/*.c*
        ${ONBOARDSDK_SOURCE}/utility/src/*.c*
        ${ONBOARDSDK_SOURCE}/api/src/*.c*
        )

add_executable(activate_async_sample
        main_async.cpp
        ${SOURCE_FILES})
target_link_libraries(activate_async_sample djiosdk-core)
