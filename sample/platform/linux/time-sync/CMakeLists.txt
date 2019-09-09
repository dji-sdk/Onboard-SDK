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
project(time-sync-sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)
include_directories(${MODULES_HEADER_DIR})
include_directories(${FLIGHT_MODULES_HEADER_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)

add_executable(time_sync_callback_sample
    time_sync_callback_sample.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
    )
target_link_libraries(time_sync_callback_sample djiosdk-core)

add_executable(time_sync_poll_sample
    time_sync_poll_sample.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
    )
target_link_libraries(time_sync_poll_sample djiosdk-core)