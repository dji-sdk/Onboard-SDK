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


#Detecting target architecture to decide which precompiled library to link against

if((CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64") AND (CMAKE_SIZEOF_VOID_P EQUAL 8))
  set(TARGET_ARCH "x86_64")
elseif((CMAKE_SYSTEM_PROCESSOR MATCHES "i386|i686|x86|AMD64") AND (CMAKE_SIZEOF_VOID_P EQUAL 4))
  set(TARGET_ARCH "x86")
elseif((CMAKE_SYSTEM_PROCESSOR MATCHES "^arm*") OR (CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch*"))
  set(TARGET_ARCH "arm")
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(PROC_VERSION "v7")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PROC_VERSION "v8")
  endif()
endif()

#Detect platform - from https://gist.github.com/CoolerVoid/1781717

#### DEPRECATED BEHAVIOR ####
if(USE_PRECISION_MISSIONS OR USE_COLLISION_AVOIDANCE OR USE_POINTCLOUD2LAS)
  message(DEPRECATION "DJI external modules are no longer supported!")
  EXECUTE_PROCESS(
    COMMAND cat /etc/lsb-release
    COMMAND grep DISTRIB_RELEASE
    COMMAND awk -F= "{ print $2 }"
    COMMAND tr "\n" " "
    COMMAND sed "s/ //"
    OUTPUT_VARIABLE LSB_VER
    )

  if( ${LSB_VER} STREQUAL "16.04")
    set(DISTRO_VERSION 1604)
  elseif(${LSB_VER} STREQUAL "14.04")
    set(DISTRO_VERSION 1404)
  elseif(${LSB_VER} STREQUAL "18")
    set(DISTRO_VERSION 1604)
  else()
    set(DISTRO_VERSION UNKNOWN)
  endif()
endif()
