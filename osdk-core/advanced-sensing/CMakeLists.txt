#*  @Copyright (c) 2016-17 DJI.
#*  NOTE THAT this file is part of the advanced sensing
#*  closed-source library. For licensing information,
#*  please visit https://developer.dji.com/policies/eula/
#*

cmake_minimum_required(VERSION 2.8)
project(advanced-sensing)

unset(MY_CPU_ARCH)
if((CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(X86_64)|(amd64)|(AMD64)") AND (CMAKE_SIZEOF_VOID_P EQUAL 8))
    set(MY_CPU_ARCH "AMD64")
elseif((CMAKE_SYSTEM_PROCESSOR MATCHES "i386|i686|x86|AMD64") AND (CMAKE_SIZEOF_VOID_P EQUAL 4))
    set(MY_CPU_ARCH "IA32")
else()
    set(MY_CPU_ARCH "ARM")
endif() # we are not dealing with IA64 or POWERPC

# Detect OS
unset(MY_OS)
if(APPLE)
    set(MY_OS "OSX")
elseif(UNIX)
    set(MY_OS "LINUX") #we don't care BSD
elseif(WIN32)
    set(MY_OS "WIN32")
endif()

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -w -fPIC -Wextra -finline-functions -O3 -fno-strict-aliasing -fvisibility=hidden  -D${MY_CPU_ARCH} -D${MY_OS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(ORI_OSDK_CORE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/ori-osdk-core")
set(NEW_OSDK_CORE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/..")

include_directories(
        ${NEW_OSDK_CORE_SRC}/platform/inc
        ${NEW_OSDK_CORE_SRC}/logger/inc
        ${NEW_OSDK_CORE_SRC}/utility/inc
        ${NEW_OSDK_CORE_SRC}/linker/${ARCH}/inc)

include_directories(${NEW_OSDK_CORE_SRC}/api/inc)
include_directories(${NEW_OSDK_CORE_SRC}/modules/inc/payload)
include_directories(${NEW_OSDK_CORE_SRC}/modules/inc/flight)
include_directories(${NEW_OSDK_CORE_SRC}/modules/inc/mop)
include_directories(${NEW_OSDK_CORE_SRC}/modules/inc/filemgr)
include_directories(${ORI_OSDK_CORE_SRC}/platform/linux/inc)
include_directories(${ORI_OSDK_CORE_SRC}/platform/default/inc)
include_directories(${ORI_OSDK_CORE_SRC}/protocol/inc)
include_directories(${ORI_OSDK_CORE_SRC}/hal/inc)
include_directories(${ORI_OSDK_CORE_SRC}/utility/inc)
include_directories(api/inc)
include_directories(platform/inc)
include_directories(protocol/inc)
include_directories(camera_stream/src)
include_directories(camera_stream/udt/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../v1/protocol/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../v1/api/inc)
##

# Add a cmake file to find libusb
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${ORI_OSDK_CORE_SRC}/cmake-modules/")
find_package(LibUSB REQUIRED)
include_directories(${LIBUSB_1_INCLUDE_DIRS})

find_package(FFMPEG REQUIRED)

if(FFMPEG_FOUND)
    message("Found FFMPEG FFMPEG_INCLUDE_DIR = ${FFMPEG_INCLUDE_DIR}")
    message("Found FFMPEG FFMPEG_LIBRARIES   = ${FFMPEG_LIBRARIES}")
else()
    message("Cannot Find FFMPEG")
endif(FFMPEG_FOUND)
include_directories(${FFMPEG_INCLUDE_DIR})

FILE(GLOB CLOSED_SRC
        api/src/*.cpp
        protocol/src/*.cpp
        platform/src/*.cpp
        camera_stream/src/*.cpp
        camera_stream/udt/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../v1/protocol/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../v1/api/src/*.cpp
        ${ORI_OSDK_CORE_SRC}/protocol/src/*.cpp
        ${ORI_OSDK_CORE_SRC}/hal/src/*.cpp
        ${ORI_OSDK_CORE_SRC}/platform/linux/src/linux_serial_device.cpp
        ${ORI_OSDK_CORE_SRC}/platform/linux/src/posix_thread_manager.cpp
        ${ORI_OSDK_CORE_SRC}/platform/default/src/*.cpp
        ${ORI_OSDK_CORE_SRC}/utility/src/*.cpp
        )
##add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/camera_stream/udt)
##add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/camera_stream/src)

add_library(${PROJECT_NAME} STATIC ${CLOSED_SRC})
target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES} pthread)

set(CLOSED_SOURCE_LIBS ${NEW_OSDK_CORE_SRC}/advanced-sensing-2.0.3)

FILE(GLOB ADVANCED_SENSING_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/api/inc/dji_advanced_sensing.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/api/inc/dji_liveview.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/api/inc/dji_perception.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/inc/*.h*
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/inc/*.h*
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_stream/src/dji_camera_image.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_stream/src/dji_camera_stream.hpp
    ${ORI_OSDK_CORE_SRC}/protocol/inc/dji_aes.hpp
    ${ORI_OSDK_CORE_SRC}/protocol/inc/dji_protocol_base.hpp
    ${ORI_OSDK_CORE_SRC}/hal/inc/dji_hard_driver.hpp
    ${ORI_OSDK_CORE_SRC}/hal/inc/dji_memory.hpp
    ${ORI_OSDK_CORE_SRC}/hal/inc/dji_thread_manager.hpp
    ${ORI_OSDK_CORE_SRC}/platform/linux/inc/posix_thread_manager.hpp
    )

set_target_properties(${PROJECT_NAME} PROPERTIES
        PUBLIC_HEADER "${ADVANCED_SENSING_HEADERS}")

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include" COMPONENT dev
        )
