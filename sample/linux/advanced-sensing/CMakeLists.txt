cmake_minimum_required(VERSION 2.8)
project(djiosdk-advanced-sensing-sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/osdk-core/cmake-modules/")
set(ADVANCED_SENSING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../osdk-core/advanced-sensing-1.0.0")

find_package(LibUSB REQUIRED)
include_directories(${LIBUSB_1_INCLUDE_DIRS})

# Try to see if user has OpenCV installed
# if yes, default callback will display the image
find_package( OpenCV QUIET )
if (OpenCV_FOUND)
    message( STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
    add_definitions(-DOPEN_CV_INSTALLED)
else()
    message( STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
endif ()

include_directories(${ADVANCED_SENSING_DIR}/inc)
include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)


## First sample
FILE(GLOB SOURCE_FILES
        advanced_sensing_sample.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}
        djiosdk-core
        ${LIBUSB_1_LIBRARIES}
        )

if (OpenCV_FOUND)
    target_link_libraries(${PROJECT_NAME}
            ${OpenCV_LIBS}
            )
endif ()

## Second sample
FILE(GLOB MULTI_THREAD_SAMPLE_SOURCE_FILES
        advanced_sensing_multi_thread_sample.cpp
        utility_thread.cpp
        image_process_container.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
        )

add_executable(djiosdk-advanced-sensing-multi-thread-sample
        ${MULTI_THREAD_SAMPLE_SOURCE_FILES})

target_link_libraries(djiosdk-advanced-sensing-multi-thread-sample
        djiosdk-core
        ${LIBUSB_1_LIBRARIES}
        )

if (OpenCV_FOUND)
    target_link_libraries(djiosdk-advanced-sensing-multi-thread-sample
            ${OpenCV_LIBS}
            )
endif ()
