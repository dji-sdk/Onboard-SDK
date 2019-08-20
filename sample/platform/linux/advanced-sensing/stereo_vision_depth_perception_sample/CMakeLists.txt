cmake_minimum_required(VERSION 2.8)
project(stereo-vision-depth-perception-sample)

find_path(OPENCV_CONTRIB_IMG_PROC
        NAMES
        disparity_filter.hpp
        PATHS
        /usr/include/opencv2/ximgproc
        /usr/local/include/opencv2/ximgproc
        /opt/local/include/opencv2/ximgproc
        /sw/include/opencv2/ximgproc
        PATH_SUFFIXES
        opencv2/ximgproc
        )

find_path(OPENCV_VIZ
        NAMES
        viz3d.hpp
        PATHS
        /usr/include/opencv2/viz
        /usr/local/include/opencv2/viz
        /opt/local/include/opencv2/viz
        /sw/include/opencv2/viz
        PATH_SUFFIXES
        opencv2/viz
        )

# Try to see if user has OpenCV installed
# If there's version 3.3.0+, depth perception samples will be compiled
# If it's not version 3.3.0+, the sample will not be compiled
find_package( OpenCV 3.3.0 QUIET )
if (OpenCV_FOUND)
    message( "\n${PROJECT_NAME}...")
    message( STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, will use it for depth perception sample")
    message( STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
    add_definitions(-DOPEN_CV_INSTALLED)
    set(OPEN_CV_3_3_0_INSTALLED 1)

    if(NOT OPENCV_CONTRIB_IMG_PROC STREQUAL OPENCV_CONTRIB_IMG_PROC-NOTFOUND)
        message(STATUS "Found ximgproc module in OpenCV, will use it to filter disparity map in depth perception sample")
        add_definitions(-DUSE_OPEN_CV_CONTRIB)
    else()
        message(STATUS "Did not find ximgproc in OpenCV")
    endif()

    if(NOT OPENCV_VIZ STREQUAL OPENCV_VIZ-NOTFOUND)
        message(STATUS "Found viz3d in OpenCV, will use it to visualize point cloud")
        set(FOUND_OPENCV_VIZ TRUE)
    else()
        message(STATUS "Did not find viz3d in OpenCV")
    endif()
else()
    find_package( OpenCV QUIET )
    if (OpenCV_FOUND)
        message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, this sample requires 3.3.0+ for better experience")
        message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
        add_definitions(-DOPEN_CV_INSTALLED)
    else()
        message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
    endif()
endif()


find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_GPU)
elseif()
    message(STATUS "Did not find CUDA in the system")
endif()

set(HELPER_FUNCTIONS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
set(MULTI_THREAD_SAMPLE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../stereo_vision_multi_thread_sample)

include_directories(utility)
include_directories(${MULTI_THREAD_SAMPLE_DIR})

if (OPEN_CV_3_3_0_INSTALLED AND FOUND_OPENCV_VIZ)
    message(STATUS "Found OpenCV ${OpenCV_VERSION} and Viz3d module, stereo vision depth perception sample will be compiled.")

    FILE(GLOB DEPTH_PERCEPTION_SAMPLE_SOURCE_FILES
            ${HELPER_FUNCTIONS_DIR}/dji_linux_environment.cpp
            ${HELPER_FUNCTIONS_DIR}/dji_linux_helpers.cpp
            stereo_vision_depth_perception_sample.cpp
            ${MULTI_THREAD_SAMPLE_DIR}/image_process_container.cpp
            ${MULTI_THREAD_SAMPLE_DIR}/utility_thread.cpp
            stereo_process_container.cpp
            utility/*.cpp
            )

    add_executable(${PROJECT_NAME}
            ${DEPTH_PERCEPTION_SAMPLE_SOURCE_FILES}
            )

    target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

    target_link_libraries(${PROJECT_NAME}
            djiosdk-core
            )

    if (OpenCV_FOUND)
        target_link_libraries(${PROJECT_NAME}
                ${OpenCV_LIBRARIES}
                )
    endif ()
else()
    message(STATUS "Did not find required libraries, stereo vision depth perception sample will not be compiled.")
endif ()
