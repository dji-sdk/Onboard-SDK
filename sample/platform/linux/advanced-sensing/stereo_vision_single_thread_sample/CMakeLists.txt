cmake_minimum_required(VERSION 2.8)
project(stereo-vision-single-thread-sample)

# Try to see if user has OpenCV installed
# if yes, default callback will display the image
find_package( OpenCV QUIET )
if (OpenCV_FOUND)
    message( "\n${PROJECT_NAME}...")
    message( STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
    message( STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
    add_definitions(-DOPEN_CV_INSTALLED)
else()
    message( STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
endif ()

set(HELPER_FUNCTIONS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
add_executable(${PROJECT_NAME}
        ${HELPER_FUNCTIONS_DIR}/dji_linux_environment.cpp
        ${HELPER_FUNCTIONS_DIR}/dji_linux_helpers.cpp
        stereo_vision_single_thread_sample.cpp
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
