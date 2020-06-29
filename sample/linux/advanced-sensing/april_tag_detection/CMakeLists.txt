cmake_minimum_required(VERSION 2.8)
project(april-tag-detection)

set(CURRENT_CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)
include(${CURRENT_CMAKE_MODULE_PATH}/Eigen3Config.cmake)
add_definitions(${EIGEN3_DEFINITIONS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# Try to see if user has OpenCV installed
# if yes, default callback will display the image
set(OpenCV_CMAKE_DIR "/home/tao/software/install/opencv-2.4.13/")
find_package( OpenCV 2.4.13 REQUIRED PATHS ${OpenCV_CMAKE_DIR} )
if (OpenCV_FOUND)
  message( STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
  message( STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
  message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
  add_definitions(-DOPEN_CV_INSTALLED)
else()
  message( FATAL_ERROR "Did not find OpenCV in the system...")
endif()

include_directories(/home/tao/work/project/JAV/M210/land_april_tag/april_tag/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(HELPER_FUNCTIONS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
add_executable(${PROJECT_NAME}
  ${HELPER_FUNCTIONS_DIR}/dji_linux_environment.cpp
  ${HELPER_FUNCTIONS_DIR}/dji_linux_helpers.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/AprilTag_TX2.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/polar_coordinates.cpp
  april_tag_detection.cpp
  )
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC djiosdk-core)

target_link_libraries(${PROJECT_NAME} PUBLIC april_tag)

if (OpenCV_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})
endif ()
