cmake_minimum_required(VERSION 2.8)
project(stereo-vision-m300-sample)

set(HELPER_FUNCTIONS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
add_executable(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${HELPER_FUNCTIONS_DIR}/dji_linux_environment.cpp
  ${HELPER_FUNCTIONS_DIR}/dji_linux_helpers.cpp

  main.cpp
  )

target_link_libraries(${PROJECT_NAME}
  djiosdk-core
  )
