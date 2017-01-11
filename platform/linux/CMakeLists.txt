cmake_minimum_required(VERSION 2.8)
project(djiosdk-platform)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILER} -pthread -g -O0")

include_directories(../../osdk-core/inc)
include_directories(inc)

FILE(GLOB SOURCE_FILES inc/*.h src/*.cpp)
add_library(${PROJECT_NAME} ${SOURCE_FILES})

if(${TARGET_ARCH} MATCHES "arm" )
  set(LINKING_FLAGS rt)
endif()

target_link_libraries(${PROJECT_NAME} djiosdk-core ${LINKING_FLAGS})

