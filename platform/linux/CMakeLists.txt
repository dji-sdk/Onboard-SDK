cmake_minimum_required(VERSION 2.8)
project(djiosdk-platform)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

include_directories(../../osdk-core/inc)
include_directories(inc)

FILE(GLOB SOURCE_FILES inc/*.h src/*.cpp)
add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} djiosdk-core)

