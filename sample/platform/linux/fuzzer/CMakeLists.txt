# cd ~/Documents/Onboard-SDK/build/ &&  make clean && cmake .. && make && cd ~/Documents/Onboard-SDK/build/bin/ && ./djiosdk-testing

cmake_minimum_required(VERSION 2.8)
project(djiosdk-fuzzer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

FILE(GLOB SOURCE_FILES 
    *.hpp 
    *.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../../../osdk-core/api/src/dji_flight_controller.cpp
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../../../osdk-core/api/inc # mode struct
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../../../osdk-core/modules/inc/flight # enums
)

# target_link_libraries(
#     ${PROJECT_NAME}
#     PUBLIC
#     djiosdk-core
#     djiosdk-flightcontrol-sample  
# )