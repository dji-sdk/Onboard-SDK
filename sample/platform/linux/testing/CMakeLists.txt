# cd ~/Documents/Onboard-SDK/build/ &&  make clean && cmake .. && make && cd ~/Documents/Onboard-SDK/build/bin/ && ./djiosdk-testing

cmake_minimum_required(VERSION 2.8)
project(djiosdk-testing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

FILE(GLOB SOURCE_FILES 
        # *.hpp 
        *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../hal/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../osal/*.c

        # for flight control, no need to supply source cos alr compiled in the previous package before this
        # ${CMAKE_CURRENT_SOURCE_DIR}/../flight-control/flight_control_sample.cpp

        # for euler angle function
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../core/src/flight_sample.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../core/inc
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    djiosdk-fuzzer
)