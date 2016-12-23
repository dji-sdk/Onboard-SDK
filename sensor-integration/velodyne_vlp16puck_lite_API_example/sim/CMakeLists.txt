cmake_minimum_required(VERSION 2.8)
project(Simulator)

if (COVERAGE)
  set(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage -fPIC")
  set(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
endif ()

# Add module path for Find<name>.cmake conf files
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../contrib)

# Find QtCore library
find_package(Qt5Core REQUIRED)

# Try to find PCAP in your system (install if does not exist)
find_package(PCAP QUIET)
if(NOT PCAP_FOUND)
  message(STATUS " Pcap cannot be found. Installing libpcap-dev package")
  execute_process(COMMAND apt-get install libpcap-dev)
  # PCAP must be available now
  find_package(PCAP REQUIRED)
endif()

# Try to find Boost in your system (install if does not exist)
find_package(Boost COMPONENTS thread system QUIET)
if (NOT Boost_FOUND)
  message(STATUS " Boost cannot be found. Installing libboost-all-dev package")
  execute_process(COMMAND apt-get install libboost-all-dev)
  # Boost must be available now
  find_package(Boost COMPONENTS thread system REQUIRED)
endif()
message(STATUS " Boost include dir set to ${Boost_INCLUDE_DIRS}")

# Make these available for the user to set
mark_as_advanced(CLEAR Boost_INCLUDE_DIR) 
mark_as_advanced(CLEAR Boost_LIBRARY_DIRS) 

# Find VTK in your system 
set(VTK_CMAKE_PATHS 
  /lib/cmake 
  /usr/lib/cmake 
  /usr/local/lib/cmake 
  /opt/VTK-7.0.0/lib/cmake)

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

include_directories(${PCAP_ICLUDE_DIR}  
  ${Boost_INCLUDE_DIRS})

# Buid Velodyne LiDAR Simulator
FILE(GLOB SIMULATOR_SOURCE_FILES src/*.cxx src/*.cpp inc/*.h)
include_directories(inc)

add_executable(${PROJECT_NAME} ${SIMULATOR_SOURCE_FILES})

add_dependencies(${PROJECT_NAME} VTK)
if(VTK_FOUND)
  include(${VTK_USE_FILE})
  message(STATUS " VTK include dir set to ${VTK_INCLUDE_DIRS}")
endif()

target_link_libraries(${PROJECT_NAME} ${PCAP_LIBRARY} boost_thread boost_system ${VTK_LIBRARIES} Qt5::Core)

