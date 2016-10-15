cmake_minimum_required(VERSION 2.8)
project(vlp16lidar-driver)

if (COVERAGE)
  set(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage -fPIC")
  set(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
endif ()

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

# Try to find liblas in your system (install if does not exist)
find_package(LibLAS QUIET)
if (NOT LIBLAS_FOUND)
  message(STATUS " liblas cannot be found. Installing liblas-dev package")
  execute_process(COMMAND apt-get install liblas-dev)
  # liblas must be available now
  find_package(LibLAS REQUIRED)
endif()

# Try to find VTK in your system (install if does not exist)
set(VTK_CMAKE_PATHS /lib/cmake /usr/lib/cmake /usr/local/lib/cmake /opt/VTK-7.0.0/lib/cmake)
find_package(VTK 7.0 QUIET 
  HINTS ${VTK_CMAKE_PATHS}
  NO_DEFAULT_PATH)

if(NOT VTK_FOUND)
  # Download and installation will take up to 15min
  include(External_VTK.cmake)
  find_package(VTK 7.0 REQUIRED)
endif()
include(${VTK_USE_FILE})
message(STATUS " VTK include dir set to ${VTK_INCLUDE_DIRS}")

# Set compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

include_directories(inc
  ${PCAP_ICLUDE_DIR}  
  ${Boost_INCLUDE_DIRS}
  ${LIBLAS_INCLUDE_DIR})

# Build lidar_APIs_LIB into a static libraary
FILE(GLOB LIB_SOURCE_FILES src/*.cxx src/*.cpp inc/*.h)

add_library(${PROJECT_NAME} STATIC ${LIB_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${LIBLAS_LIBRARY} ${PCAP_LIBRARY} boost_thread boost_system ${VTK_LIBRARIES})

