project(djiosdk-core)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING
            "Choose the type of build Debug/Release/Coverage:"
            FORCE)
endif ()

## Set compiler flags accordingly to the build type
if (MSVC)
  include_directories(${Qt5Widgets_INCLUDES})
  add_definitions(${Qt5Widgets_DEFINITIONS})
  set(COMMON_CXX_FLAGS "-std=c++11 ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
endif (MSVC)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  set(COMMON_CXX_FLAGS "-std=c++11 -pthread")
endif ()

if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
endif ()

if (COVERAGE)
  SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage -fPIC")
  SET(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${GCC_COVERAGE_COMPILE_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Release")
  set(CMAKE_CXX_FLAGS_RELASE "${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif ()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  inc
  test
  ../platform/linux/inc
)

## Declare a C++ library
FILE(GLOB DJI_SDK_LIB_SOURCES src/*.cpp inc/*.h)
add_library(${PROJECT_NAME}
  STATIC
  ${DJI_SDK_LIB_SOURCES})

INSTALL(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${EXECUTABLE_OUTPUT_PATH}
  LIBRARY DESTINATION ${EXECUTABLE_OUTPUT_PATH}
  ARCHIVE DESTINATION ${EXECUTABLE_OUTPUT_PATH}
  CONFIGURATIONS Debug
  COMPONENT library)

INSTALL(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${EXECUTABLE_OUTPUT_PATH}
  LIBRARY DESTINATION ${EXECUTABLE_OUTPUT_PATH}
  ARCHIVE DESTINATION ${EXECUTABLE_OUTPUT_PATH}
  CONFIGURATIONS Release
  COMPONENT library)

if(GTEST)
	## Add gtest based cpp test target and link libraries
	if (CMAKE_SYSTEM_NAME MATCHES Linux)
	#    FILE(GLOB DJI_SDK_LIB_TEST_SOURCES test/*.cpp test/*.h)
	#    add_executable(DJI_APITest ${DJI_SDK_LIB_TEST_SOURCES})
    add_executable(DJI_APITest
      test/DJI_APITest.cpp
      test/DJI_CameraTest.cpp
      test/DJI_Environment.cpp
      test/DJI_HotPointTest.cpp
      test/DJI_Main.cpp
      test/DJI_WayPointTest.cpp
      test/DJI_FlightTest.cpp
      test/DJI_VirtualRCTest.cpp
    )
    add_dependencies(DJI_APITest googletest)
    add_dependencies(DJI_APITest rapidjson)

    # Link with platform specific impl.
    target_link_libraries(DJI_APITest djiosdk-platform)
  
    if (TARGET DJI_APITest)
      if (MSVC)
          qt5_use_modules(DJI_APITest Core SerialPort)
          target_link_libraries(DJI_APITest
                  ${PROJECT_NAME}
                  ${GTEST_LIBRARIES}
                  ${Qt5Widgets_LIBRARIES})
      endif (MSVC)
      if (CMAKE_SYSTEM_NAME MATCHES Linux)
          target_link_libraries(DJI_APITest
                  ${PROJECT_NAME}
                  ${binary_dir}/googlemock/gtest/libgtest.a
                  ${binary_dir}/googlemock/gtest/libgtest_main.a
                  ${RAPIDJSON_LIBRARIES}
                  pthread)
      endif ()
    endif ()

    add_test(${EXECUTABLE_OUTPUT_PATH}/DJI_APITest ${EXECUTABLE_OUTPUT_PATH}/DJI_APITest)
	endif ()
endif()
