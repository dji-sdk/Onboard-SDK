# Once done this will be defined
#
#  WAYPT2_CORE_FOUND - system has Waypt2 core installed relative to the DJI OSDK Core
#  WAYPT2_CORE_INCLUDE_DIRS - the Waypt2 core include directory
#  WAYPT2_CORE_LIBRARY - Waypt2 core library
#  WAYPT2_CORE_DEFINITIONS - Compiler switches required for using Waypt2 core
#

if (WAYPT2_CORE_LIBRARY AND WAYPT2_CORE_INCLUDE_DIRS)
  set(WAYPT2_CORE_FOUND TRUE)
else (WAYPT2_CORE_LIBRARY AND WAYPT2_CORE_INCLUDE_DIRS)
  find_path(WAYPT2_CORE_INCLUDE_DIRS
      NAMES
      dji_waypointv2_interface.hpp
      PATHS
      ${CMAKE_CURRENT_SOURCE_DIR}/waypointv2-core-1.0.0/inc
      ${EXECUTABLE_OUTPUT_PATH}/waypointv2-core-1.0.0/inc
      )

  find_library(WAYPT2_INTERFACE_LIBRARY
      NAMES
      waypointv2-interface
      PATHS
      ${CMAKE_CURRENT_SOURCE_DIR}/waypointv2-core-1.0.0/lib
      ${EXECUTABLE_OUTPUT_PATH}/waypointv2-core-1.0.0/lib
      NO_DEFAULT_PATH
      )

  find_library(WAYPT2_CORE_LIBRARY
      NAMES
      waypointv2-core
      PATHS
      ${CMAKE_CURRENT_SOURCE_DIR}/waypointv2-core-1.0.0/lib
      ${EXECUTABLE_OUTPUT_PATH}/waypointv2-core-1.0.0/lib
      NO_DEFAULT_PATH
      )

  find_library(SDK_COMMON_LIBRARY
      NAMES
      djisdk-common
      PATHS
      ${CMAKE_CURRENT_SOURCE_DIR}/waypointv2-core-1.0.0/lib
      ${EXECUTABLE_OUTPUT_PATH}/waypointv2-core-1.0.0/lib
      NO_DEFAULT_PATH
      )

  find_library(DJI_PROTOBUF_LIBRARY
      NAMES
      DJIProtobuf
      PATHS
      ${CMAKE_CURRENT_SOURCE_DIR}/waypointv2-core-1.0.0/lib
      ${EXECUTABLE_OUTPUT_PATH}/waypointv2-core-1.0.0/lib
      NO_DEFAULT_PATH
      )

  set(WAYPT2_CORE_INCLUDE_DIRS
      ${WAYPT2_CORE_INCLUDE_DIRS}
      CACHE STRING "" FORCE)
  set(WAYPT2_INTERFACE_LIBRARY
      ${WAYPT2_INTERFACE_LIBRARY}
      CACHE STRING "" FORCE)
  set(WAYPT2_CORE_LIBRARY
      ${WAYPT2_CORE_LIBRARY}
      CACHE STRING "" FORCE)
  set(SDK_COMMON_LIBRARY
      ${SDK_COMMON_LIBRARY}
      CACHE STRING "" FORCE)
  set(DJI_PROTOBUF_LIBRARY
      ${DJI_PROTOBUF_LIBRARY}
      CACHE STRING "" FORCE)


  if (WAYPT2_CORE_INCLUDE_DIRS AND WAYPT2_CORE_LIBRARY)
    if(SDK_COMMON_LIBRARY AND DJI_PROTOBUF_LIBRARY)
      if(WAYPT2_INTERFACE_LIBRARY)
        set(WAYPT2_CORE_FOUND TRUE)
      endif(WAYPT2_INTERFACE_LIBRARY)
    endif(SDK_COMMON_LIBRARY AND DJI_PROTOBUF_LIBRARY)
  endif (WAYPT2_CORE_INCLUDE_DIRS AND WAYPT2_CORE_LIBRARY)

  if (WAYPT2_CORE_FOUND)
    if (NOT libDJIWaypointV2Core_FIND_QUIETLY)
      message(STATUS "Found Waypoint2 Core:")
      message(STATUS " - Includes: ${WAYPT2_CORE_INCLUDE_DIRS}")
      message(STATUS " - Libraries: ${WAYPT2_INTERFACE_LIBRARY}")
      message(STATUS " - Libraries: ${WAYPT2_CORE_LIBRARY}")
      message(STATUS " - Libraries: ${SDK_COMMON_LIBRARY}")
      message(STATUS " - Libraries: ${DJI_PROTOBUF_LIBRARY}")
    endif (NOT libDJIWaypointV2Core_FIND_QUIETLY)
  else (WAYPT2_CORE_FOUND)
    if (WAYPT2_CORE_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find Waypoint2 Core")
    endif (WAYPT2_CORE_FIND_REQUIRED)
  endif (WAYPT2_CORE_FOUND)

  # Show the WAYPT2_CORE_INCLUDE_DIRS and WAYPT2_CORE_LIBRARY variables only in the advanced view
  mark_as_advanced(WAYPT2_CORE_INCLUDE_DIRS WAYPT2_CORE_LIBRARY)

endif (WAYPT2_CORE_LIBRARY AND WAYPT2_CORE_INCLUDE_DIRS)
