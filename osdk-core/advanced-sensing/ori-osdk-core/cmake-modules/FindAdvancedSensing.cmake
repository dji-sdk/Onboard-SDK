# Once done this will be defined
#
#  ADVANCED_SENSING_FOUND - system has Advanced Sensing installed relative to the DJI OSDK Core
#  ADVANCED_SENSING_INCLUDE_DIRS - the Advanced Sensing include directory
#  ADVANCED_SENSING_LIBRARY - Advanced Sensing library
#  ADVANCED_SENSING_DEFINITIONS - Compiler switches required for using Advanced Sensing
#

if (ADVANCED_SENSING_LIBRARY AND ADVANCED_SENSING_INCLUDE_DIRS)
    set(ADVANCED_SENSING_FOUND TRUE)
else (ADVANCED_SENSING_LIBRARY AND ADVANCED_SENSING_INCLUDE_DIRS)
    find_path(ADVANCED_SENSING_INCLUDE_DIRS
            NAMES
            dji_advanced_sensing_protocol.hpp
            PATHS
            ${CMAKE_CURRENT_SOURCE_DIR}/advanced-sensing-2.0.3/inc
            ${EXECUTABLE_OUTPUT_PATH}/advanced-sensing-2.0.3/inc
            PATH_SUFFIXES
            advanced-sensing
            )

    find_library(ADVANCED_SENSING_LIBRARY
            NAMES
            advanced-sensing
            PATHS
            ${CMAKE_CURRENT_SOURCE_DIR}/advanced-sensing-2.0.3/lib
            ${EXECUTABLE_OUTPUT_PATH}/advanced-sensing-2.0.3/lib
            NO_DEFAULT_PATH
            )

    set(ADVANCED_SENSING_INCLUDE_DIRS
            ${ADVANCED_SENSING_INCLUDE_DIRS}
            CACHE STRING "" FORCE)
    set(ADVANCED_SENSING_LIBRARY
            ${ADVANCED_SENSING_LIBRARY}
            CACHE STRING "" FORCE)

    if (ADVANCED_SENSING_INCLUDE_DIRS AND ADVANCED_SENSING_LIBRARY)
        set(ADVANCED_SENSING_FOUND TRUE)
    endif (ADVANCED_SENSING_INCLUDE_DIRS AND ADVANCED_SENSING_LIBRARY)

    if (ADVANCED_SENSING_FOUND)
        if (NOT libadvanced-sensing_FIND_QUIETLY)
            message(STATUS "Found Advanced Sensing:")
            message(STATUS " - Includes: ${ADVANCED_SENSING_INCLUDE_DIRS}")
            message(STATUS " - Libraries: ${ADVANCED_SENSING_LIBRARY}")
        endif (NOT libadvanced-sensing_FIND_QUIETLY)
    else (ADVANCED_SENSING_FOUND)
        if (ADVANCED_SENSING_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find Advanced Sensing")
        endif (ADVANCED_SENSING_FIND_REQUIRED)
    endif (ADVANCED_SENSING_FOUND)

    # Show the ADVANCED_SENSING_INCLUDE_DIRS and ADVANCED_SENSING_LIBRARY variables only in the advanced view
    mark_as_advanced(ADVANCED_SENSING_INCLUDE_DIRS ADVANCED_SENSING_LIBRARY)

endif (ADVANCED_SENSING_LIBRARY AND ADVANCED_SENSING_INCLUDE_DIRS)
