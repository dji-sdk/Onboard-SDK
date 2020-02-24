# Once do# Once done this will be defined
#
#  djiosdk-core_FOUND - system has djiosdk-core installed relative to the DJI OSDK Core
#  djiosdk-core_INCLUDE_DIRS - the djiosdk-core include directory
#  djiosdk-core_LIBRARY - djiosdk-core library
#

message(STATUS "Using Finddjiosdk-core.cmake find djiosdk-core")
set(INSTALL_LIB_DIR lib)
set(INSTALL_INCLUDE_DIR include)
set(DEF_INSTALL_CMAKE_DIR　lib/cmake)

find_path(djiosdk-core_INCLUDE_DIRS
            NAMES
            djiosdk
            PATHS
            ${INSTALL_INCLUDE_DIR}
            )

find_library(djiosdk-core_LIBRARY
            NAMES
            djiosdk-core
            PATHS
            ${INSTALL_LIB_DIR}
            )

set(djiosdk-core_INCLUDE_DIRS
    ${djiosdk-core_INCLUDE_DIRS}
    CACHE STRING "" FORCE)
set(djiosdk-core_LIBRARY
    ${djiosdk-core_LIBRARY}
    CACHE STRING "" FORCE)

if ( djiosdk-core_INCLUDE_DIRS AND djiosdk-core_LIBRARY)
    set(djiosdk-core_FOUND TRUE)
endif ()

if (djiosdk-core_FOUND)
    if (NOT libdjiosdk-core_FIND_QUIETLY)
        message(STATUS "Found djiosdk-core:")
        message(STATUS " - Includes: ${djiosdk-core_INCLUDE_DIRS}")
        message(STATUS " - Libraries: ${djiosdk-core_LIBRARY}")
    endif ()
else ()
    if (djiosdk-core_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find djiosdk-core")
    endif ()
endif ()


