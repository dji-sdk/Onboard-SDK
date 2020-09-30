# source of this file
# https://github.com/OpenKinect/libfreenect/blob/master/cmake_modules/Findlibusb-1.0.cmake

# Once done this will define
#
#  LIBUDEV_FOUND - system has libusb
#  LIBUDEV_INCLUDE_DIRS - the libusb include directory
#  LIBUDEV_LIBRARIES - Link these to use libusb
#  LIBUDEV_DEFINITIONS - Compiler switches required for using libusb
#

if (LIBUDEV_LIBRARIES AND LIBUDEV_INCLUDE_DIRS)
    # in cache already
    set(LIBUDEV_FOUND TRUE)
else (LIBUDEV_LIBRARIES AND LIBUDEV_INCLUDE_DIRS)
    find_path(LIBUDEV_INCLUDE_DIR
            NAMES
            libudev.h
            PATHS
            /usr/include
            /usr/local/include
            /opt/local/include
            /sw/include
            PATH_SUFFIXES
            libusb-1.0
            )

    find_library(LIBUDEV_LIBRARY
            NAMES
            udev
            PATHS
            /usr/lib
            /usr/local/lib
            /opt/local/lib
            /sw/lib
            )

    set(LIBUDEV_INCLUDE_DIRS
            ${LIBUDEV_INCLUDE_DIR}
            )
    set(LIBUDEV_LIBRARIES
            ${LIBUDEV_LIBRARY}
            )

    if (LIBUDEV_INCLUDE_DIRS AND LIBUDEV_LIBRARIES)
        set(LIBUDEV_FOUND TRUE)
    endif (LIBUDEV_INCLUDE_DIRS AND LIBUDEV_LIBRARIES)

    if (LIBUDEV_FOUND)
        if (NOT LIBUDEV_FIND_QUIETLY)
            message(STATUS "Found libudev:")
            message(STATUS " - Includes: ${LIBUDEV_INCLUDE_DIRS}")
            message(STATUS " - Libraries: ${LIBUDEV_LIBRARIES}")
        endif (NOT LIBUDEV_FIND_QUIETLY)
    else (LIBUDEV_FOUND)
        if (LIBUDEV_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find libusb")
        endif (LIBUDEV_FIND_REQUIRED)
    endif (LIBUDEV_FOUND)

    # show the LIBUDEV_INCLUDE_DIRS and LIBUDEV_LIBRARIES variables only in the advanced view
    mark_as_advanced(LIBUDEV_INCLUDE_DIRS LIBUDEV_LIBRARIES)

endif (LIBUDEV_LIBRARIES AND LIBUDEV_INCLUDE_DIRS)