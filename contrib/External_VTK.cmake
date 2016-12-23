include(ExternalProject)
message( "External project - VTK" )

IF(CMAKE_EXTRA_GENERATOR)
  SET(cmake_gen "${CMAKE_EXTRA_GENERATOR} - ${CMAKE_GENERATOR}")
ELSE()
  SET(cmake_gen "${CMAKE_GENERATOR}")
ENDIF()

# Try to find VTK in your system (install if does not exist)
set(VTK_CMAKE_PATHS /lib/cmake /usr/lib/cmake /usr/local/lib/cmake /opt/VTK-7.0.0/lib/cmake)
find_package(VTK 7.0 QUIET 
  HINTS ${VTK_CMAKE_PATHS}
  NO_DEFAULT_PATH)

if(NOT VTK_FOUND)
ExternalProject_Add(VTK
  GIT_REPOSITORY http://vtk.org/VTK.git
  GIT_TAG v7.0.0
  SOURCE_DIR VTK
  BINARY_DIR VTK-build
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CMAKE_GENERATOR ${cmake_gen}
  CMAKE_ARGS
    -DBUILD_EXAMPLES:BOOL=OFF
    -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
    -DBUILD_TESTING:BOOL=OFF
    -DCMAKE_BUILD_TYPE:STRING=${BUILD_TYPE}
    -DVTK_BUILD_ALL_MODULES:BOOL=OFF
    -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DEPENDENCIES_DIR}
  )
endif()

# Find VTK in your system 
set(VTK_CMAKE_PATHS /lib/cmake /usr/lib/cmake /usr/local/lib/cmake /opt/VTK-7.0.0/lib/cmake)
find_package(VTK 7.0 QUIET 
  HINTS ${VTK_CMAKE_PATHS}
  NO_DEFAULT_PATH)

