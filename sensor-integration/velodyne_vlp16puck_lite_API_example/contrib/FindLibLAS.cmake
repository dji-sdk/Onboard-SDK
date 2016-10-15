set(EXTERNAL_ROOT_DIR /usr/include /usr/local/include /usr/lib /usr/local/lib)

# Find liblas
find_path(LIBLAS_INCLUDE_DIR
  liblas.hpp
  PATH_PREFIXES liblas
  HINTS ${EXTERNAL_ROOT_DIR}
  NO_DEFAULT_PATH)

set(LIBLAS_NAMES "liblas.so")
find_library(LIBLAS_LIBRARY
  NAMES ${LIBLAS_NAMES}
  HINTS ${EXTERNAL_ROOT_DIR}
  NO_DEFAULT_PATH)

# Make these available for the user to set
mark_as_advanced(CLEAR LIBLAS_INCLUDE_DIR)
mark_as_advanced(CLEAR LIBLAS_LIBRARY_FILE)

if(LIBLAS_INCLUDE_DIR)
  message(STATUS " LibLAS include dir set to ${LIBLAS_INCLUDE_DIR}")
  set(LIBLAS_FOUND "YES")
endif()

if(LIBLAS_LIBRARY)
  message(STATUS " LibLAS include dir set to ${LIBLAS_LIBRARY}")
endif()

