#                                               -*- cmake -*-
#
#  Eigen3Config.cmake(.in)

# Use the following variables to compile and link against Eigen:
#  EIGEN3_FOUND              - True if Eigen was found on your system
#  EIGEN3_USE_FILE           - The file making Eigen usable
#  EIGEN3_DEFINITIONS        - Definitions needed to build with Eigen
#  EIGEN3_INCLUDE_DIR        - Directory where signature_of_eigen3_matrix_library can be found
#  EIGEN3_INCLUDE_DIRS       - List of directories of Eigen and it's dependencies
#  EIGEN3_ROOT_DIR           - The base directory of Eigen
#  EIGEN3_VERSION_STRING     - A human-readable string containing the version
#  EIGEN3_VERSION_MAJOR      - The major version of Eigen
#  EIGEN3_VERSION_MINOR      - The minor version of Eigen
#  EIGEN3_VERSION_PATCH      - The patch version of Eigen

set(EIGEN3_FOUND 1)
set(EIGEN3_USE_FILE "${CMAKE_CURRENT_LIST_DIR}/UseEigen3.cmake")

set(EIGEN3_DEFINITIONS "")
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
set(EIGEN3_INCLUDE_DIRS "/usr/include/eigen3")
set(EIGEN3_ROOT_DIR "/usr")

set(EIGEN3_VERSION_STRING "3.2.92")
set(EIGEN3_VERSION_MAJOR "3")
set(EIGEN3_VERSION_MINOR "2")
set(EIGEN3_VERSION_PATCH "92")
