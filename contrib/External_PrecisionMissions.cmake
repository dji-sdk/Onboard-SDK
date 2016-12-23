include(ExternalProject)
message( "External project - DJI Precision Missions" )

set(VERSION "1.0.2")

if((${TARGET_ARCH} EQUAL "x86_64") AND (${DISTRO_VERSION} EQUAL "1404"))
  message("Warning! Precision Trajectory Features do not work in Ubuntu 14.04 on x86.\
     Please upgrade to Ubuntu 16.04 to use these features. Your program WILL SEGFAULT \
     if you try to run precision trajectories.")
endif()

ExternalProject_Add(PrecisionMissions
  GIT_REPOSITORY https://github.com/dji-sdk/Onboard-SDK-Resources.git
  BINARY_DIR PrecisionMissions
  GIT_TAG precision-missions-${VERSION}
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CMAKE_COMMAND ""
  INSTALL_COMMAND ""
  )

ExternalProject_Get_Property(PrecisionMissions source_dir)

# Make sure src/bin directory names do not collide
# with other external projects
set(precision_missions_source_dir ${source_dir}/precision-missions-${VERSION})
include_directories("${precision_missions_source_dir}/inc")

