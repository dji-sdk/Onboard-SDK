include(ExternalProject)
message( "External project - DJI Pointcloud to LAS Converter" )

set(VERSION "1.0.0")
set(PROJECT_NAME dji-ros-pointcloud2las)
set(PKG_NAME pointcloud2las)

ExternalProject_Add(${PROJECT_NAME}
  GIT_REPOSITORY https://github.com/dji-sdk/Onboard-SDK-Resources.git 
  BINARY_DIR ${PROJECT_NAME}/${PKG_NAME}
  GIT_TAG ${PKG_NAME}-${VERSION}
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CMAKE_COMMAND ""
  INSTALL_COMMAND ""
  )

ExternalProject_Get_Property(${PROJECT_NAME} source_dir)

add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD 
  COMMAND ${CMAKE_COMMAND} -E make_directory ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}
  )

add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E tar xzf ${source_dir}/${PKG_NAME}-${VERSION}.tar.gz
  WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}
  )

