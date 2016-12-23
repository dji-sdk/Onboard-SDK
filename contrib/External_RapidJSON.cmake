include(ExternalProject)
message("External project - RapidJSON")

ExternalProject_Add(
  rapidjson
  # Disable update step
  UPDATE_COMMAND ""
  GIT_REPOSITORY https://github.com/miloyip/rapidjson.git
  CMAKE_ARGS -DRAPIDJSON_BUILD_TESTS=OFF
    -DRAPIDJSON_BUILD_DOC=OFF
    -DRAPIDJSON_BUILD_EXAMPLES=OFF
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}
  # Disable install step
  INSTALL_COMMAND ""
  TEST_COMMAND ""
  # UPDATE_DISCONNECTED
  )

ExternalProject_Get_Property(rapidjson source_dir)

# Make sure src/bin directory names do not collide
# with other external projects
set(rapidjson_source_dir ${source_dir})
include_directories("${rapidjson_source_dir}/include")

