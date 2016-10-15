include(ExternalProject)
message("External project RapidJSON")

if(RapidJSON)
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
    include_directories("${source_dir}/include")
endif()

