include(ExternalProject)
message( "External project - GoogleTest" )

if(GTEST)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    enable_testing()
    find_package(Threads)
    ExternalProject_Add(
      googletest
      # Disable update step
      UPDATE_COMMAND ""
      GIT_REPOSITORY https://github.com/google/googletest.git
      CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
        -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
        -DCMAKE_CXX_FLAGS=${MSVC_COMPILER_DEFS}
        -Dgtest_force_shared_crt=${GTEST_FORCE_SHARED_CRT}
        -Dgtest_disable_pthreads=${GTEST_DISABLE_PTHREADS}
      PREFIX ${CMAKE_CURRENT_BINARY_DIR}
      # Disable install step
      INSTALL_COMMAND ""
    )
   ExternalProject_Get_Property(googletest source_dir)
   ExternalProject_Get_Property(googletest binary_dir)

   # Make sure src/bin directory names do not collide
   # with other external projects built within trajectory
   set(gtest_source_dir ${source_dir})
   set(gtest_binary_dir ${binary_dir})

   include_directories("${gtest_source_dir}/googletest/include")
  endif ()
endif ()
