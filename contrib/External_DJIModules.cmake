if(USE_PRECISION_MISSIONS)
  add_definitions(-DUSE_PRECISION_MISSIONS)
  include(${CMAKE_MODULE_PATH}/External_PrecisionMissions.cmake)
endif()

if(USE_COLLISION_AVOIDANCE)
  include(${CMAKE_MODULE_PATH}/External_CollisionAvoidance.cmake)
endif()

if(USE_POINTCLOUD2LAS)
  include(${CMAKE_MODULE_PATH}/External_Pointcloud2Las.cmake)
endif()

