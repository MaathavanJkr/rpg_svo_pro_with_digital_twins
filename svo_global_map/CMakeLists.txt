cmake_minimum_required(VERSION 3.16)
project(svo_global_map)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(GTSAMCMakeTools REQUIRED)
find_package(GTSAM REQUIRED)
message(STATUS ${GTSAM_INCLUDE_DIR})
message(STATUS ${GTSAM_LIBRARIES})

include(SvoSetup)
include_directories(${GTSAM_INCLUDE_DIR})

set(HEADERS
  # gtsam interface
  include/svo/gtsam/gtsam_optimizer.h
  include/svo/gtsam/graph_manager.h
  include/svo/gtsam/smart_factors_fwd.h
  include/svo/gtsam/camera_bearing_factor.h
  include/svo/gtsam/camera_bearing_extrinsics_factor.h
  include/svo/global_map.h
  include/svo/null_stream.h
  )

set(SOURCES
  # gtsam interface
  src/gtsam/gtsam_optimizer.cpp
  src/gtsam/graph_manager.cpp
  src/global_map.cpp
  src/null_stream.cpp
  )

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} gtsam ${SVO_SYSTEM_LIBRARIES})

ament_auto_package()
