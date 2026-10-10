cmake_minimum_required(VERSION 3.16)
project(svo_tracker)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

set(HEADERS
  include/svo/tracker/feature_tracking_types.h
  include/svo/tracker/feature_tracking_utils.h
  include/svo/tracker/feature_tracking_viz.h
  include/svo/tracker/feature_tracker.h
  )

set(SOURCES
  src/feature_tracking_types.cpp
  src/feature_tracking_utils.cpp
  src/feature_tracking_viz.cpp
  src/feature_tracker.cpp
  )

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${SVO_SYSTEM_LIBRARIES})

################################################################################
ament_auto_package()
