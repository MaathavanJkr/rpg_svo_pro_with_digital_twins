cmake_minimum_required(VERSION 3.16)
project(svo)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)
find_package(GeographicLib REQUIRED)

list(APPEND SVO_SOURCEFILES
  src/frame_handler_mono.cpp
  src/frame_handler_stereo.cpp
  src/frame_handler_array.cpp
  src/frame_handler_base.cpp
  src/map.cpp
  src/pose_optimizer.cpp
  src/initialization.cpp
  src/reprojector.cpp
  src/imu_handler.cpp
  src/global_positions_handler.cpp
  src/global_pose_handler.cpp
  src/stereo_triangulation.cpp
  src/io.cpp
)

list(APPEND SVO_HEADERFILES
  include/svo/abstract_bundle_adjustment.h
  include/svo/frame_handler_base.h
  include/svo/frame_handler_mono.h
  include/svo/frame_handler_stereo.h
  include/svo/frame_handler_array.h
  include/svo/global.h
  include/svo/imu_handler.h
  include/svo/global_positions_handler.h
  include/svo/global_pose_handler.h
  include/svo/initialization.h
  include/svo/map.h
  include/svo/pose_optimizer.h
  include/svo/reprojector.h
  include/svo/stereo_triangulation.h
  include/svo/io.h
  include/svo/svo.h
  )

################################################################################
# Create svo library
ament_auto_add_library(svo SHARED ${SVO_SOURCEFILES} ${SVO_HEADERFILES})
target_include_directories(svo SYSTEM PUBLIC ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(svo ${GeographicLib_LIBRARIES} ${SVO_SYSTEM_LIBRARIES})

################################################################################
###
### GTEST
###
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test-frame test/test_frame.cpp)
  target_link_libraries(${PROJECT_NAME}-test-frame ${PROJECT_NAME})
  svo_link_test_deps(${PROJECT_NAME}-test-frame)
endif()

################################################################################
ament_auto_package()
