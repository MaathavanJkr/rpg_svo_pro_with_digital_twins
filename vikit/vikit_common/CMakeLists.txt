cmake_minimum_required(VERSION 3.16)
project(vikit_common)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

set(HEADERS
  include/vikit/blender_utils.h
  include/vikit/math_utils.h
  include/vikit/performance_monitor.h
  include/vikit/ringbuffer.h
  include/vikit/sample.h
  include/vikit/timer.h
  include/vikit/user_input_thread.h
  include/vikit/vision.h
  include/vikit/homography_decomp.h
  include/vikit/homography.h
  include/vikit/path_utils.h
  include/vikit/csv_utils.h
  include/aslam/common/entrypoint.h
  include/aslam/common/macros.h
  include/aslam/common/memory.h
  include/aslam/common/pose-types.h
  include/aslam/common/yaml-serialization.h
  include/aslam/common/yaml-serialization-eigen.h
  )

set(SOURCES
  src/homography.cpp
  src/math_utils.cpp
  src/performance_monitor.cpp
  src/sample.cpp
  src/user_input_thread.cpp
  src/vision.cpp
  )

# Create vikit shared library
ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${SVO_SYSTEM_LIBRARIES})

################################################################################
# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(homography test/test_homography.cpp)
  target_link_libraries(homography ${PROJECT_NAME})
  svo_link_test_deps(homography)

  ament_add_gtest(math_utils test/test_math_utils.cpp)
  target_link_libraries(math_utils ${PROJECT_NAME})
  svo_link_test_deps(math_utils)

  # Other tests
  add_executable(vikit_test_timer test/test_timer.cpp)
  target_link_libraries(vikit_test_timer ${PROJECT_NAME})

  add_executable(vikit_test_common test/test_common.cpp)
  target_link_libraries(vikit_test_common ${PROJECT_NAME})
endif()

################################################################################
ament_auto_package()
