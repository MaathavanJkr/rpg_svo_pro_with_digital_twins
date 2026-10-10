cmake_minimum_required(VERSION 3.16)
project(svo_common)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

set(HEADERS
  include/svo/common/camera.h
  include/svo/common/camera_fwd.h
  include/svo/common/feature_wrapper.h
  include/svo/common/frame.h
  include/svo/common/imu_calibration.h
  include/svo/common/global_positions_settings.h
  include/svo/common/logging.h
  include/svo/common/occupancy_grid_2d.h
  include/svo/common/point.h
  include/svo/common/seed.h
  include/svo/common/transformation.h
  include/svo/common/types.h
  )

set(SOURCES
  src/frame.cpp
  src/point.cpp
  )

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${SVO_SYSTEM_LIBRARIES})

################################################################################
# TESTS
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/test_feature_wrapper.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} pthread)
  svo_link_test_deps(${PROJECT_NAME}-test)

  ament_add_gtest(container-helpers-test
    test/test_container_helpers.cpp
  )
  target_link_libraries(container-helpers-test ${PROJECT_NAME})
  svo_link_test_deps(container-helpers-test)
endif()

################################################################################
ament_auto_package()
