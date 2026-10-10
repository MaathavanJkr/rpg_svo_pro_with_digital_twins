cmake_minimum_required(VERSION 3.16)
project(svo_direct)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

set(HEADERS
  include/svo/direct/patch_warp.h
  include/svo/direct/patch_score.h
  include/svo/direct/patch_utils.h
  include/svo/direct/feature_alignment.h
  include/svo/direct/feature_detection_types.h
  include/svo/direct/feature_detection.h
  include/svo/direct/feature_detection_utils.h
  include/svo/direct/matcher.h
  include/svo/direct/depth_filter.h
  include/svo/direct/depth_estimation.h
  include/svo/direct/elder_zucker.h
)

set(SOURCES
  src/patch_warp.cpp
  src/feature_alignment.cpp
  src/feature_detection.cpp
  src/feature_detection_utils.cpp
  src/matcher.cpp
  src/depth_filter.cpp
  src/depth_estimation.cpp
  src/elder_zucker.cpp
  )

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${SVO_SYSTEM_LIBRARIES})

################################################################################
# TESTS
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  foreach(test_name svo_test_patch_score svo_test_edgelet_direction
          svo_test_edgelet_extraction svo_test_feature_alignment)
    string(REPLACE "svo_test_" "test/test_" test_src ${test_name})
    add_executable(${test_name} ${test_src}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME} ament_index_cpp::ament_index_cpp)
  endforeach()

  ament_add_gtest(${PROJECT_NAME}-test-depth-estimation
    test/test_depth_estimation.cpp
    test/test_utils.h
  )
  target_link_libraries(${PROJECT_NAME}-test-depth-estimation ${PROJECT_NAME}
    ament_index_cpp::ament_index_cpp pthread)
  svo_link_test_deps(${PROJECT_NAME}-test-depth-estimation)
endif()

################################################################################
ament_auto_package()
