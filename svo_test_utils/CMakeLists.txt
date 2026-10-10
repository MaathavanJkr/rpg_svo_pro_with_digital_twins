cmake_minimum_required(VERSION 3.16)
project(svo_test_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

################################################################################
set(HEADERS
  include/svo/test_utils/test_utils.h
  include/svo/test_utils/imu_simulation.h
  include/svo/test_utils/synthetic_dataset.h
  include/svo/test_utils/simple_raytracer.h
  include/svo/test_utils/trajectory_generator.h
  include/svo/test_utils/point_cloud_generator.h
)

set(SOURCES
  src/test_utils.cpp
  src/imu_simulation.cpp
  src/synthetic_dataset.cpp
  src/simple_raytracer.cpp
  src/trajectory_generator.cpp
  src/point_cloud_generator.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${SVO_SYSTEM_LIBRARIES})

################################################################################
ament_auto_package()
