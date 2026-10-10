cmake_minimum_required(VERSION 3.16)
project(vikit_cameras)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

set(HEADERS
  include/vikit/cameras.h
  include/vikit/cameras/camera_geometry_base.h
  include/vikit/cameras/implementation/camera_geometry_base.hpp
  include/vikit/cameras/no_distortion.h
  include/vikit/cameras/atan_distortion.h
  include/vikit/cameras/equidistant_distortion.h
  include/vikit/cameras/equidistant_fisheye_geometry.h
  include/vikit/cameras/equidistant_fisheye_projection.h
  include/vikit/cameras/omni_geometry.h
  include/vikit/cameras/omni_projection.h
  include/vikit/cameras/radial_tangential_distortion.h
  include/vikit/cameras/pinhole_projection.h
  include/vikit/cameras/implementation/pinhole_projection.hpp
  include/vikit/cameras/camera_geometry.h
  include/vikit/cameras/implementation/camera_geometry.hpp
  include/vikit/cameras/camera_factory.h
  include/vikit/cameras/ncamera.h
  include/vikit/cameras/yaml/camera-yaml-serialization.h
  include/vikit/cameras/yaml/ncamera-yaml-serialization.h
  )

set(SOURCES
  src/camera_factory.cpp
  src/ncamera.cpp
  src/camera_geometry_base.cpp
  src/camera_yaml_serialization.cpp
  src/equidistant_fisheye_geometry.cpp
  src/equidistant_fisheye_projection.cpp
  src/omni_geometry.cpp
  src/omni_projection.cpp
  src/ncamera_yaml_serialization.cpp
  )

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${SVO_SYSTEM_LIBRARIES})

################################################################################
# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_tests test/test_cameras.cpp)
  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
  svo_link_test_deps(${PROJECT_NAME}_tests)
endif()

################################################################################
ament_auto_package()
