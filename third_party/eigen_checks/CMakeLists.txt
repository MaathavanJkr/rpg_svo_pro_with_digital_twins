cmake_minimum_required(VERSION 3.16)
project(eigen_checks CXX)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(GTest REQUIRED)

# Upstream: https://github.com/ethz-asl/eigen_checks
# To build offline, pass -DFETCHCONTENT_SOURCE_DIR_EIGEN_CHECKS_SRC=<path to eigen_checks checkout>.
include(FetchContent)
FetchContent_Declare(eigen_checks_src
  URL https://github.com/ethz-asl/eigen_checks/archive/22a6247a3df11bc285d43d1a030f4e874a413997.tar.gz
  URL_HASH SHA256=14c723978b0b349d1565b78d375d8043eba5f8e6b08af2854b0e4045f6c57f4f
  SOURCE_SUBDIR do_not_add_subdirectory)
FetchContent_MakeAvailable(eigen_checks_src)
set(EC_SRC ${eigen_checks_src_SOURCE_DIR})

add_library(eigen_checks SHARED ${EC_SRC}/src/eigen-checks.cc)
target_include_directories(eigen_checks PUBLIC
  $<BUILD_INTERFACE:${EC_SRC}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(eigen_checks PUBLIC Eigen3::Eigen glog::glog GTest::gtest)

install(DIRECTORY ${EC_SRC}/include/ DESTINATION include)
install(TARGETS eigen_checks EXPORT export_eigen_checks
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(eigen_checks)
ament_export_targets(export_eigen_checks HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 glog GTest)
ament_package()
