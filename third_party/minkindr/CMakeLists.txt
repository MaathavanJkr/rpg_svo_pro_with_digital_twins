cmake_minimum_required(VERSION 3.16)
project(minkindr NONE)

find_package(ament_cmake REQUIRED)

# Upstream: https://github.com/ethz-asl/minkindr (header-only).
# To build offline, pass -DFETCHCONTENT_SOURCE_DIR_MINKINDR_SRC=<path to minkindr checkout>.
include(FetchContent)
FetchContent_Declare(minkindr_src
  URL https://github.com/ethz-asl/minkindr/archive/564f12639a8447d4d3e5e7707851424302941056.tar.gz
  URL_HASH SHA256=366deb82ee3ea43c1bf7a5de6a467a0416ceeced547340809add7f328ff7c056
  SOURCE_SUBDIR do_not_add_subdirectory)
FetchContent_MakeAvailable(minkindr_src)

install(DIRECTORY ${minkindr_src_SOURCE_DIR}/minkindr/include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
