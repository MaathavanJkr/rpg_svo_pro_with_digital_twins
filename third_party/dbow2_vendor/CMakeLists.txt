cmake_minimum_required(VERSION 3.16)
project(dbow2_vendor CXX)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core features2d)

# Upstream: https://github.com/dorian3d/DBoW2
# To build offline, pass -DFETCHCONTENT_SOURCE_DIR_DBOW2_SRC=<path to DBoW2 checkout>.
include(FetchContent)
FetchContent_Declare(dbow2_src
  URL https://github.com/dorian3d/DBoW2/archive/3924753db6145f12618e7de09b7e6b258db93c6e.tar.gz
  URL_HASH SHA256=7f6f072e86c923d8241bc29eef916e9235c9fa80fbe24127511c6be4542c91cf
  SOURCE_SUBDIR do_not_add_subdirectory)
FetchContent_MakeAvailable(dbow2_src)
set(DBOW2_SRC ${dbow2_src_SOURCE_DIR})

add_library(DBoW2 SHARED
  ${DBOW2_SRC}/src/BowVector.cpp
  ${DBOW2_SRC}/src/FBrief.cpp
  ${DBOW2_SRC}/src/FORB.cpp
  ${DBOW2_SRC}/src/FeatureVector.cpp
  ${DBOW2_SRC}/src/QueryResults.cpp
  ${DBOW2_SRC}/src/ScoringObject.cpp)
target_include_directories(DBoW2 PUBLIC
  $<BUILD_INTERFACE:${DBOW2_SRC}/include>
  $<BUILD_INTERFACE:${DBOW2_SRC}/include/DBoW2>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/DBoW2>)
target_link_libraries(DBoW2 PUBLIC ${OpenCV_LIBS})

install(DIRECTORY ${DBOW2_SRC}/include/ DESTINATION include)
install(TARGETS DBoW2 EXPORT export_dbow2_vendor
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include include/DBoW2)
ament_export_libraries(DBoW2)
ament_export_targets(export_dbow2_vendor HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)
ament_package()
