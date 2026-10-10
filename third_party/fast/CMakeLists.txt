cmake_minimum_required(VERSION 3.16)
project(fast CXX)

find_package(ament_cmake REQUIRED)

# Upstream: https://github.com/uzh-rpg/fast_neon (previously pulled in via vcs-import).
# To build offline, pass -DFETCHCONTENT_SOURCE_DIR_FAST_NEON_SRC=<path to fast_neon checkout>.
include(FetchContent)
FetchContent_Declare(fast_neon_src
  URL https://github.com/uzh-rpg/fast_neon/archive/4914ed963aba8d2bb476551e186e4b20a206f03d.tar.gz
  URL_HASH SHA256=6fee0f92a16a6f251be62133943b74c8d90773baddb8ade7176e19ba6e1d1b4e
  SOURCE_SUBDIR do_not_add_subdirectory)
FetchContent_MakeAvailable(fast_neon_src)
set(FAST_SRC ${fast_neon_src_SOURCE_DIR})

set(SOURCES
  ${FAST_SRC}/src/fast_9.cpp
  ${FAST_SRC}/src/fast_10.cpp
  ${FAST_SRC}/src/fast_10_score.cpp
  ${FAST_SRC}/src/nonmax_3x3.cpp)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)")
  list(APPEND SOURCES ${FAST_SRC}/src/faster_corner_9_neon.cpp)
  set(FAST_ARCH_FLAGS "")
else()
  list(APPEND SOURCES ${FAST_SRC}/src/faster_corner_10_sse.cpp)
  set(FAST_ARCH_FLAGS -mmmx -msse -msse2 -msse3 -mssse3)
endif()

add_library(fast SHARED ${SOURCES})
target_compile_options(fast PRIVATE -O3 -fomit-frame-pointer ${FAST_ARCH_FLAGS})
target_include_directories(fast PUBLIC
  $<BUILD_INTERFACE:${FAST_SRC}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY ${FAST_SRC}/include/ DESTINATION include)
install(TARGETS fast EXPORT export_fast
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(fast)
ament_export_targets(export_fast HAS_LIBRARY_TARGET)
ament_package()
