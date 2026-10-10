cmake_minimum_required(VERSION 3.16)
project(svo_img_align)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

set(HEADERS_CPU
  include/svo/img_align/sparse_img_align_base.h
  include/svo/img_align/sparse_img_align.h
  )

set(SOURCES_CPU
  src/sparse_img_align_base.cpp
  src/sparse_img_align.cpp
  )

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES_CPU} ${HEADERS_CPU})
target_link_libraries(${PROJECT_NAME} ${SVO_SYSTEM_LIBRARIES})

ament_auto_package()
