cmake_minimum_required(VERSION 3.16)
project(svo_pgo)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)
find_package(Ceres REQUIRED)

set(HEADERS
  include/svo/pgo/pgo.h
  )

set(SOURCES
  src/pgo.cpp
  )

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} Ceres::ceres ${SVO_SYSTEM_LIBRARIES})

if(BUILD_TESTING)
  add_executable(test_pgo test/test.cpp)
  target_link_libraries(test_pgo ${PROJECT_NAME})
endif()

ament_auto_package()
