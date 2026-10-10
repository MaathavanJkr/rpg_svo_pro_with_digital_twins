cmake_minimum_required(VERSION 3.16)
project(svo_online_loopclosing)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)
find_package(opengv REQUIRED)

set(HEADERS
  include/svo/online_loopclosing/bow.h
  include/svo/online_loopclosing/geometric_verification.h
  include/svo/online_loopclosing/loop_closing.h
  include/svo/online_loopclosing/read_file.h
  include/svo/online_loopclosing/keyframe.h
  include/svo/online_loopclosing/loop_closing_types.h
  include/svo/online_loopclosing/map_alignment.h
  )

set(SOURCES
  src/bow.cpp
  src/geometric_verification.cpp
  src/loop_closing.cpp
  src/read_file.cpp
  src/map_alignment.cpp
  )

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} opengv ${SVO_SYSTEM_LIBRARIES})

ament_auto_add_library(bow SHARED src/bow.cpp include/svo/online_loopclosing/bow.h
  NO_TARGET_LINK_LIBRARIES)
target_link_libraries(bow ${SVO_SYSTEM_LIBRARIES})

if(BUILD_TESTING)
  add_executable(createvoc test/createvoc.cpp)
  target_link_libraries(createvoc bow ament_index_cpp::ament_index_cpp)
endif()

install(DIRECTORY vocabularies DESTINATION share/${PROJECT_NAME})

ament_auto_package()
