cmake_minimum_required(VERSION 3.16)
project(rpg_common)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

FILE(GLOB_RECURSE HEADERS "include/*")

list(APPEND SOURCES src/fs.cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${SVO_SYSTEM_LIBRARIES})

ament_auto_package()
