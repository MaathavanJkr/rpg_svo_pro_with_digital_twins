cmake_minimum_required(VERSION 3.16)
project(svo_cmake NONE)

find_package(ament_cmake REQUIRED)

# install all included cmake files
install(DIRECTORY cmake/Modules DESTINATION share/${PROJECT_NAME}/cmake)

ament_package(CONFIG_EXTRAS cmake/svo_cmake-extras.cmake)
