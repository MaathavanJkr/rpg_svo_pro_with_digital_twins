cmake_minimum_required(VERSION 3.16)
project(satellite_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
# Debian/Ubuntu install FindGeographicLib.cmake here (libgeographiclib-dev).
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DoubleMsg.msg
  msg/EmitterMsg.msg
  msg/DoubleArrayMsg.msg
  msg/IntMsg.msg
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(gnss_simulator src/main.cpp src/gnss.cpp src/satellite_param.cpp src/emitter.cpp src/frame_handler.cpp)
target_include_directories(gnss_simulator PRIVATE include ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(gnss_simulator
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  "${cpp_typesupport_target}"
  Eigen3::Eigen
  ${YAML_CPP_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  Ceres::ceres
  glog::glog
  pthread)

install(TARGETS gnss_simulator DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/gps_error_publisher.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
