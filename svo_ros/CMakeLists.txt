cmake_minimum_required(VERSION 3.16)
project(svo_ros)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include(SvoSetup)

find_package(GeographicLib REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

set(HEADERS
  include/svo_ros/svo_interface.h
  include/svo_ros/svo_factory.h
  include/svo_ros/visualizer.h
  include/svo_ros/csv_dataset_reader.h
  include/svo_ros/ceres_backend_factory.h
  include/svo_ros/svo_node_base.h
)

set(SOURCES
  src/ceres_backend_factory.cpp
  src/svo_interface.cpp
  src/svo_factory.cpp
  src/svo_node_base.cpp
  src/visualizer.cpp
  src/csv_dataset_reader.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${GeographicLib_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
  ${PCL_LIBRARIES}
  ${SVO_SYSTEM_LIBRARIES})

ament_auto_add_executable(svo_node src/svo_node.cpp)

# rclcpp component (replaces the ROS 1 nodelet svo_ros/SvoNodelet).
ament_auto_add_library(svo_component SHARED src/svo_component.cpp)
rclcpp_components_register_nodes(svo_component "svo_ros::SvoComponent")

ament_auto_package(INSTALL_TO_SHARE launch param RVIZ)
