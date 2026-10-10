cmake_minimum_required(VERSION 3.16)
project(svo_registration NONE)

find_package(ament_cmake REQUIRED)

# Online registration of the SVO point cloud to the city twin.
# Requires Open3D (`pip install open3d`), which is not packaged for Ubuntu 24.04.
install(PROGRAMS
  scripts/online_registration.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
