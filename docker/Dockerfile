# Build/run environment for SVO Pro with Digital Twins on ROS 2 Jazzy (Ubuntu 24.04).
#
#   docker build -t svo-digital-twin:jazzy -f docker/Dockerfile docker
#
# See README.md ("Docker") for how to build the workspace and run the Oerlikon dataset.
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      git \
      python3-colcon-common-extensions \
      python3-pip \
      python3-vcstool \
      libceres-dev \
      libeigen3-dev \
      libgeographiclib-dev \
      libgflags-dev \
      libgoogle-glog-dev \
      libopencv-dev \
      libopengv-dev \
      libsuitesparse-dev \
      libyaml-cpp-dev \
      python3-matplotlib \
      python3-scipy \
      python3-transforms3d \
      python3-yaml \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-transport \
      ros-jazzy-pcl-conversions \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Open3D (used by svo_registration) is not packaged for Ubuntu 24.04.
# Keep numpy < 2 so the apt-built ROS Python bindings (cv_bridge, ...) keep working.
# `rosbags` is used to convert ROS 1 bags (e.g. the Oerlikon dataset) to ROS 2 bags.
# psutil is installed separately because pip cannot upgrade the Debian-managed copy in place.
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed psutil \
 && pip3 install --no-cache-dir --break-system-packages \
      "numpy<2" \
      open3d \
      rosbags

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
