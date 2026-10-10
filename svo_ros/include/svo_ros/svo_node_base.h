#pragma once

#include <rclcpp/rclcpp.hpp>

#include "svo_ros/svo_interface.h"

namespace svo_ros {

class SvoNodeBase {
 public:
  // Initializes ROS, glog and gflags (gflags only sees the non-ROS arguments).
  static void initThirdParty(int argc, char **argv);

  // Node options used by SVO: all parameters given in the launch/yaml files are
  // accepted, since they are read lazily by the factories (like in ROS 1).
  static rclcpp::NodeOptions defaultNodeOptions(
      rclcpp::NodeOptions options = rclcpp::NodeOptions());

  explicit SvoNodeBase(const rclcpp::NodeOptions& options = defaultNodeOptions());

  void run();

  rclcpp::Node::SharedPtr node() const { return node_; }

 private:
  rclcpp::Node::SharedPtr node_;
  svo::PipelineType type_;

 public:
   svo::SvoInterface svo_interface_;
};

}  // namespace svo_ros
