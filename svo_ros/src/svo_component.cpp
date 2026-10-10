// rclcpp component wrapping SVO (replaces the ROS 1 nodelet svo_ros/SvoNodelet).
//
// Load it into a component container, e.g.:
//   ros2 component standalone svo_ros svo_ros::SvoComponent --ros-args --params-file ...

#include <memory>

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "svo_ros/svo_node_base.h"

namespace svo_ros {

class SvoComponent
{
public:
  explicit SvoComponent(const rclcpp::NodeOptions& options)
  {
    static bool glog_initialized = false;
    if (!glog_initialized)
    {
      google::InitGoogleLogging("svo");
      glog_initialized = true;
    }
    svo_.reset(new SvoNodeBase(SvoNodeBase::defaultNodeOptions(options)));
    RCLCPP_INFO_STREAM(svo_->node()->get_logger(), "Initialized SVO component.");
  }

  ~SvoComponent()
  {
    RCLCPP_INFO_STREAM(svo_->node()->get_logger(), "SVO quit");
    svo_->svo_interface_.quit_ = true;
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
  {
    return svo_->node()->get_node_base_interface();
  }

private:
  std::unique_ptr<SvoNodeBase> svo_;
};

}  // namespace svo_ros

RCLCPP_COMPONENTS_REGISTER_NODE(svo_ros::SvoComponent)
