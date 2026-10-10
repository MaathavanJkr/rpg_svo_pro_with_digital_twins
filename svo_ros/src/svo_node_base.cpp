#include "svo_ros/svo_node_base.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <svo/common/logging.h>
#include <vikit/params_helper.h>

namespace svo_ros {

void SvoNodeBase::initThirdParty(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Pass only the non-ROS arguments (e.g. --v=0) on to gflags/glog.
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> c_args;
  for (std::string& arg : args)
  {
    c_args.push_back(&arg[0]);
  }
  int c_argc = static_cast<int>(c_args.size());
  char** c_argv = c_args.data();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&c_argc, &c_argv, true);
  google::InstallFailureSignalHandler();
  // FLAGS_logtostderr = 1;
}

rclcpp::NodeOptions SvoNodeBase::defaultNodeOptions(rclcpp::NodeOptions options)
{
  return options
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true);
}

SvoNodeBase::SvoNodeBase(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>("svo", options))
  , type_(vk::param<bool>(node_, "pipeline_is_stereo", true) ?
          svo::PipelineType::kStereo : svo::PipelineType::kMono)
  , svo_interface_(type_, node_) // svo interface created here
{

  // subscribe to imu, gps, image and start the associated threads, waiting for new measurements.
  if (svo_interface_.imu_handler_)
  {
    svo_interface_.subscribeImu();
  }
  else
  {
    LOG(FATAL) << "This version of SVO needs IMU!";
  }

  if (svo_interface_.use_global_measurements_)
  { // call back that starts reading all new GP measurements 
    if (svo_interface_.globalpositions_handler_)
    {
      svo_interface_.subscribeGlobalPositions();
    }
    else
    {
      LOG(FATAL) << "Asked to use gp measurements but handler is not available!";
    }
  }

  if (svo_interface_.use_map_registration_)
  {
    if (svo_interface_.globalpose_handler_)
    {
      svo_interface_.subscribeGlobalPose();
    }
    else
    {
      LOG(FATAL) << "Asked to use gp pose but handler is not available!";
    }
  }

  svo_interface_.subscribeImage();
  svo_interface_.subscribeRemoteKey();
}

void SvoNodeBase::run()
{
  rclcpp::spin(node_);
  SVO_INFO_STREAM("SVO quit");
  svo_interface_.quit_ = true;
  SVO_INFO_STREAM("SVO terminated.\n");
}

}  // namespace svo_ros
