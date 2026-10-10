#pragma once

#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>    // user-input
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>


#include <svo/common/types.h>
#include <svo/common/camera_fwd.h>
#include <svo/common/transformation.h>

namespace svo {

// forward declarations
class FrameHandlerBase;
class Visualizer;
class ImuHandler;
class GlobalPositionsHandler;
class GlobalPoseHandler;
class RawGnssHandler ;
class BackendInterface;
class CeresBackendInterface;
class CeresBackendPublisher;

enum class PipelineType {
  kMono,
  kStereo,
  kArray
};

/// SVO Interface
class SvoInterface
{
public:

  // ROS subscription and publishing.
  // In ROS 2 there is no separate private node handle: parameters and
  // published topics ("~/...") live in the node namespace.
  rclcpp::Node::SharedPtr pnh_;
  PipelineType pipeline_type_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_remote_key_;
  std::string remote_input_;

  // Each sensor stream is processed in its own thread, spinning its own
  // callback group (replaces the per-thread ros::CallbackQueue of ROS 1).
  std::unique_ptr<std::thread> imu_thread_;
  std::unique_ptr<std::thread> globalpositions_thread_;
  std::unique_ptr<std::thread> globalpose_thread_;
  std::unique_ptr<std::thread> image_thread_;

  // SVO modules.
  std::shared_ptr<FrameHandlerBase> svo_;
  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<ImuHandler> imu_handler_;
  std::shared_ptr<GlobalPositionsHandler> globalpositions_handler_;
  std::shared_ptr<GlobalPoseHandler> globalpose_handler_;
  std::shared_ptr<CeresBackendInterface> ceres_backend_interface_;
  std::shared_ptr<CeresBackendPublisher> ceres_backend_publisher_;

  CameraBundlePtr ncam_;

  // Parameters
  bool set_initial_attitude_from_gravity_ = true;
  bool long_enough_trajectory_alignement = false;

  // System state.
  bool quit_ = false;
  bool idle_ = false;
  bool automatic_reinitialization_ = false;

  SvoInterface(const PipelineType& pipeline_type,
               const rclcpp::Node::SharedPtr& node);

  virtual ~SvoInterface();

  // Processing
  void processImageBundle(
      const std::vector<cv::Mat>& images,
      int64_t timestamp_nanoseconds);

  bool setImuPrior(const int64_t timestamp_nanoseconds);

  void publishResults(
      const std::vector<cv::Mat>& images,
      const int64_t timestamp_nanoseconds);

  // Subscription and callbacks
  void monoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void stereoCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& msg0,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg1);
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);
  void globalpositionsCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& gp_msg);
  void globalposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& gp_msg);
  void inputKeyCallback(const std_msgs::msg::String::ConstSharedPtr& key_input);

  // These functions are called before and after monoCallback or stereoCallback.
  // a derived class can implement some additional logic here.
  virtual void imageCallbackPreprocessing(int64_t timestamp_nanoseconds) {}
  virtual void imageCallbackPostprocessing() {}

  void subscribeImu();
  void subscribeImage();
  void subscribeGlobalPositions();
  void subscribeGlobalPose();
  void subscribeRemoteKey();

  void imuLoop();
  void monoLoop();
  void stereoLoop();
  void globalpositionsLoop();
  void globalposeLoop();

  // Pipeline uses global positions.
  bool use_global_measurements_ = true;
  bool use_map_registration_ = true;
  // Pipeline initialized with initial global position.
  bool gp_initialized_ = false;
  bool gpose_initialized_ = false;

  bool refined_T_WL = false;
  bool reinitialize_T_W_VIO = false;
  bool estimate_world_vio_transform_ = false; // @todo: move this to a better location (Gp param???)

private:
  /// Spins the given callback group until shutdown (one executor per thread).
  void spinCallbackGroup(const rclcpp::CallbackGroup::SharedPtr& group);

  /// Serializes the creation of ROS entities from the different threads.
  std::mutex ros_entities_mutex_;
};

} // namespace svo
