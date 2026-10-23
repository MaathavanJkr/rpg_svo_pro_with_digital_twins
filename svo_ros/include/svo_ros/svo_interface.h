#pragma once

#include <thread>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>    // user-input
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>


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
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  PipelineType pipeline_type_;
  ros::Subscriber sub_remote_key_;
  std::string remote_input_;
  
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
  std::shared_ptr<BackendInterface> backend_interface_;
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
          const ros::NodeHandle& nh,
          const ros::NodeHandle& private_nh);

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
  void monoCallback(const sensor_msgs::ImageConstPtr& msg);
  void stereoCallback(
      const sensor_msgs::ImageConstPtr& msg0,
      const sensor_msgs::ImageConstPtr& msg1);
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void globalpositionsCallback(const geometry_msgs::PoseStampedConstPtr& gp_msg);
  void globalposeCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& gp_msg);
  void inputKeyCallback(const std_msgs::StringConstPtr& key_input);

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

  // FYP: optional per-frame event log (stamp, keyframe, backend/scale initialised).
  // Log only, off by default (param log_frame_events); does not affect estimation.
  void logFrameEvent(const int64_t timestamp_nanoseconds);
  bool log_frame_events_ = false;
  std::ofstream frame_event_log_;
  std::vector<char> frame_event_log_buffer_;
  size_t frame_event_log_count_ = 0;
  double frame_event_log_time_sum_ = 0.0;
  double frame_event_log_time_max_ = 0.0;

};

} // namespace svo
