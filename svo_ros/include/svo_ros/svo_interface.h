#pragma once

#include <thread>
#include <fstream>
#include <vector>
#include <array>
#include <map>
#include <mutex>

#include <ros/ros.h>
#include <std_msgs/String.h>    // user-input
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>


#include <svo/common/types.h>
#include <svo/common/camera_fwd.h>
#include <svo/common/transformation.h>

namespace tf2_ros { class StaticTransformBroadcaster; }

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

  // FYP decision D1(B): one-time 4-DoF initial alignment T_M_W (= T_w_vio) from GT positions.
  // Off by default (param d1_alignment). t0 = first keyframe with backend and scale initialised
  // and GT available (bracketing GT samples <= 10 ms apart). Window = /svo/vio_backend_pose_imu
  // samples from t0 until the GT path length reaches d1_window_length (default 2 m); T_M_W =
  // 4-DoF least-squares fit of VIO to interpolated GT positions over the window (GT orientation
  // not used). Reference implementation: analysis/initial_alignment.py (window_alignment).
  // Sets T_w_vio and gp_initialized_, broadcasts static TF world->vio.
  bool d1_enabled_ = false;
  double d1_window_length_ = 2.0;
  std::vector<double> d1_gt_t_;                    // GT stamps [s]
  std::vector<std::array<double, 7>> d1_gt_pose_;  // px py pz qw qx qy qz
  std::mutex d1_mutex_;
  int64_t d1_t0_ns_ = -1;
  bool d1_done_ = false;
  std::map<int64_t, std::array<double, 7>> d1_vio_poses_;  // recent backend poses by stamp [ns]
  std::string d1_log_file_;
  std::unique_ptr<std::thread> d1_thread_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> d1_tf_broadcaster_;
  void d1Loop();
  bool d1GtBracket(const double t, size_t* i) const;
  void d1TryAlign();  // d1_mutex_ must be held

};

} // namespace svo
