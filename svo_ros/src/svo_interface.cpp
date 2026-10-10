#include <svo_ros/svo_interface.h>


#include <svo_ros/svo_factory.h>
#include <svo_ros/visualizer.h>
#include <svo/common/frame.h>
#include <svo/map.h>
#include <svo/imu_handler.h>
#include <svo/global_positions_handler.h>
#include <svo/global_pose_handler.h>
#include <svo/common/camera.h>
#include <svo/common/conversions.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/frame_handler_array.h>
#include <svo/initialization.h>
#include <svo/direct/depth_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <vikit/params_helper.h>
#include <vikit/timer.h>
#include <svo_ros/ceres_backend_factory.h>


// #include <svo/backend/backend_interface.h>
// #include <svo/backend/backend_optimizer.h>

// for the gps 
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#ifdef SVO_LOOP_CLOSING
#include <svo/online_loopclosing/loop_closing.h>
#endif

#ifdef SVO_GLOBAL_MAP
#include <svo/global_map.h>
#endif

namespace svo {

namespace {

int64_t toNSec(const builtin_interfaces::msg::Time& stamp)
{
  return rclcpp::Time(stamp).nanoseconds();
}

double toSec(const builtin_interfaces::msg::Time& stamp)
{
  return rclcpp::Time(stamp).seconds();
}

} // namespace

SvoInterface::SvoInterface(
    const PipelineType& pipeline_type,
    const rclcpp::Node::SharedPtr& node)
  : pnh_(node)
  , pipeline_type_(pipeline_type)
  , set_initial_attitude_from_gravity_(
      vk::param<bool>(pnh_, "set_initial_attitude_from_gravity", true))
  , automatic_reinitialization_(
      vk::param<bool>(pnh_, "automatic_reinitialization", false))
{
  switch (pipeline_type)
  {
    case PipelineType::kMono:
      svo_ = factory::makeMono(pnh_);
      break;
    case PipelineType::kStereo:
      svo_ = factory::makeStereo(pnh_);
      break;
    case PipelineType::kArray:
      svo_ = factory::makeArray(pnh_);
      break;
    default:
      LOG(FATAL) << "Unknown pipeline";
      break;
  }
  ncam_ = svo_->getNCamera();

  visualizer_.reset(
        new Visualizer(svo_->options_.trace_dir, pnh_, ncam_->getNumCameras()));

  if(vk::param<bool>(pnh_, "use_imu", true))
  {
    imu_handler_ = factory::getImuHandler(pnh_);
    svo_->imu_handler_ = imu_handler_;
  }

  // Initialize class dealing with global positions.
  globalpositions_handler_ = factory::getGlobalPositionsHandler(pnh_);
  svo_->globalpositions_handler_ = globalpositions_handler_;

  use_global_measurements_ = vk::param<bool>(pnh_, "use_global_measurements", true);
  estimate_world_vio_transform_ = vk::param<bool>(pnh_, "estimate_world_vio_transform", true);

  svo_->use_global_measurements_ = use_global_measurements_;

  if (use_global_measurements_)
  {
    SVO_INFO_STREAM("Using Global Position Measurements");
  }


  //! @todo : here we need to add the global pose handler
  // Initialize class dealing with global pose (map registration)
  globalpose_handler_ = factory::getGlobalPoseHandler(pnh_);
  svo_->globalpose_handler_ = globalpose_handler_; //HERE DO THIS
  use_map_registration_ = vk::param<bool>(pnh_, "use_map_registration", true);
  svo_->use_map_registration_ = use_map_registration_;

  if (use_map_registration_)
  {
    SVO_INFO_STREAM("Using Map registration  Global Pose Measurements");
  }

  if(vk::param<bool>(pnh_, "use_ceres_backend", true))
  {
    ceres_backend_interface_ = ceres_backend_factory::makeBackend(pnh_, ncam_);

    if(imu_handler_){
      svo_->setBundleAdjuster(ceres_backend_interface_);
      ceres_backend_interface_->setImu(imu_handler_);
      ceres_backend_interface_->makePublisher(pnh_, ceres_backend_publisher_);
    }
    else
    {
      SVO_ERROR_STREAM("Cannot use ceres backend without using imu");
    }

    if (use_global_measurements_)
    {
      ceres_backend_interface_->setGpHandler(globalpositions_handler_);
      ceres_backend_interface_->use_global_measurements_ = true;

    }

    if (use_map_registration_)
    {
      ceres_backend_interface_->setGposeHandler(globalpose_handler_); // here
      ceres_backend_interface_->use_map_registration_ = true;
    }

  }
  if(vk::param<bool>(pnh_, "runlc", false))
  {
    LOG(FATAL) << "Loop closure is not supported in this version of SVO!";

#ifdef SVO_LOOP_CLOSING
    LoopClosingPtr loop_closing_ptr =
        factory::getLoopClosingModule(pnh_, svo_->getNCamera());
    svo_->lc_ = std::move(loop_closing_ptr);
    CHECK(svo_->depth_filter_->options_.extra_map_points)
        << "The depth filter seems to be initialized without extra map points.";
#else
    LOG(FATAL) << "You have to enable loop closing in svo_cmake.";
#endif
  }

  if(vk::param<bool>(pnh_, "use_global_map", false))
  {
#ifdef SVO_GLOBAL_MAP
    svo_->global_map_ = factory::getGlobalMap(pnh_, svo_->getNCamera());
    if (imu_handler_)
    {
      svo_->global_map_->initializeIMUParams(imu_handler_->imu_calib_,
                                             imu_handler_->imu_init_);
    }
#else
    LOG(FATAL) << "You have to enable global map in cmake";
#endif
  }

  svo_->start();
}

SvoInterface::~SvoInterface()
{
  if (imu_thread_)
    imu_thread_->join();
  if (image_thread_)
    image_thread_->join();
  if (globalpositions_thread_)
    globalpositions_thread_->join();
  if (globalpose_thread_)
    globalpose_thread_->join();

  VLOG(1) << "Destructed SVO.";
}



void SvoInterface::processImageBundle(
    const std::vector<cv::Mat>& images,
    const int64_t timestamp_nanoseconds)
{
  if (!svo_->isBackendValid()) // if ceres backend not valid, create it -> ceres backend = BA
  {
    if (vk::param<bool>(pnh_, "use_ceres_backend", true, true)) // if using ceres backend, set to true if not specified
    {
      ceres_backend_interface_ = ceres_backend_factory::makeBackend(pnh_, ncam_);
      if(globalpositions_handler_)
      {
        ceres_backend_interface_->setGpHandler(globalpositions_handler_);
      }
      else
      {
        SVO_ERROR_STREAM("Cannot create global position handler in the backend");
      }
      if(globalpose_handler_)
      {
        ceres_backend_interface_->setGposeHandler(globalpose_handler_);
      }
      else
      {
        SVO_ERROR_STREAM("Cannot create global pose handler in the backend");
      }
      if (imu_handler_)
      {
        svo_->setBundleAdjuster(ceres_backend_interface_);
        ceres_backend_interface_->setImu(imu_handler_);
        std::lock_guard<std::mutex> lock(ros_entities_mutex_);
        ceres_backend_interface_->makePublisher(pnh_, ceres_backend_publisher_);
      }
      else
      {
        SVO_ERROR_STREAM("Cannot use ceres backend without using imu");
      }
    }
  }
  svo_->addImageBundle(images, timestamp_nanoseconds);
}

void SvoInterface::publishResults(
    const std::vector<cv::Mat>& images,
    const int64_t timestamp_nanoseconds)
{
  CHECK_NOTNULL(svo_.get());
  CHECK_NOTNULL(visualizer_.get());

  visualizer_->img_caption_.clear();
  if (svo_->isBackendValid())
  {
    std::string static_str = ceres_backend_interface_->getStationaryStatusStr();
    visualizer_->img_caption_ = static_str;
  }

  visualizer_->publishSvoInfo(svo_.get(), timestamp_nanoseconds);
  switch (svo_->stage())
  {
    case Stage::kTracking: {
      Eigen::Matrix<double, 6, 6> covariance;
      covariance.setZero();
      visualizer_->publishImuPose(svo_->getLastFrames()->get_T_W_B(), covariance, timestamp_nanoseconds);
      visualizer_->publishCameraPoses(svo_->getLastFrames(), timestamp_nanoseconds);
      visualizer_->visualizeMarkers(svo_->getLastFrames(), svo_->closeKeyframes(), svo_->map(), svo::Visualizer::kVioFrame);
      visualizer_->exportToDense(svo_->getLastFrames());
      bool draw_boundary = false;
      if (svo_->isBackendValid())
      {
        draw_boundary = svo_->getBundleAdjuster()->isFixedToGlobalMap();
      }
      visualizer_->publishImagesWithFeatures(
            svo_->getLastFrames(), timestamp_nanoseconds,
            draw_boundary);
#ifdef SVO_LOOP_CLOSING
      // detections
      if (svo_->lc_)
      {
        visualizer_->publishLoopClosureInfo(
              svo_->lc_->cur_loop_check_viz_info_,
              std::string("loop_query"),
              Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.5);
        visualizer_->publishLoopClosureInfo(
              svo_->lc_->loop_detect_viz_info_, std::string("loop_detection"),
              Eigen::Vector3f(1.0f, 0.0f, 0.0f), 1.0);
        if (svo_->isBackendValid())
        {
          visualizer_->publishLoopClosureInfo(
                svo_->lc_->loop_correction_viz_info_,
                std::string("loop_correction"),
                Eigen::Vector3f(0.0f, 1.0f, 0.0f), 3.0);
        }
        if (svo_->getLastFrames()->at(0)->isKeyframe())
        {
          bool pc_recalculated = visualizer_->publishPoseGraph(
                svo_->lc_->kf_list_,
                svo_->lc_->need_to_update_pose_graph_viz_,
                static_cast<size_t>(svo_->lc_->options_.ignored_past_frames));
          if(pc_recalculated)
          {
            svo_->lc_->need_to_update_pose_graph_viz_ = false;
          }
        }
      }
#endif
#ifdef SVO_GLOBAL_MAP
      if (svo_->global_map_)
      {
        visualizer_->visualizeGlobalMap(*(svo_->global_map_),
                                        std::string("global_vis"),
                                        Eigen::Vector3f(0.0f, 0.0f, 1.0f),
                                        0.3);
        visualizer_->visualizeFixedLandmarks(svo_->getLastFrames()->at(0));
      }
#endif
      break;
    }
    case Stage::kInitializing: {
      visualizer_->publishBundleFeatureTracks(
            svo_->initializer_->frames_ref_, svo_->getLastFrames(),
            timestamp_nanoseconds);
      break;
    }
    case Stage::kPaused:
    case Stage::kRelocalization:{
      reinitialize_T_W_VIO = true;
      visualizer_->publishImages(images, timestamp_nanoseconds);

      break;
    }
    default:
      LOG(FATAL) << "Unknown stage";
      break;
  }
}

bool SvoInterface::setImuPrior(const int64_t timestamp_nanoseconds)
{

  if(svo_->getBundleAdjuster())
  {

    //if we use backend, this will take care of setting priors
    if(!svo_->hasStarted())
    {
      //before starting up, make sure we already have IMU measurements
      if(imu_handler_->getMeasurementsCopy().size() < 10u)
      {
        return false;
      }
    }

    return true;
  }


  if(imu_handler_ && !svo_->hasStarted() && set_initial_attitude_from_gravity_)
  {
    // set initial orientation
    Quaternion R_imu_world;
    if(imu_handler_->getInitialAttitude(timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds, R_imu_world))
    { //R_imu orld is set based on gravity
      VLOG(0) << "Set initial orientation from accelerometer measurements.";
      svo_->setRotationPrior(R_imu_world);
    }
    else
    {
      return false;
    }
  }
  else if(imu_handler_ && svo_->getLastFrames())
  {
    // set incremental rotation prior
    Quaternion R_lastimu_newimu;
    if(imu_handler_->getRelativeRotationPrior(
         svo_->getLastFrames()->getMinTimestampNanoseconds() *
         common::conversions::kNanoSecondsToSeconds,
         timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
         false, R_lastimu_newimu))
    {
      VLOG(3) << "Set incremental rotation prior from IMU.";
      svo_->setRotationIncrementPrior(R_lastimu_newimu);
    }
  }
  return true;
}

void SvoInterface::monoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if(idle_) {
    return;
  }


  cv::Mat image;
  try
  {
    image = cv_bridge::toCvCopy(msg)->image;
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(pnh_->get_logger(), "cv_bridge exception: %s", e.what());
  }

  std::vector<cv::Mat> images;
  images.push_back(image.clone());

  const int64_t stamp_ns = toNSec(msg->header.stamp);
  if(!setImuPrior(stamp_ns))
  {
    VLOG(0) << "Could not align gravity! Attempting again in next iteration.";
    return;
  }

  imageCallbackPreprocessing(stamp_ns); // does nothing for now

  processImageBundle(images, stamp_ns);

  publishResults(images, stamp_ns);

  if(svo_->stage() == Stage::kPaused && automatic_reinitialization_)
    svo_->start();

  imageCallbackPostprocessing(); // does nothing for now 
}

void SvoInterface::stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg0,
    const sensor_msgs::msg::Image::ConstSharedPtr& msg1)
{
  if(idle_)
    return;

  cv::Mat img0, img1;
  try {
    img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
    img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(pnh_->get_logger(), "cv_bridge exception: %s", e.what());
  }

  const int64_t stamp_ns = toNSec(msg0->header.stamp);
  if(!setImuPrior(stamp_ns))
  {
    VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
    return;
  }

  imageCallbackPreprocessing(stamp_ns); //does nothing

  processImageBundle({img0, img1}, stamp_ns);
  publishResults({img0, img1}, stamp_ns);

  if(svo_->stage() == Stage::kPaused && automatic_reinitialization_) {
    std::cout << "(re)start of svo" << std::endl;
    svo_->start();
  }

  imageCallbackPostprocessing(); // does nothing 
}

void SvoInterface::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
  const Eigen::Vector3d omega_imu(
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  const Eigen::Vector3d lin_acc_imu(
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const ImuMeasurement m(toSec(msg->header.stamp), omega_imu, lin_acc_imu);
  if(imu_handler_){
    imu_handler_->addImuMeasurement(m);
  }
  else
    SVO_ERROR_STREAM("SvoNode has no ImuHandler");
}

void SvoInterface::globalpositionsCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
  // Recieved in WORLD frame
  const Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  const GpMeasurement m(toSec(msg->header.stamp), position);

  if((globalpositions_handler_))
  {
    if(svo_->isBackendValid()){

    if(estimate_world_vio_transform_) {
      if(svo_->isBackendScaleInitialised()) {
        ViNodeState last_state_;
        ceres_backend_interface_->getLastState(&last_state_);
        Transformation T_W_B = last_state_.get_T_W_B(); // confusing name ?? This i actually the local frame ???
        Eigen::Vector3d W_v_B = last_state_.get_W_v_B();
        int64_t stamp = last_state_.getTimestamp(); // last optimize state but we are a bit above rght ? Time of optimization
        double stamp_sec = stamp * 1e-9;
        double d_t_ = m.timestamp_ - stamp_sec;
        Eigen::Vector3d position_backend = d_t_*W_v_B + T_W_B.getPosition();
        const GpMeasurement m_vio(d_t_ +  stamp_sec, position_backend);

        Eigen::Vector2d vel_xy(W_v_B(0), W_v_B(1));
        double norm_xy = vel_xy.norm();      
          
        if((m_vio.position_.head<2>().norm() > globalpositions_handler_->gp_settings_.alignement_d_min)) {// && (norm_xy >= globalpositions_handler_->gp_settings_.alignement_speed_min)) { // take only measurement that helps the observability

          long_enough_trajectory_alignement = true;
        }
        if(long_enough_trajectory_alignement) {

          globalpositions_handler_->addAlignmentGpMeasurement(m); // here we need to handle delays... both of them have timestamp. Can we get the estimated velocity from the vio ? or should we interpolate (adding delay but for alignment doesnt matter) ?
          globalpositions_handler_->addAlignmentVIOMeasurement(m_vio);
          // Transformation T_l = 
          globalpositions_handler_->PublishLocalEstimateInWorldFrame(last_state_.get_T_W_B());
          ceres_backend_interface_->setT_w_vio_temp(globalpositions_handler_->getT_w_vio_temp());

          if(!gp_initialized_) {
            if(globalpositions_handler_->alignFrames()) 
            {
              globalpositions_handler_->addGpMeasurement(m);
              globalpose_handler_->setT_w_vio(globalpositions_handler_->getT_w_vio());
              gp_initialized_ = true;
              ceres_backend_interface_->setT_w_vio(globalpositions_handler_->getT_w_vio());
              ceres_backend_interface_->setT_w_vio_temp(globalpositions_handler_->getT_w_vio_temp());

            }
          } 
        }
      }
    }

    if(gp_initialized_ && svo_->isBackendScaleInitialised()) {
      globalpositions_handler_->addGpMeasurement(m);
    }

    } else {
    SVO_ERROR_STREAM("Backend not valid!");
    }
  }
  else
    SVO_ERROR_STREAM("SvoNode has no Global Position Handler");
}

void SvoInterface::globalposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
{  
  if(globalpose_handler_)
  {
    // Read message 
    const Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const Eigen::Quaterniond orientation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Matrix<double, 6, 6> information ;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) 
            information(i, j) = msg->pose.covariance[i * 6 + j];
    }
    const GposeMeasurement m(toSec(msg->header.stamp), position, orientation, information);

    if(globalpose_handler_->readInitialPose())
    {
      if(svo_->isBackendValid() && gp_initialized_) 
          gpose_initialized_ = true; 
      
      globalpose_handler_->setReadInitialPose(false);
    } else {

      if(gp_initialized_) {
        if(refined_T_WL || !globalpose_handler_->gp_settings_.refine_T_WL) {

          globalpose_handler_->addGposeMeasurement(m); // world to local transform performed inside
        } else {

          // Get svo estimation in local frame
          double t_mr = toSec(msg->header.stamp);
          Transformation svo_T_LB; 
          if(ceres_backend_interface_->getPoseAtKFatTimestamp(t_mr, &svo_T_LB)) {
            GposeMeasurement m_vio(t_mr, svo_T_LB.getPosition(), Eigen::Quaterniond::Identity());
            globalpose_handler_->addAlignmentVIOMeasurement(m_vio); // local frame
            globalpose_handler_->addAlignmentGposeMeasurement(m); // world frame
            if(globalpose_handler_->alignFrames()) {
              if (globalpositions_handler_)
                globalpositions_handler_->setT_w_vio(globalpose_handler_->getT_w_vio());
              

              ceres_backend_interface_->setT_w_vio(globalpose_handler_->getT_w_vio(), true);
              refined_T_WL = true;
            }

          }
        }
      }
    }
  }
  else
    SVO_ERROR_STREAM("SvoNode has no Global Pose Handler");

}

void SvoInterface::inputKeyCallback(const std_msgs::msg::String::ConstSharedPtr& key_input)
{
  std::string remote_input = key_input->data;
  char input = remote_input.c_str()[0];
  switch(input)
  {
    case 'q':
      quit_ = true;
      SVO_INFO_STREAM("SVO user input: QUIT");
      break;
    case 'r':
      svo_->reset();
      idle_ = true;
      SVO_INFO_STREAM("SVO user input: RESET");
      break;
    case 's':
      svo_->start();
      idle_ = false;
      SVO_INFO_STREAM("SVO user input: START");
      break;
     case 'c':
      svo_->setCompensation(true);
      SVO_INFO_STREAM("Enabled affine compensation.");
      break;
     case 'C':
      svo_->setCompensation(false);
      SVO_INFO_STREAM("Disabled affine compensation.");
      break;
    default: ;
  }
}

void SvoInterface::subscribeImu()
{
  imu_thread_ = std::unique_ptr<std::thread>(
        new std::thread(&SvoInterface::imuLoop, this));
  sleep(3);
}

void SvoInterface::subscribeGlobalPositions()
{
  globalpositions_thread_ = std::unique_ptr<std::thread>(
    new std::thread(&SvoInterface::globalpositionsLoop, this));
}

void SvoInterface::subscribeGlobalPose()
{
  globalpose_thread_ = std::unique_ptr<std::thread>(
    new std::thread(&SvoInterface::globalposeLoop, this));
}

void SvoInterface::subscribeImage()
{
  if(pipeline_type_ == PipelineType::kMono)
    image_thread_ = std::unique_ptr<std::thread>(
          new std::thread(&SvoInterface::monoLoop, this));
  else if(pipeline_type_ == PipelineType::kStereo)
    image_thread_ = std::unique_ptr<std::thread>(
        new std::thread(&SvoInterface::stereoLoop, this));
}

void SvoInterface::subscribeRemoteKey()
{
  std::lock_guard<std::mutex> lock(ros_entities_mutex_);
  std::string remote_key_topic = vk::param<std::string>(pnh_, "remote_key_topic", "svo/remote_key");
  sub_remote_key_ = pnh_->create_subscription<std_msgs::msg::String>(
      remote_key_topic, 5,
      std::bind(&svo::SvoInterface::inputKeyCallback, this, std::placeholders::_1));
}

void SvoInterface::spinCallbackGroup(const rclcpp::CallbackGroup::SharedPtr& group)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_callback_group(group, pnh_->get_node_base_interface());
  while(rclcpp::ok() && !quit_)
  {
    executor.spin_once(std::chrono::milliseconds(100));
  }
}

void SvoInterface::imuLoop() // callback to read all new imu measurements 
{
  SVO_INFO_STREAM("SvoNode: Started IMU loop.");
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  {
    std::lock_guard<std::mutex> lock(ros_entities_mutex_);
    group = pnh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    std::string imu_topic = vk::param<std::string>(pnh_, "imu_topic", "/kingfisher/agiros_pilot/sensors/clean_imu");
    // Large queue: IMU messages are cheap and must not be dropped.
    sub_imu = pnh_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::QoS(1000),
        std::bind(&svo::SvoInterface::imuCallback, this, std::placeholders::_1), options);
  }
  spinCallbackGroup(group);
}

void SvoInterface::globalpositionsLoop() // callback to read all new gp measurements 
{
  SVO_INFO_STREAM("SvoNode: Started Global Positions loop.");
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_gp;
  {
    std::lock_guard<std::mutex> lock(ros_entities_mutex_);
    group = pnh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    std::string gp_topic = vk::param<std::string>(pnh_, "gp_topic", "/gps") ; 
    sub_gp = pnh_->create_subscription<geometry_msgs::msg::PoseStamped>(
        gp_topic, 5,
        std::bind(&svo::SvoInterface::globalpositionsCallback, this, std::placeholders::_1), options);
  }
  spinCallbackGroup(group);
}

void SvoInterface::globalposeLoop() // callback to read all new gp measurements 
{
  SVO_INFO_STREAM("SvoNode: Started Global Pose loop.");
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gpose;
  {
    std::lock_guard<std::mutex> lock(ros_entities_mutex_);
    group = pnh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    std::string gpose_topic = vk::param<std::string>(pnh_, "map_alignement_T_w_b", "/mr");
    sub_gpose = pnh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        gpose_topic, 5,
        std::bind(&svo::SvoInterface::globalposeCallback, this, std::placeholders::_1), options);
  }
  spinCallbackGroup(group);
}

void SvoInterface::monoLoop()
{
  SVO_INFO_STREAM("SvoNode: Started Image loop.");

  rclcpp::CallbackGroup::SharedPtr group;
  image_transport::Subscriber it_sub;
  {
    std::lock_guard<std::mutex> lock(ros_entities_mutex_);
    group = pnh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 5;
    std::string image_topic = vk::param<std::string>(pnh_, "cam0_topic", "/img_0");
    it_sub = image_transport::create_subscription(
        pnh_.get(), image_topic,
        std::bind(&svo::SvoInterface::monoCallback, this, std::placeholders::_1),
        "raw", qos, options);
  }
  spinCallbackGroup(group);
}

void SvoInterface::stereoLoop()
{
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

  rclcpp::CallbackGroup::SharedPtr group;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub0;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub1;
  {
    std::lock_guard<std::mutex> lock(ros_entities_mutex_);
    group = pnh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 1;

    // subscribe to cam msgs
    std::string cam0_topic(vk::param<std::string>(pnh_, "cam0_topic", "/img0"));
    std::string cam1_topic(vk::param<std::string>(pnh_, "cam1_topic", "/img1"));
    sub0.subscribe(pnh_.get(), cam0_topic, qos, options);
    sub1.subscribe(pnh_.get(), cam1_topic, qos, options);
  }
  ExactSync sync_sub(ExactPolicy(5), sub0, sub1);
  sync_sub.registerCallback(std::bind(&svo::SvoInterface::stereoCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));

  spinCallbackGroup(group);
}

} // namespace svo
