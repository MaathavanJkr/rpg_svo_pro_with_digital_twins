#include <svo_ros/svo_interface.h>

#include <ros/callback_queue.h>

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
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <vikit/params_helper.h>
#include <vikit/timer.h>
#include <svo_ros/ceres_backend_factory.h>


// #include <svo/backend/backend_interface.h>
// #include <svo/backend/backend_optimizer.h>

// for the gps 
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#ifdef SVO_USE_GTSAM_BACKEND
#include <svo_ros/backend_factory.h>
#include <svo/backend/backend_interface.h>
#include <svo/backend/backend_optimizer.h>
#endif

#ifdef SVO_LOOP_CLOSING
#include <svo/online_loopclosing/loop_closing.h>
#endif

#ifdef SVO_GLOBAL_MAP
#include <svo/global_map.h>
#endif

namespace svo {

SvoInterface::SvoInterface(
    const PipelineType& pipeline_type,
    const ros::NodeHandle& nh,
    const ros::NodeHandle& private_nh)
  : nh_(nh)
  , pnh_(private_nh)
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

    // FYP fix: pass the configured flags to the backend. Its members default to
    // true and were only ever set to true, so with use_global_measurements=false
    // the backend dereferenced a null GlobalPositionsHandler on the first frame.
    ceres_backend_interface_->use_global_measurements_ = use_global_measurements_;
    ceres_backend_interface_->use_map_registration_ = use_map_registration_;

  }
#ifdef SVO_USE_GTSAM_BACKEND
  if(vk::param<bool>(pnh_, "use_backend", false))
  {
    backend_interface_ = svo::backend_factory::makeBackend(pnh_);
    ceres_backend_publisher_.reset(new CeresBackendPublisher(svo_->options_.trace_dir, pnh_));
    svo_->setBundleAdjuster(backend_interface_);
    backend_interface_->imu_handler_ = imu_handler_;
  }
#endif
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

  // FYP: optional per-frame event log (log only; does not affect estimation).
  log_frame_events_ = vk::param<bool>(pnh_, "log_frame_events", false);
  if (log_frame_events_)
  {
    const std::string path = vk::param<std::string>(
          pnh_, "frame_event_log_file", "/tmp/svo_frame_events.csv");
    frame_event_log_buffer_.resize(1 << 20);  // 1 MB buffer, set before open()
    frame_event_log_.rdbuf()->pubsetbuf(frame_event_log_buffer_.data(),
                                        frame_event_log_buffer_.size());
    frame_event_log_.open(path);
    frame_event_log_ << "stamp_ns,frame_stamp_ns,bundle_id,stage,is_kf,backend_init,scale_init\n";
    SVO_INFO_STREAM("Logging frame events to " << path);
  }

  svo_->start();
}

SvoInterface::~SvoInterface()
{
  if (frame_event_log_.is_open())
  {
    frame_event_log_.close();  // FYP: flushes the frame event log
    if (frame_event_log_count_ > 0)
      std::cout << "FYP frame event log: " << frame_event_log_count_ << " rows, mean "
                << 1e6 * frame_event_log_time_sum_ / frame_event_log_count_ << " us, max "
                << 1e6 * frame_event_log_time_max_ << " us per frame" << std::endl;
  }
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
        ceres_backend_interface_->makePublisher(pnh_, ceres_backend_publisher_);
      }
      else
      {
        SVO_ERROR_STREAM("Cannot use ceres backend without using imu");
      }
      // FYP fix: same as in the constructor, pass the configured flags.
      ceres_backend_interface_->use_global_measurements_ = use_global_measurements_;
      ceres_backend_interface_->use_map_registration_ = use_map_registration_;
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

#ifdef SVO_USE_GTSAM_BACKEND
  if(svo_->stage() == Stage::kTracking && backend_interface_)
  {
    if(svo_->getLastFrames()->isKeyframe())
    {
      std::lock_guard<std::mutex> estimate_lock(backend_interface_->optimizer_->estimate_mut_);
      const gtsam::Values& state = backend_interface_->optimizer_->estimate_;
      // ceres_backend_publisher_->visualizeFrames(state);
      if(backend_interface_->options_.add_imu_factors)
        ceres_backend_publisher_->visualizeVelocity(state);
      ceres_backend_publisher_->visualizePoints(state);
    }
  }
#endif
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

void SvoInterface::monoCallback(const sensor_msgs::ImageConstPtr& msg)
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
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  std::vector<cv::Mat> images;
  images.push_back(image.clone());

  if(!setImuPrior(msg->header.stamp.toNSec()))
  {
    VLOG(0) << "Could not align gravity! Attempting again in next iteration.";
    return;
  }

  imageCallbackPreprocessing(msg->header.stamp.toNSec()); // does nothing for now

  processImageBundle(images, msg->header.stamp.toNSec());

  publishResults(images, msg->header.stamp.toNSec());

  logFrameEvent(msg->header.stamp.toNSec()); // FYP: no-op unless log_frame_events

  if(svo_->stage() == Stage::kPaused && automatic_reinitialization_)
    svo_->start();

  imageCallbackPostprocessing(); // does nothing for now 
}

void SvoInterface::stereoCallback(
    const sensor_msgs::ImageConstPtr& msg0,
    const sensor_msgs::ImageConstPtr& msg1)
{
  if(idle_)
    return;

  cv::Mat img0, img1;
  try {
    img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
    img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if(!setImuPrior(msg0->header.stamp.toNSec()))
  {
    VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
    return;
  }

  imageCallbackPreprocessing(msg0->header.stamp.toNSec()); //does nothing 

  processImageBundle({img0, img1}, msg0->header.stamp.toNSec());
  publishResults({img0, img1}, msg0->header.stamp.toNSec());
  logFrameEvent(msg0->header.stamp.toNSec()); // FYP: no-op unless log_frame_events

  if(svo_->stage() == Stage::kPaused && automatic_reinitialization_) {
    std::cout << "(re)start of svo" << std::endl;
    svo_->start();
  }

  imageCallbackPostprocessing(); // does nothing
}

// FYP: one CSV row per processed image bundle. Reads state only.
// backend_init: backend exists and has finished at least one optimisation.
// scale_init:   frontend flag also required by the original GPS alignment.
void SvoInterface::logFrameEvent(const int64_t timestamp_nanoseconds)
{
  if (!log_frame_events_ || !frame_event_log_.is_open())
    return;
  vk::Timer timer;
  timer.start();
  const FrameBundlePtr frames = svo_->getLastFrames();
  const bool is_kf = frames && frames->at(0)->isKeyframe();
  const bool backend_init = svo_->isBackendValid()
      && svo_->getBundleAdjuster()->lastOptimizedBundleId() >= 0;
  frame_event_log_ << timestamp_nanoseconds << ','
                   << (frames ? frames->getMinTimestampNanoseconds() : -1) << ','
                   << (frames ? frames->getBundleId() : -1) << ','
                   << static_cast<int>(svo_->stage()) << ','
                   << is_kf << ',' << backend_init << ','
                   << svo_->isBackendScaleInitialised() << '\n';
  if (++frame_event_log_count_ % 200 == 0)
    frame_event_log_.flush();
  const double dt = timer.stop();
  frame_event_log_time_sum_ += dt;
  frame_event_log_time_max_ = std::max(frame_event_log_time_max_, dt);
}

void SvoInterface::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  const Eigen::Vector3d omega_imu(
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  const Eigen::Vector3d lin_acc_imu(
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const ImuMeasurement m(msg->header.stamp.toSec(), omega_imu, lin_acc_imu);
  if(imu_handler_){
    imu_handler_->addImuMeasurement(m);
  }
  else
    SVO_ERROR_STREAM("SvoNode has no ImuHandler");
}

void SvoInterface::globalpositionsCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // Recieved in WORLD frame
  const Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  const GpMeasurement m(msg->header.stamp.toSec(), position);

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

void SvoInterface::globalposeCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
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
    const GposeMeasurement m(msg->header.stamp.toSec(), position, orientation, information);

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
          double t_mr = msg->header.stamp.toSec();
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

void SvoInterface::inputKeyCallback(const std_msgs::StringConstPtr& key_input)
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
  std::string remote_key_topic = vk::param<std::string>(pnh_, "remote_key_topic", "svo/remote_key");
  sub_remote_key_ =
      nh_.subscribe(remote_key_topic, 5, &svo::SvoInterface::inputKeyCallback, this);
}

void SvoInterface::imuLoop() // callback to read all new imu measurements 
{
  SVO_INFO_STREAM("SvoNode: Started IMU loop.");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  std::string imu_topic = vk::param<std::string>(pnh_, "imu_topic", "/kingfisher/agiros_pilot/sensors/clean_imu");
  ros::Subscriber sub_imu = nh.subscribe(imu_topic, 10, &svo::SvoInterface::imuCallback, this);

  while(ros::ok() && !quit_)
  {
    queue.callAvailable(ros::WallDuration(0.1));
  }
}

void SvoInterface::globalpositionsLoop() // callback to read all new gp measurements 
{
  SVO_INFO_STREAM("SvoNode: Started Global Positions loop.");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  std::string gp_topic = vk::param<std::string>(pnh_, "gp_topic", "/gps") ; 
  ros::Subscriber sub_gp = nh.subscribe(gp_topic, 5, &svo::SvoInterface::globalpositionsCallback, this);
  while(ros::ok() && !quit_)
  {
    queue.callAvailable(ros::WallDuration(0.1));
  }
}

void SvoInterface::globalposeLoop() // callback to read all new gp measurements 
{
  SVO_INFO_STREAM("SvoNode: Started Global Positions loop.");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  std::string gpose_topic = vk::param<std::string>(pnh_, "map_alignement_T_w_b", "/mr");
  ros::Subscriber sub_gp = nh.subscribe(gpose_topic, 5, &svo::SvoInterface::globalposeCallback, this);
  while(ros::ok() && !quit_)
  {
    queue.callAvailable(ros::WallDuration(0.1));
  }
}

void SvoInterface::monoLoop()
{
  SVO_INFO_STREAM("SvoNode: Started Image loop.");

  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  image_transport::ImageTransport it(nh);
  std::string image_topic = vk::param<std::string>(pnh_, "cam0_topic", "/img_0");
  image_transport::Subscriber it_sub = it.subscribe(image_topic, 5, &svo::SvoInterface::monoCallback, this);

  while(ros::ok() && !quit_)
  {
    queue.callAvailable(ros::WallDuration(0.1));
  }
}

void SvoInterface::stereoLoop()
{
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

  ros::NodeHandle nh(nh_, "image_thread");
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  // subscribe to cam msgs
  std::string cam0_topic(vk::param<std::string>(pnh_, "cam0_topic", "/img0"));
  std::string cam1_topic(vk::param<std::string>(pnh_, "cam1_topic", "/img1"));
  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter sub0(it, cam0_topic, 1, std::string("raw"));
  image_transport::SubscriberFilter sub1(it, cam1_topic, 1, std::string("raw"));
  ExactSync sync_sub(ExactPolicy(5), sub0, sub1);
  sync_sub.registerCallback(boost::bind(&svo::SvoInterface::stereoCallback, this, _1, _2));

  while(ros::ok() && !quit_)
  {
    queue.callAvailable(ros::WallDuration(0.1));
  }
}

} // namespace svo
