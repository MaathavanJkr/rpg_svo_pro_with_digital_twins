// Author : Giovanni Cioffi <cioffi at ifi dot uzh dot ch>.
// Reference : svo/imu_handler.h.

#pragma once

#include <memory>          // std::shared_ptr
#include <mutex>           // std::mutex

#include <svo/common/types.h>
#include <svo/common/transformation.h>
#include <svo/common/global_pose_settings.h>
#include <fstream>




namespace svo {

class GlobalPoseHandler
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<GlobalPoseHandler> Ptr;
  typedef std::mutex mutex_t;
  typedef std::unique_lock<mutex_t> ulock_t;

  GlobalPoseSettings gp_settings_;

  GlobalPoseHandler(const GlobalPoseSettings& gp_settings);
  ~GlobalPoseHandler();

  /// Get Covariance
  const Eigen::Matrix<double, 6, 6>& getCovariance() const
  {
    return gp_settings_.cov;
  }

  /// Get body - prism distance
  const Eigen::Vector3d& getBodyPrismDistance() const
  {
    return gp_settings_.B_t_PB;
  }

  /// Get body - prism distance
  const Eigen::Quaterniond& getBodyWorldRotation() const
  {
    return gp_settings_.B_q_PB;
  }

  /// Get measurement till a given time.
  /// If a set of measurements is available, get the middle one.
  bool getMeasurementTillTime(
      const double timestamp,
      const double oldest_timestamp,
      GposeMeasurement& extracted_measurement,
      const bool remove_measurements);

  /// Get measurements till a given time.
  bool getMeasurementsTillTime(
      const double timestamp,
      const double oldest_timestamp,
      GposeMeasurements& extracted_measurements,
      const size_t max_num_measurements,
      const int pickup_strategy,
      const bool remove_measurements);

  /// Get measurements in some time interval. Note that you have to provide
  /// the camera timestamps.
  bool getMeasurementsInInterval(
      const double old_cam_timestamp, // seconds
      const double new_cam_timestamp, // seconds
      GposeMeasurements& extracted_measurements);

  bool addGposeMeasurement(const GposeMeasurement& measurement);

  bool addAlignmentGposeMeasurement(const GposeMeasurement& measurement);

  bool addAlignmentVIOMeasurement(const GposeMeasurement& measurement); // GP measurements ? It isnt gp but should contains timestamp + x y z

  static GlobalPoseSettings loadSettingsFromFile(const std::string& filename);

  bool readInitialPose() { return read_initial_pose_; }

  void setReadInitialPose(bool read_initial_pose)
  {
    read_initial_pose_ = read_initial_pose;
  }

  bool setInitialOrientation(const Eigen::Quaterniond& q_W_B);

  bool setInitialPose(const Eigen::Vector3d& t_W_B, const Eigen::Quaterniond& q_W_B);

  bool setT_w_vio(const Transformation& T_w_vio);

  bool loadGposeMeasurementsFromFile(const std::string& filename, const size_t first_meas_id);

  bool alignFrames();

  bool estimateTransformation(Transformation& T_w_vio);

  Transformation getT_w_vio() { return T_w_vio_;}

  void reset();

private:
  int it_trust_alignement = 0; 
  mutable mutex_t measurements_mut_;

  GposeMeasurements measurements_; ///< Newest measurement is at the front of the list
  GposeMeasurements alignment_gp_measurements_; ///< Newest measurement is at the front of the list
  GposeMeasurements alignment_vio_measurements_; ///< Newest measurement is at the front of the list

  bool read_initial_orientation_ = true;
  bool read_initial_pose_ = true;

  // Transformation T_w_vio_init_;
  Transformation T_w_vio_;
  Transformation T_vio_w_;

  Eigen::Quaterniond q_W_B_; /// changes if initial orientation of Body is known.

  Eigen::Matrix3d estimateYawRotation(const Eigen::MatrixXd& data_zero_centered, 
                                    const Eigen::MatrixXd& model_zero_centered) const ;
  void prepareAlignmentData(Eigen::MatrixXd& model, Eigen::MatrixXd& data) const ;
  bool refineAlignment(const Transformation& T_w_vio_curr) ;
  
};

} // namespace svo
