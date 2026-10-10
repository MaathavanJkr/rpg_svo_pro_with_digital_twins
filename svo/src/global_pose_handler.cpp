// Author : Giovanni Cioffi <cioffi at ifi dot uzh dot ch>.
// Reference : imu_handler.cpp.

#include "svo/global_pose_handler.h"
#include "svo/global_positions_handler.h"

#include <numeric>

#include <svo/common/logging.h>
#include <vikit/math_utils.h>
#include <vikit/csv_utils.h>
#include <vikit/timer.h>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

namespace svo {


GlobalPoseHandler::GlobalPoseHandler(
    const GlobalPoseSettings& gp_settings)
  : gp_settings_(gp_settings)
{
  if(gp_settings_.initial_orientation_known)
  {
    read_initial_orientation_ = true;
  }
  else
  {
    q_W_B_.w() = 1;
    q_W_B_.x() = 0;
    q_W_B_.y() = 0;
    q_W_B_.z() = 0;
  }

}

GlobalPoseHandler::~GlobalPoseHandler()
{
}

//we want the measurements between the last keyframe and the current time-
bool GlobalPoseHandler::getMeasurementTillTime(
    const double timestamp,
    const double oldest_timestamp,
    GposeMeasurement& extracted_measurement,
    const bool remove_measurements)
{
  ulock_t lock(measurements_mut_);

  double oldest_timestamp_new = oldest_timestamp - 0.001; 

  if(measurements_.empty())
  {
    VLOG(10) << "don't have any gp measurements!";
    return false;
  } 


  // Find the first measurement older than timestamp,
  // note that the newest measurement is at the front of the list!
  GposeMeasurements::iterator it_front = measurements_.begin();
  std::size_t num_measurements_newer_timestamp = 0;

  for(; it_front!=measurements_.end(); ++it_front) 
  {
    if(it_front->timestamp_ < timestamp)
    {
      break;
    }
    num_measurements_newer_timestamp += 1;
  }

  if(num_measurements_newer_timestamp == measurements_.size()) // if all measurements have a timestamp newer then the requested one
  {
    VLOG(10) << "need an older gpose measurement!";
    return false;
  }

  // Find the first measurement newer than oldest_timestamp.
  GposeMeasurements::iterator it_back = measurements_.begin();
  std::size_t num_measurements_newer_oldest_timestamp = 0;
  for(; it_back!=measurements_.end(); ++it_back)
  {
    if(it_back->timestamp_ <= oldest_timestamp_new)
    {
      break;
    }
    num_measurements_newer_oldest_timestamp += 1;
  }
  // If there are measurements older than oldest_timestamp,
  // move it to the first measurement newer than oldest_timestamp.
  if (!(num_measurements_newer_oldest_timestamp ==  measurements_.size())) // if not all measurements have a timestamp newer then the requested one
  {
    --it_back;
  }

  // copy affected measurement
  std::size_t num_measurements_available = num_measurements_newer_oldest_timestamp - num_measurements_newer_timestamp;
  if (num_measurements_available == 0)
  {
     VLOG(10) << "no gp measurements available!";
    return false;
  }
  else
  {
    std::size_t idx = num_measurements_newer_timestamp;

    //take the newest measurement


    if (num_measurements_available > 1)
    { }

    extracted_measurement = measurements_.at(idx);
  }

  if (remove_measurements)
  {
    measurements_.erase(it_front, measurements_.end());
  }

  return true;
}

bool GlobalPoseHandler::getMeasurementsTillTime(
    const double timestamp,
    const double oldest_timestamp,
    GposeMeasurements& extracted_measurements,
    const size_t max_num_measurements,
    const int pickup_strategy,
    const bool remove_measurements)
{
  ulock_t lock(measurements_mut_);
  if(measurements_.empty())
  {
    VLOG(10) << "don't have any gp measurements!";
    return false;
  }

  if(max_num_measurements == 0)
  {
    LOG(WARNING) << "Max num gp measurements is zero!";
    return false;
  }

  // Find the first measurement older than timestamp,
  // note that the newest measurement is at the front of the list!
  GposeMeasurements::iterator it_front = measurements_.begin();
  size_t num_measurements_newer_timestamp = 0;
  for(; it_front!=measurements_.end(); ++it_front)
  {
    if(it_front->timestamp_ <= timestamp)
    {
      break;
    }
    num_measurements_newer_timestamp += 1;
  }

  if(num_measurements_newer_timestamp == measurements_.size())
  {
    VLOG(10) << "need an older gp measurement!";
    return false;
  }

  // Find the first measurement newer than oldest_timestamp.
  GposeMeasurements::iterator it_back = measurements_.end();
  it_back--;
  std::size_t num_measurements_older_oldest_timestamp = 0;
  for(; it_back!=measurements_.begin(); --it_back)
  {
    if(it_back->timestamp_ > oldest_timestamp)
    {
      break;
    }
    num_measurements_older_oldest_timestamp += 1;
  }

  // copy affected measurements
  std::size_t num_measurements_available =
          measurements_.size() - num_measurements_older_oldest_timestamp - num_measurements_newer_timestamp;

  if (num_measurements_available > max_num_measurements)
  {
    if (pickup_strategy == 2)
    {
      // Take newest measurements
      const size_t position_oldest_measurement_to_extract = max_num_measurements +
              num_measurements_newer_timestamp;
      extracted_measurements.insert(extracted_measurements.begin(),
                                    it_front,
                                    measurements_.begin()+position_oldest_measurement_to_extract);
    }
    else if(pickup_strategy == 1)
    {
      // Take equidistant measurements      
      std::size_t step = roundToNearestInt(num_measurements_available,
                                           max_num_measurements+1);
      std::deque<std::size_t> pos;
      std::size_t pos_last_meas = num_measurements_older_oldest_timestamp + step;
      pos.push_back(pos_last_meas);

      std::size_t num_measurements_to_retain = max_num_measurements - 1;

      while(num_measurements_to_retain > 0)
      {
        pos_last_meas += step;
        pos.push_back(pos_last_meas);

        num_measurements_to_retain -=1;
      }

      for (std::size_t i = 0; i < pos.size(); ++i)
      {
        std::size_t ind = measurements_.size() - pos.at(i);
        GposeMeasurement iter = measurements_.at(ind);
        extracted_measurements.push_front(iter);
      }
    }
    else if (pickup_strategy == 0)
    {
      extracted_measurements.insert(extracted_measurements.begin(),
                                    it_back-max_num_measurements+1,
                                    it_back+1);
    }
    else
    {
      LOG(ERROR) << "Unknown pickup strategy";
      return false;
    }
  }
  else
  {
    extracted_measurements.insert(extracted_measurements.begin(),
                                  it_front,
                                  it_back+1);
  }

  if (remove_measurements)
  {
    measurements_.erase(it_front, measurements_.end());
  }

  return true;
}

bool GlobalPoseHandler::getMeasurementsInInterval(
    const double old_cam_timestamp,
    const double new_cam_timestamp,
    GposeMeasurements& extracted_measurements)
{
  assert(new_cam_timestamp > old_cam_timestamp);
  ulock_t lock(measurements_mut_);
  if(measurements_.empty())
  {
    LOG(WARNING) << "don't have any gp measurement!";
    return false;
  }

  // Newest measurements are at the front of the list!
  const double t1 = old_cam_timestamp;
  const double t2 = new_cam_timestamp;

  GposeMeasurements::iterator it1=measurements_.end(); // older timestamp
  GposeMeasurements::iterator it2=measurements_.end(); // newer timestamp
  bool it2_set = false;
  for(GposeMeasurements::iterator it=measurements_.begin();
      it!=measurements_.end(); ++it)
  {
    if(!it2_set && it->timestamp_ < t2)
    {
      it2 = it;
      it2_set = true;
    }

    if(it->timestamp_ <= t1)
    {
      it1 = it;
      break;
    }
  }

  // check
  if(it1 == measurements_.begin())
  {
    LOG(WARNING) << "all measurements older than requested interval!";
    return false;
  }

  if(!it2_set)
  {
    LOG(WARNING) << "all measurements newer than requested interval!";
    return false;
  }

  if(it1 == it2)
  {
    if (t1 < it1->timestamp_ && t2 > it2->timestamp_)
    {
      --it2;
    }
    else
    {
      LOG(WARNING) << "no measurements in requested interval!";
      return false;
    }
  }

  // copy affected measurements
  extracted_measurements.insert(extracted_measurements.begin(), it2, it1);

  return true;
}

bool GlobalPoseHandler::addGposeMeasurement(
      const GposeMeasurement& m)
{
  ulock_t lock(measurements_mut_);

  Transformation T_m(
      Quaternion(m.orientation_.w(), m.orientation_.x(), m.orientation_.y(), m.orientation_.z()),
      kindr::minimal::Position(m.position_(0), m.position_(1), m.position_(2))
  );

  // Apply the transformations
  //registration in world frame -> need to add T_vio_w*T_registration*T_w_vio
  auto T_result = T_vio_w_ * T_m ; 

  // Extract the new position and orientation from T_result
  Eigen::Vector3d newPosition = T_result.getPosition();
  Eigen::Quaterniond newOrientation = Eigen::Quaterniond(T_result.getRotation().w(), 
                                                          T_result.getRotation().x(), 
                                                          T_result.getRotation().y(), 
                                                          T_result.getRotation().z());

  // the information also need to be rotated in VIO frame (originally in world frame)
  Eigen::Matrix3d R_vio_w_ = T_vio_w_.getRotationMatrix();
  Eigen::Matrix<double, 6, 6> R(6, 6);
  R.setZero();
  R.topLeftCorner<3, 3>() = R_vio_w_;
  R.bottomRightCorner<3, 3>() = R_vio_w_; // Adjust based on orientation representation

  // get the covariance matrix : 
  Eigen::Matrix<double, 6, 6> covariance = m.information_;

  Eigen::Matrix<double, 6, 6> cov_vio = R * covariance * R.transpose();

  GposeMeasurement m_W = GposeMeasurement(m.timestamp_, newPosition, newOrientation, cov_vio);
  measurements_.push_front(m_W); // new measurement is at the front of the list!

  return true;
}

bool GlobalPoseHandler::addAlignmentGposeMeasurement(
      const GposeMeasurement& m)
{
  ulock_t lock(measurements_mut_);

  if (alignment_gp_measurements_.size() >= 200) {
    alignment_gp_measurements_.pop_back();
  }

  alignment_gp_measurements_.push_front(m); // new at front of queue

  return true;
}

bool GlobalPoseHandler::addAlignmentVIOMeasurement(
      const GposeMeasurement& m)
{
  ulock_t lock(measurements_mut_);
    if (alignment_vio_measurements_.size() >= 200) {
    alignment_vio_measurements_.pop_back();
  }
  alignment_vio_measurements_.push_front(m); // store the gnss measurement


  return true;
}

bool GlobalPoseHandler::setT_w_vio(const Transformation& T_w_vio)
{
  T_w_vio_ = T_w_vio;
  T_vio_w_ = T_w_vio_.inverse();
  return true;
}


bool GlobalPoseHandler::estimateTransformation(Transformation& T_w_vio) {
  ulock_t lock(measurements_mut_);

  if (alignment_vio_measurements_.size() <= 2 || alignment_gp_measurements_.size() <= 2) {
    return false;
  }

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  // Prepare data for alignment.
  Eigen::MatrixXd model, data;
  prepareAlignmentData(model, data);

  // Compute mean-centered model and data.
  Eigen::Vector3d model_mean = model.colwise().mean();
  Eigen::Vector3d data_mean = data.colwise().mean();
  Eigen::MatrixXd model_zero_centered = model.rowwise() - model_mean.transpose();
  Eigen::MatrixXd data_zero_centered = data.rowwise() - data_mean.transpose();

  // Estimate rotation and translation.
  Eigen::Matrix3d rotation = estimateYawRotation(data_zero_centered, model_zero_centered);
  Eigen::Vector3d translation = model_mean - rotation * data_mean;

  // Create the transformation matrix.
  T.block<3, 3>(0, 0) = rotation;
  T.block<3, 1>(0, 3) = translation;
  Transformation T_w_vio_curr(T);

  // Refine registration and calculate Hessian.
  if (!refineAlignment(T_w_vio_curr)) {
    return false;
  }

  T_vio_w_ = Transformation(T);
  T_w_vio_ = T_w_vio = T_vio_w_.inverse();

  // Clear the alignment data after successful estimation.
  alignment_gp_measurements_.clear();
  alignment_vio_measurements_.clear();

  return true;
}

void GlobalPoseHandler::prepareAlignmentData(Eigen::MatrixXd& model, Eigen::MatrixXd& data) const {
  int N = alignment_gp_measurements_.size();
  model.resize(3, N);
  data.resize(3, N);

  for (int i = 0; i < N; ++i) {
    data.col(i) = alignment_gp_measurements_[i].position_;
    model.col(i) = alignment_vio_measurements_[i].position_;
  }

  model.transposeInPlace();
  data.transposeInPlace();
}

Eigen::Matrix3d GlobalPoseHandler::estimateYawRotation(const Eigen::MatrixXd& data_zero_centered, 
                                                      const Eigen::MatrixXd& model_zero_centered) const {
  Eigen::Matrix3d C = data_zero_centered.transpose() * model_zero_centered;
  double A = C(0, 1) - C(1, 0);
  double B = C(0, 0) + C(1, 1);
  double theta = M_PI / 2 - std::atan2(B, A);

  Eigen::AngleAxisd yaw_angle_axis(theta, Eigen::Vector3d::UnitZ());
  return yaw_angle_axis.toRotationMatrix();
}


bool GlobalPoseHandler::refineAlignment(const Transformation& T_w_vio_curr) {
  Eigen::Matrix<double, 3, 6> E_g_j;
  Eigen::Matrix3d C_G_W = T_w_vio_curr.getRotation().toImplementation().toRotationMatrix();
  Eigen::Matrix<double, 6, 6> Hess_error_alignment = Eigen::Matrix<double, 6, 6>::Zero();

  auto computeSkewSymmetricMatrix = [](const Eigen::Vector3d& vec) {
    Eigen::Matrix3d skew;
    skew << 0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;
    return skew;
  };

  for (const auto& measurement : alignment_vio_measurements_) {
    Eigen::Vector3d w_r = measurement.position_;
    Eigen::Vector3d temp = C_G_W * w_r;
    Eigen::Matrix3d skew = computeSkewSymmetricMatrix(temp);

    E_g_j.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    E_g_j.block<3, 3>(0, 3) = skew;

    Hess_error_alignment += E_g_j.transpose() * E_g_j; // + todo: add weight based on covariance of the ICP.
  }

  Eigen::Matrix<double, 6, 6> inv_Hess_error_alignment = Hess_error_alignment.inverse();
  double yaw_variance = std::abs(inv_Hess_error_alignment(5, 5));

  return yaw_variance < gp_settings_.alignement_threshold;
}

bool GlobalPoseHandler::alignFrames() { 
  // Improvement : This should also be called when no global pose / gps data has been
  // recieved in a while to re-align the VIO and World frames

  if(gp_settings_.estimate_world_vio_transform_via_MR) {
    if(!estimateTransformation(T_w_vio_)){
      return false;
    } else {
      SVO_INFO_STREAM("World to local transform estimated! Global measurements can now be added to the optimization");
      return true;
    }
  }

  return true; 
}

bool GlobalPoseHandler::setInitialPose(const Eigen::Vector3d& t_W_B, const Eigen::Quaterniond &q_W_B)
{

  return true;
}

bool GlobalPoseHandler::setInitialOrientation(const Eigen::Quaterniond &q_W_B)
{
  q_W_B_.w() = q_W_B.w();
  q_W_B_.x() = q_W_B.x();
  q_W_B_.y() = q_W_B.y();
  q_W_B_.z() = q_W_B.z();

  return true;
}

GlobalPoseSettings GlobalPoseHandler::loadSettingsFromFile(const std::string& filename)
{
  YAML::Node data = YAML::LoadFile(filename);
  GlobalPoseSettings settings;
  if(data["gpose_settings"].IsDefined())
  {
    settings.alignement_threshold = data["gpose_settings"]["alignement_threshold"].as<double>();
    settings.max_num_residuals = data["gpose_settings"]["max_num_residuals"].as<int>();
    settings.initial_orientation_known = data["gpose_settings"]["initial_orientation_known"].as<bool>();
    settings.refine_T_WL = data["gpose_settings"]["refine_T_WL"].as<bool>();
  }
  else
  {
    LOG(FATAL) << "Could not load Global Pose calibration from file";
  }
  return settings;
}

bool GlobalPoseHandler::loadGposeMeasurementsFromFile(const std::string& filename, const size_t first_meas_id)
{
  return true;
}

void GlobalPoseHandler::reset()
{
  ulock_t lock(measurements_mut_);
  measurements_.clear();
}

} // namespace svo

