// Author : Giovanni Cioffi <cioffi at ifi dot uzh dot ch>.
// Reference : imu_handler.cpp.

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

GlobalPositionsHandler::GlobalPositionsHandler(
    const GlobalPositionsSettings& gp_settings)
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

  proj = GeographicLib::LocalCartesian(originLatitude, originLongitude, originAltitude);
}

GlobalPositionsHandler::~GlobalPositionsHandler()
{}

bool GlobalPositionsHandler::getMeasurementTillTime(
    const double timestamp,
    const double oldest_timestamp,
    GpMeasurement& extracted_measurement,
    const bool remove_measurements)
{
  ulock_t lock(measurements_mut_);

  if(measurements_.empty())
  {
    VLOG(10) << "don't have any gp measurements!";
    return false;
  }

  // Find the first measurement older than timestamp,
  // note that the newest measurement is at the front of the list!
  GpMeasurements::iterator it_front = measurements_.begin();
  std::size_t num_measurements_newer_timestamp = 0;

    for (auto it = measurements_.begin(); it != measurements_.end(); ++it) {
    }

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
  GpMeasurements::iterator it_back = measurements_.begin();
  std::size_t num_measurements_newer_oldest_timestamp = 0;
  for(; it_back!=measurements_.end(); ++it_back)
  {
    if(it_back->timestamp_ <= oldest_timestamp)
    {
      break;
    }
    num_measurements_newer_oldest_timestamp += 1;
  }
  // If there are measurements older than oldest_timestamp,
  // move it to the first measurement newer than oldest_timestamp.
  if (!(num_measurements_newer_oldest_timestamp ==  measurements_.size()))
  {
    --it_back;
  }

  // copy affected measurement
  std::size_t num_measurements_available =
          num_measurements_newer_oldest_timestamp - num_measurements_newer_timestamp;
  if (num_measurements_available == 0)
  {
     VLOG(10) << "no gp measurements available!";
    return false;
  }
  else
  {
    std::size_t idx = num_measurements_newer_timestamp;
    // If more than 1 measurement available, pick the one at the center.
    if (num_measurements_available > 1)
    {
      std::size_t incr = num_measurements_available / 2;
      idx += incr;
    }

    extracted_measurement = measurements_.at(idx);
  }

  if (remove_measurements)
  {
    measurements_.erase(it_front, measurements_.end());
  }

  return true;
}

bool GlobalPositionsHandler::getMeasurementsTillTime(
    const double timestamp,
    const double oldest_timestamp,
    GpMeasurements& extracted_measurements,
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
  GpMeasurements::iterator it_front = measurements_.begin();
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
  GpMeasurements::iterator it_back = measurements_.end();
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
        GpMeasurement iter = measurements_.at(ind);
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

bool GlobalPositionsHandler::getMeasurementsInInterval(
    const double old_cam_timestamp,
    const double new_cam_timestamp,
    GpMeasurements& extracted_measurements)
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

  GpMeasurements::iterator it1=measurements_.end(); // older timestamp
  GpMeasurements::iterator it2=measurements_.end(); // newer timestamp
  bool it2_set = false;
  for(GpMeasurements::iterator it=measurements_.begin();
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

bool GlobalPositionsHandler::addGpMeasurement(const GpMeasurement& m)
{
  ulock_t lock(measurements_mut_);

  if (gp_settings_.coordinates == GlobalPositionsSettings::CoordinateType::GEOGRAPHIC)
  {
    // Convert from geographic to local cartesian.
    double x, y, z;
    proj.Forward(m.position_(0), m.position_(1), m.position_(2), x, y, z);
    Eigen::Vector3d position(x, y, z);
    GpMeasurement m_W = GpMeasurement(m.timestamp_, position); 
    measurements_.push_front(m_W); // new measurement is at the front of the list!
  }
  else
  {
    Eigen::Vector3d w_position = m.position_;
    Eigen::Vector3d vio_position = T_vio_w_*w_position;

    GpMeasurement m_W = GpMeasurement(m.timestamp_, vio_position); // gp meas from world frame to vio frame.
    measurements_.push_front(m_W); // new measurement is at the front of the list!

  }

  return true;
}

bool GlobalPositionsHandler::addAlignmentGpMeasurement(const GpMeasurement& m)
{
  ulock_t lock(measurements_mut_);

  if (alignment_gp_measurements_.size() >= 100) {
    // If so, remove the oldest measurement from the back.
    alignment_gp_measurements_.pop_back();
  }

  if (gp_settings_.coordinates == GlobalPositionsSettings::CoordinateType::GEOGRAPHIC) {
    // Convert from geographic to local cartesian.

    double x, y, z;
    proj.Forward(m.position_(0), m.position_(1), m.position_(2), x, y, z);
    Eigen::Vector3d position(x, y, z);

    GpMeasurement m_W = GpMeasurement(m.timestamp_, position); // gp meas from world frame to vio frame.
    alignment_gp_measurements_.push_front(m_W); // new measurement is at the front of the list!

  } else {
    alignment_gp_measurements_.push_front(m); // store the gnss measurement
  }

  return true;
}

bool GlobalPositionsHandler::addAlignmentVIOMeasurement(
      const GpMeasurement& m)
{
  ulock_t lock(measurements_mut_);

  if (alignment_vio_measurements_.size() >= 100) {
    // If so, remove the oldest measurement from the back.
    alignment_vio_measurements_.pop_back();

  }
  alignment_vio_measurements_.push_front(m); // store the gnss measurement

  return true;
}

bool GlobalPositionsHandler::PublishLocalEstimateInWorldFrame(Transformation& l_T) {
  return true;
}

bool GlobalPositionsHandler::setT_w_vio(const Transformation& T_w_vio)
{
  T_w_vio_ = T_w_vio;
  T_vio_w_ = T_w_vio_.inverse();
  return true;
}


bool GlobalPositionsHandler::estimateTransformation(Transformation& T_w_vio) {
  ulock_t lock(measurements_mut_);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  if (!gp_settings_.estimate_world_vio_transform) {
    LOG(WARNING) << "Not estimating world to VIO transform, setting to identity";
    T_w_vio_ = Transformation(T);
    return true;
  }

  if (alignment_vio_measurements_.size() != alignment_gp_measurements_.size()) {
    LOG(FATAL) << "Measurement sets have to be of the same size!";
  }

  if (alignment_vio_measurements_.size() < 2) {
    return false;  // Not enough measurements.
  }

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
    T_vio_w_ = Transformation(T);
    T_w_vio_ = T_w_vio = T_vio_w_.inverse();
    return false;
  }

  SVO_INFO_STREAM("GPS Alignment done.");


  T_vio_w_ = Transformation(T);
  T_w_vio_ = T_w_vio = T_vio_w_.inverse();
  Transformation Tinv = T_w_vio_.inverse();
  LOG(INFO) << "Computed T_World_vio transform: \n" << Tinv;

  // Clear the alignment data after successful estimation.
  alignment_gp_measurements_.clear();
  alignment_vio_measurements_.clear();

  return true;
}

void GlobalPositionsHandler::prepareAlignmentData(Eigen::MatrixXd& model, Eigen::MatrixXd& data) const {
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

Eigen::Matrix3d GlobalPositionsHandler::estimateYawRotation(const Eigen::MatrixXd& data_zero_centered, const Eigen::MatrixXd& model_zero_centered) const {
  Eigen::Matrix3d C = data_zero_centered.transpose() * model_zero_centered;
  double A = C(0, 1) - C(1, 0);
  double B = C(0, 0) + C(1, 1);
  double theta = M_PI / 2 - std::atan2(B, A);

  Eigen::AngleAxisd yaw_angle_axis(theta, Eigen::Vector3d::UnitZ());
  return yaw_angle_axis.toRotationMatrix();
}


bool GlobalPositionsHandler::refineAlignment(const Transformation& T_w_vio_curr) {
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

    Hess_error_alignment += E_g_j.transpose() * E_g_j; // + todo: add weight based on covariance of the GPS.
  }

  Eigen::Matrix<double, 6, 6> inv_Hess_error_alignment = Hess_error_alignment.inverse();
  double yaw_variance = std::abs(inv_Hess_error_alignment(5, 5));

  return yaw_variance < gp_settings_.alignement_threshold;
}

bool GlobalPositionsHandler::calculatecovariances() {

  Eigen::Matrix<double, 3, 6> E_g_j;
  Eigen::Matrix3d C_G_W = T_w_vio_.getRotation().toImplementation().toRotationMatrix();

  Eigen::Vector3d w_r ; 
  Eigen::Matrix<double, 6, 6> Hess_error_alignement;

  Hess_error_alignement = Eigen::Matrix<double, 6, 6>::Zero();

  for(size_t i = 0; i < alignment_vio_measurements_.size(); i++) {

    w_r = alignment_vio_measurements_[i].position_;

    Eigen::Vector3d temp = C_G_W * w_r;
    Eigen::Matrix3d skew;

    skew << 0, -temp(2), temp(1),
                temp(2), 0, -temp(0),
                -temp(1), temp(0), 0;

    E_g_j.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    E_g_j.block<3, 3>(0, 3) = skew;
    Hess_error_alignement += E_g_j.transpose() * E_g_j; // + add weight based on covariance of the GPS.

  }

  Eigen::Matrix<double, 6, 6> inv_Hess_error_alignement; // https://max.pm/posts/hessian_ls/
  inv_Hess_error_alignement = Hess_error_alignement.inverse(); // prob no need just use the method that calculates single term.

  return true; // add some checks like inversibility

} 

bool GlobalPositionsHandler::alignFrames() { // will be called also when gnss loss

  if(!estimateTransformation(T_w_vio_)){
    return false;
  }

  T_w_vio_init_ = T_w_vio_; 

  read_initial_pose_ = false;
  T_w_vio_Initialized = true;

  return true; 
}

Transformation GlobalPositionsHandler::getT_w_vio() {
  return T_w_vio_;
}
Transformation GlobalPositionsHandler::getT_w_vio_temp() {
  return T_w_vio_temp;
}
bool GlobalPositionsHandler::setInitialPose(const Eigen::Vector3d& t_W_B, const Eigen::Quaterniond &q_W_B)
{
  if(read_initial_orientation_)
  {
    T_w_vio_init_ = Transformation(t_W_B, q_W_B);
  }
  else
  {
    T_w_vio_init_ = Transformation(t_W_B, Eigen::Quaterniond(1,0,0,0));
  }
  read_initial_pose_ = false;
  return true;
}

bool GlobalPositionsHandler::setInitialOrientation(const Eigen::Quaterniond &q_W_B)
{
  q_W_B_.w() = q_W_B.w();
  q_W_B_.x() = q_W_B.x();
  q_W_B_.y() = q_W_B.y();
  q_W_B_.z() = q_W_B.z();

  return true;
}

GlobalPositionsSettings GlobalPositionsHandler::loadSettingsFromFile(const std::string& filename)
{
  YAML::Node data = YAML::LoadFile(filename);
  GlobalPositionsSettings settings;
  if(data["gp_settings"].IsDefined())
  {
    settings.cov(0,0) = data["gp_settings"]["variance_x"].as<double>();
    settings.cov(1,1) = data["gp_settings"]["variance_y"].as<double>();
    settings.cov(2,2) = data["gp_settings"]["variance_z"].as<double>();

    settings.B_t_PB(0) = data["gp_settings"]["B_t_PB"][0].as<double>();
    settings.B_t_PB(1) = data["gp_settings"]["B_t_PB"][1].as<double>();
    settings.B_t_PB(2) = data["gp_settings"]["B_t_PB"][2].as<double>();

    settings.max_num_residuals = data["gp_settings"]["max_num_residuals"].as<int>();

    settings.initial_orientation_known = data["gp_settings"]["initial_orientation_known"].as<bool>();
    settings.estimate_world_vio_transform = data["gp_settings"]["estimate_world_vio_transform"].as<bool>();

    settings.alignement_threshold = data["gp_settings"]["alignement_threshold"].as<double>();
    settings.alignement_speed_min = data["gp_settings"]["alignement_speed_min"].as<double>();
    settings.alignement_d_min = data["gp_settings"]["alignement_d_min"].as<double>();

    std::string coord_type_str = data["gp_settings"]["coordinates"].as<std::string>();
    if (coord_type_str == "local")
    {
        settings.coordinates = svo::GlobalPositionsSettings::CoordinateType::LOCAL;
    } else if (coord_type_str == "geographic")
    {
      settings.coordinates = svo::GlobalPositionsSettings::CoordinateType::GEOGRAPHIC;
    } else {
        LOG(FATAL) << "Unknown coordinates type in configuration: " << coord_type_str  << std::endl; 
    }

  }
  else
  {
    LOG(FATAL) << "Could not load Global Positions calibration from file";
  }
  return settings;
}

bool GlobalPositionsHandler::loadGpMeasurementsFromFile(const std::string& filename, const size_t first_meas_id)
{
  return true;
}

void GlobalPositionsHandler::reset()
{
  ulock_t lock(measurements_mut_);
  measurements_.clear();
}

} // namespace svo

