// FYP Phase 8: distance-field registration (M3), C++ port of analysis/df_register.py.
// Unsigned distance grid (built offline by analysis/df_grid.py), trilinear interpolation with its
// analytic gradient, Levenberg-Marquardt on SE(3) with Huber IRLS weights. Same conventions as M2:
// the correction T_c is applied on the left in the map frame (p' = R_c p + t_c), init = identity.
// Built with standard flags (no -ffast-math), see svo_registration/df_cpp/CMakeLists.txt.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fyp_df {

class DistanceGrid
{
public:
  // Loads <bin_path> and its header <bin_path>.json and verifies: format "fyp_udf_v1", dtype float32,
  // dims * 4 == header bytes == file size, FNV-1a 64-bit checksum of the payload, and (if not empty)
  // the twin md5 recorded in the header against expected_twin_md5. Throws std::runtime_error on failure.
  DistanceGrid(const std::string& bin_path, const std::string& expected_twin_md5);

  // D(p) and dD/dp at one point. Returns false if p is outside the grid (no interpolation cell).
  // mu < 0: untruncated; otherwise D = min(D, mu) and the gradient is zero where D >= mu.
  bool query(const Eigen::Vector3d& p, double mu, double* D, Eigen::Vector3d* G) const;

  size_t memoryBytes() const { return d_.size() * sizeof(float); }
  const std::string& twinMd5() const { return twin_md5_; }
  double voxel() const { return v_; }
  Eigen::Vector3d origin() const { return origin_; }
  Eigen::Vector3i dims() const { return Eigen::Vector3i(nx_, ny_, nz_); }

private:
  float at(int i, int j, int k) const { return d_[(static_cast<size_t>(i) * ny_ + j) * nz_ + k]; }
  std::vector<float> d_;
  int nx_ = 0, ny_ = 0, nz_ = 0;
  Eigen::Vector3d origin_;
  double v_ = 0.0;
  std::string twin_md5_;
};

struct Options
{
  double mu = -1.0;        // truncation distance; < 0 = untruncated
  double delta = 0.05;     // Huber threshold [m]
  int max_iter = 100;
  double tol_step = 1e-7;  // |dx| stopping criterion
  double tol_cost = 1e-9;  // relative cost decrease stopping criterion
};

struct Result
{
  Eigen::Matrix4d T_c = Eigen::Matrix4d::Identity();
  int iterations = 0;
  long lookups = 0;
  double cost = 0.0;
  Eigen::Matrix<double, 6, 6> H_wv = Eigen::Matrix<double, 6, 6>::Zero();  // xi = (w, v)
  double sigma2 = 0.0;
  int n_valid = 0;
  int n_outside = 0;
  // Untruncated distance/gradient at the corrected points (validity checks and information).
  std::vector<double> D_untrunc;
  std::vector<Eigen::Vector3d> G_untrunc;
  std::vector<uint8_t> valid_untrunc;
  double runtime_s = 0.0;
};

// points: N x 3, map frame (the node's filtered local cloud).
Result registerCloud(const DistanceGrid& grid, const std::vector<Eigen::Vector3d>& points, const Options& opt);

// Hessian-based information J^T W J / sigma^2, reordered from xi = (w, v) to [pos, rot] (D4 ablation).
Eigen::Matrix<double, 6, 6> informationHessian(const Result& r);

}  // namespace fyp_df
