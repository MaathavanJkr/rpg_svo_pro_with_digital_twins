// FYP Phase 8: C++ port of analysis/df_register.py (same algorithm, same constants).
#include "fyp_df/df_registration.h"

#if defined(__FAST_MATH__)
#error "fyp_df must be built without -ffast-math (see svo_registration/df_cpp/CMakeLists.txt)"
#endif

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace fyp_df {

namespace {

std::string readFile(const std::string& path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cannot open " + path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// Minimal parser for the flat header written by analysis/df_grid.py:export_binary.
std::string jsonString(const std::string& js, const std::string& key)
{
  std::smatch m;
  if (!std::regex_search(js, m, std::regex("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"")))
    throw std::runtime_error("grid header: missing string key " + key);
  return m[1];
}

double jsonNumber(const std::string& js, const std::string& key)
{
  std::smatch m;
  if (!std::regex_search(js, m, std::regex("\"" + key + "\"\\s*:\\s*([-+0-9.eE]+)")))
    throw std::runtime_error("grid header: missing number key " + key);
  return std::stod(m[1]);
}

std::vector<double> jsonArray(const std::string& js, const std::string& key)
{
  std::smatch m;
  if (!std::regex_search(js, m, std::regex("\"" + key + "\"\\s*:\\s*\\[([^\\]]*)\\]")))
    throw std::runtime_error("grid header: missing array key " + key);
  std::vector<double> out;
  std::stringstream ss(m[1].str());
  std::string tok;
  while (std::getline(ss, tok, ','))
    out.push_back(std::stod(tok));
  return out;
}

uint64_t fnv1a64(const char* data, size_t n)
{
  uint64_t h = 0xcbf29ce484222325ULL;
  for (size_t i = 0; i < n; ++i)
  {
    h ^= static_cast<unsigned char>(data[i]);
    h *= 0x100000001b3ULL;
  }
  return h;
}

inline double huberW(double r, double delta)
{
  const double a = std::abs(r);
  return a <= delta ? 1.0 : delta / std::max(a, 1e-12);
}

inline double huberCost(double r, double delta)
{
  const double a = std::abs(r);
  return a <= delta ? 0.5 * a * a : delta * (a - 0.5 * delta);
}

// Same rotation as scipy Rotation.from_rotvec(w).as_matrix().
Eigen::Matrix3d expSO3(const Eigen::Vector3d& w)
{
  const double th = w.norm();
  if (th < 1e-300)
    return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(th, w / th).toRotationMatrix();
}

struct Eval
{
  std::vector<Eigen::Vector3d> q;
  std::vector<double> D;
  std::vector<Eigen::Vector3d> G;
  std::vector<uint8_t> valid;
  int n_valid = 0;
};

void evaluate(const DistanceGrid& grid, const std::vector<Eigen::Vector3d>& pts, const Eigen::Matrix3d& R,
              const Eigen::Vector3d& t, double mu, Eval* e)
{
  const size_t n = pts.size();
  e->q.resize(n); e->D.resize(n); e->G.resize(n); e->valid.resize(n);
  e->n_valid = 0;
  for (size_t i = 0; i < n; ++i)
  {
    e->q[i] = R * pts[i] + t;
    const bool ok = grid.query(e->q[i], mu, &e->D[i], &e->G[i]);
    e->valid[i] = ok;
    e->n_valid += ok;
  }
}

double costOf(const Eval& e, double delta)
{
  double c = 0.0;
  for (size_t i = 0; i < e.D.size(); ++i)
    if (e.valid[i])
      c += huberCost(e.D[i], delta);
  return c;
}

void normalEquations(const Eval& e, double delta, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b)
{
  H->setZero();
  b->setZero();
  for (size_t i = 0; i < e.D.size(); ++i)
  {
    if (!e.valid[i])
      continue;
    Eigen::Matrix<double, 6, 1> J;
    J.head<3>() = e.q[i].cross(e.G[i]);  // g^T(-[q]x) = (q x g)^T
    J.tail<3>() = e.G[i];
    const double w = huberW(e.D[i], delta);
    H->noalias() += w * J * J.transpose();
    b->noalias() += w * e.D[i] * J;
  }
}

}  // namespace

DistanceGrid::DistanceGrid(const std::string& bin_path, const std::string& expected_twin_md5)
{
  const std::string hdr = readFile(bin_path + ".json");
  if (jsonString(hdr, "format") != "fyp_udf_v1")
    throw std::runtime_error("grid header: unexpected format");
  if (jsonString(hdr, "dtype") != "float32")
    throw std::runtime_error("grid header: dtype must be float32");
  const std::vector<double> dims = jsonArray(hdr, "dims");
  const std::vector<double> origin = jsonArray(hdr, "origin");
  if (dims.size() != 3 || origin.size() != 3)
    throw std::runtime_error("grid header: dims/origin must have 3 entries");
  nx_ = static_cast<int>(dims[0]); ny_ = static_cast<int>(dims[1]); nz_ = static_cast<int>(dims[2]);
  origin_ = Eigen::Vector3d(origin[0], origin[1], origin[2]);
  v_ = jsonNumber(hdr, "voxel");
  twin_md5_ = jsonString(hdr, "twin_md5");
  if (!expected_twin_md5.empty() && expected_twin_md5 != twin_md5_)
    throw std::runtime_error("grid header: twin md5 " + twin_md5_ + " != expected " + expected_twin_md5);
  const size_t n = static_cast<size_t>(nx_) * ny_ * nz_;
  const size_t bytes = static_cast<size_t>(jsonNumber(hdr, "bytes"));
  // Read the payload straight into the grid buffer (no second copy, so peak memory = grid size).
  std::ifstream f(bin_path, std::ios::binary | std::ios::ate);
  if (!f)
    throw std::runtime_error("cannot open " + bin_path);
  const size_t file_size = static_cast<size_t>(f.tellg());
  if (bytes != n * sizeof(float) || file_size != bytes)
    throw std::runtime_error("grid: size mismatch (dims, header bytes, file size)");
  d_.resize(n);
  f.seekg(0);
  if (!f.read(reinterpret_cast<char*>(d_.data()), static_cast<std::streamsize>(bytes)))
    throw std::runtime_error("grid: read failed");
  std::ostringstream cs;
  cs << std::hex;
  cs.width(16); cs.fill('0');
  cs << fnv1a64(reinterpret_cast<const char*>(d_.data()), bytes);
  if (cs.str() != jsonString(hdr, "checksum_fnv1a64"))
  {
    d_.clear();
    throw std::runtime_error("grid: checksum mismatch");
  }
}

bool DistanceGrid::query(const Eigen::Vector3d& p, double mu, double* D, Eigen::Vector3d* G) const
{
  const Eigen::Vector3d g = (p - origin_) / v_;
  const int x = static_cast<int>(std::floor(g.x()));
  const int y = static_cast<int>(std::floor(g.y()));
  const int z = static_cast<int>(std::floor(g.z()));
  if (x < 0 || y < 0 || z < 0 || x >= nx_ - 1 || y >= ny_ - 1 || z >= nz_ - 1)
    return false;
  const double fx = g.x() - x, fy = g.y() - y, fz = g.z() - z;
  double c[2][2][2];
  for (int a = 0; a < 2; ++a)
    for (int b = 0; b < 2; ++b)
      for (int cc = 0; cc < 2; ++cc)
        c[a][b][cc] = static_cast<double>(at(x + a, y + b, z + cc));
  const double c00 = c[0][0][0] * (1 - fx) + c[1][0][0] * fx;
  const double c01 = c[0][0][1] * (1 - fx) + c[1][0][1] * fx;
  const double c10 = c[0][1][0] * (1 - fx) + c[1][1][0] * fx;
  const double c11 = c[0][1][1] * (1 - fx) + c[1][1][1] * fx;
  const double c0 = c00 * (1 - fy) + c10 * fy;
  const double c1 = c01 * (1 - fy) + c11 * fy;
  double Dv = c0 * (1 - fz) + c1 * fz;
  const double dx = ((c[1][0][0] - c[0][0][0]) * (1 - fy) + (c[1][1][0] - c[0][1][0]) * fy) * (1 - fz)
                  + ((c[1][0][1] - c[0][0][1]) * (1 - fy) + (c[1][1][1] - c[0][1][1]) * fy) * fz;
  const double dy = (c10 - c00) * (1 - fz) + (c11 - c01) * fz;
  const double dz = c1 - c0;
  Eigen::Vector3d Gv(dx / v_, dy / v_, dz / v_);
  if (mu >= 0.0 && Dv >= mu)
  {
    Dv = mu;
    Gv.setZero();
  }
  *D = Dv;
  *G = Gv;
  return true;
}

Result registerCloud(const DistanceGrid& grid, const std::vector<Eigen::Vector3d>& points, const Options& opt)
{
  const auto t_start = std::chrono::steady_clock::now();
  Result res;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double lam = 1e-3;
  const long n = static_cast<long>(points.size());

  Eval cur, trial;
  evaluate(grid, points, R, t, opt.mu, &cur);
  res.lookups += n;
  double cost = costOf(cur, opt.delta);
  int it = 0;
  Eigen::Matrix<double, 6, 6> H;
  Eigen::Matrix<double, 6, 1> b;
  for (it = 1; it <= opt.max_iter; ++it)
  {
    normalEquations(cur, opt.delta, &H, &b);
    bool accepted = false;
    Eigen::Matrix<double, 6, 1> dxv;
    Eigen::Matrix3d Rn;
    Eigen::Vector3d tn;
    double cn = 0.0;
    while (lam < 1e8)
    {
      Eigen::Matrix<double, 6, 6> A = H;
      for (int k = 0; k < 6; ++k)
        A(k, k) += lam * (H(k, k) + 1e-12);
      dxv = A.partialPivLu().solve(-b);  // numpy.linalg.solve uses LU with partial pivoting
      const Eigen::Matrix3d dR = expSO3(dxv.head<3>());
      Rn = dR * R;
      tn = dR * t + dxv.tail<3>();
      evaluate(grid, points, Rn, tn, opt.mu, &trial);
      res.lookups += n;
      cn = costOf(trial, opt.delta) * (static_cast<double>(cur.n_valid) / std::max(trial.n_valid, 1));
      if (cn <= cost)
      {
        accepted = true;
        break;
      }
      lam *= 10.0;
    }
    if (!accepted)
      break;
    const double rel = (cost - cn) / std::max(cost, 1e-12);
    R = Rn; t = tn;
    std::swap(cur, trial);
    cost = cn;
    lam = std::max(lam / 10.0, 1e-9);
    if (dxv.norm() < opt.tol_step || rel < opt.tol_cost)
      break;
  }
  if (it > opt.max_iter)
    it = opt.max_iter;
  res.iterations = it;
  res.T_c.block<3, 3>(0, 0) = R;
  res.T_c.block<3, 1>(0, 3) = t;
  // Gauss-Newton Hessian at the final estimate.
  normalEquations(cur, opt.delta, &H, &b);
  res.H_wv = H;
  res.cost = cost;
  res.n_valid = cur.n_valid;
  // Untruncated distance at the final estimate (validity checks, information).
  Eval fin;
  evaluate(grid, points, R, t, -1.0, &fin);
  res.lookups += n;
  res.D_untrunc = fin.D;
  res.G_untrunc = fin.G;
  res.valid_untrunc = fin.valid;
  res.n_outside = static_cast<int>(n) - fin.n_valid;
  double s = 0.0;
  for (size_t i = 0; i < cur.D.size(); ++i)
    if (cur.valid[i])
      s += huberW(cur.D[i], opt.delta) * cur.D[i] * cur.D[i];
  res.sigma2 = s / std::max(cur.n_valid - 6, 1);
  res.runtime_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  return res;
}

Eigen::Matrix<double, 6, 6> informationHessian(const Result& r)
{
  const Eigen::Matrix<double, 6, 6> H = r.H_wv / std::max(r.sigma2, 1e-12);
  Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
  P.block<3, 3>(0, 3).setIdentity();
  P.block<3, 3>(3, 0).setIdentity();
  return P * H * P.transpose();
}

}  // namespace fyp_df
