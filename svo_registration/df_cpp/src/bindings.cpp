// FYP Phase 8: pybind11 bindings of the distance-field registration library (Python module fyp_df).
// Same call signature and result keys as analysis/df_register.py:register, so the registration node
// and the equivalence tests can use either implementation interchangeably.
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "fyp_df/df_registration.h"

namespace py = pybind11;
using fyp_df::DistanceGrid;

namespace {

using RowMatX3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

std::vector<Eigen::Vector3d> toPoints(const Eigen::Ref<const RowMatX3>& P)
{
  std::vector<Eigen::Vector3d> pts(P.rows());
  for (Eigen::Index i = 0; i < P.rows(); ++i)
    pts[i] = P.row(i).transpose();
  return pts;
}

py::tuple query(const DistanceGrid& g, const Eigen::Ref<const RowMatX3>& P, py::object mu)
{
  const double m = mu.is_none() ? -1.0 : mu.cast<double>();
  const Eigen::Index n = P.rows();
  py::array_t<double> D(n);
  py::array_t<double> G({static_cast<py::ssize_t>(n), static_cast<py::ssize_t>(3)});
  py::array_t<bool> V(n);
  auto d = D.mutable_unchecked<1>();
  auto gg = G.mutable_unchecked<2>();
  auto v = V.mutable_unchecked<1>();
  for (Eigen::Index i = 0; i < n; ++i)
  {
    double di;
    Eigen::Vector3d gi;
    const bool ok = g.query(P.row(i).transpose(), m, &di, &gi);
    d(i) = ok ? di : std::numeric_limits<double>::quiet_NaN();
    for (int k = 0; k < 3; ++k)
      gg(i, k) = ok ? gi[k] : 0.0;
    v(i) = ok;
  }
  return py::make_tuple(D, G, V);
}

py::dict registerPy(const DistanceGrid& g, const Eigen::Ref<const RowMatX3>& P, py::object mu, double delta,
                    int max_iter, double tol_step, double tol_cost)
{
  fyp_df::Options opt;
  opt.mu = mu.is_none() ? -1.0 : mu.cast<double>();
  opt.delta = delta;
  opt.max_iter = max_iter;
  opt.tol_step = tol_step;
  opt.tol_cost = tol_cost;
  const std::vector<Eigen::Vector3d> pts = toPoints(P);
  fyp_df::Result r;
  {
    py::gil_scoped_release release;
    r = fyp_df::registerCloud(g, pts, opt);
  }
  const size_t n = pts.size();
  py::array_t<double> D(n);
  py::array_t<double> G({static_cast<py::ssize_t>(n), static_cast<py::ssize_t>(3)});
  py::array_t<bool> V(n);
  auto d = D.mutable_unchecked<1>();
  auto gg = G.mutable_unchecked<2>();
  auto v = V.mutable_unchecked<1>();
  for (size_t i = 0; i < n; ++i)
  {
    d(i) = r.valid_untrunc[i] ? r.D_untrunc[i] : std::numeric_limits<double>::quiet_NaN();
    for (int k = 0; k < 3; ++k)
      gg(i, k) = r.valid_untrunc[i] ? r.G_untrunc[i][k] : 0.0;
    v(i) = r.valid_untrunc[i] != 0;
  }
  py::dict out;
  out["T_c"] = r.T_c;
  out["iterations"] = r.iterations;
  out["lookups"] = r.lookups;
  out["runtime_s"] = r.runtime_s;
  out["H_wv"] = r.H_wv;
  out["n_points"] = static_cast<long>(n);
  out["n_valid"] = r.n_valid;
  out["n_outside"] = r.n_outside;
  out["cost"] = r.cost;
  out["D_untrunc"] = D;
  out["G_untrunc"] = G;
  out["valid_untrunc"] = V;
  out["sigma2"] = r.sigma2;
  return out;
}

Eigen::Matrix<double, 6, 6> informationHessianPy(const Eigen::Matrix<double, 6, 6>& H_wv, double sigma2)
{
  fyp_df::Result r;
  r.H_wv = H_wv;
  r.sigma2 = sigma2;
  return fyp_df::informationHessian(r);
}

}  // namespace

PYBIND11_MODULE(fyp_df, m)
{
  m.doc() = "FYP distance-field registration (C++ port of analysis/df_register.py)";
  py::class_<DistanceGrid>(m, "DistanceGrid")
      .def(py::init<const std::string&, const std::string&>(), py::arg("bin_path"), py::arg("expected_twin_md5"))
      .def("query", &query, py::arg("p"), py::arg("mu") = py::none())
      .def("memory_bytes", &DistanceGrid::memoryBytes)
      .def_property_readonly("twin_md5", &DistanceGrid::twinMd5)
      .def_property_readonly("voxel", &DistanceGrid::voxel)
      .def_property_readonly("origin", &DistanceGrid::origin)
      .def_property_readonly("dims", &DistanceGrid::dims);
  m.def("register", &registerPy, py::arg("grid"), py::arg("points"), py::arg("mu") = py::none(),
        py::arg("delta") = 0.05, py::arg("max_iter") = 100, py::arg("tol_step") = 1e-7, py::arg("tol_cost") = 1e-9);
  m.def("information_hessian", &informationHessianPy, py::arg("H_wv"), py::arg("sigma2"));
}
