#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdio>
// Prints results of Eigen double ops used by SVO / minkindr; compare between flag sets.
int main() {
  Eigen::Quaterniond a(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
  Eigen::Quaterniond b(Eigen::AngleAxisd(-1.9, Eigen::Vector3d(-2, 1, 0.5).normalized()));
  Eigen::Vector3d v(0.3, -1.2, 2.5);
  Eigen::Quaterniond p = a * b, c = a.conjugate(), ai = a.inverse();
  Eigen::Vector3d r = a * v;
  Eigen::Matrix<double, 6, 6> H; for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) H(i, j) = (i == j ? 7.0 : 0.0) + 0.1 * (i + 1) * (j + 1) + 0.05 * ((i * 7 + j * 3) % 5);
  Eigen::Matrix<double, 6, 6> Hi = H.inverse();
  Eigen::Matrix3d R = a.toRotationMatrix(), Ri = R.inverse();
  Eigen::Matrix4d M; M << 4,1,2,0.5, 1,5,1,1, 2,1,6,1, 0.5,1,1,7; Eigen::Matrix4d MM = M * M;
  Eigen::Vector4d x = M.ldlt().solve(Eigen::Vector4d(1,2,3,4));
  printf("prod %.12f %.12f %.12f %.12f\n", p.w(), p.x(), p.y(), p.z());
  printf("conj %.12f %.12f %.12f %.12f | inv %.12f %.12f %.12f %.12f\n", c.w(), c.x(), c.y(), c.z(), ai.w(), ai.x(), ai.y(), ai.z());
  printf("rot  %.12f %.12f %.12f\n", r.x(), r.y(), r.z());
  printf("6x6inv %.12f %.12f | 3x3inv %.12f %.12f | 4x4mul %.12f | ldlt %.12f %.12f\n", Hi(0,0), Hi(2,5), Ri(0,1), Ri(2,0), MM(1,3), x(0), x(3));
  return 0;
}
