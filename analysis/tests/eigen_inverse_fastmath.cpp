#include <Eigen/Dense>
#include <iostream>
// Rigid transform: yaw 130.454 deg about z, t = (0.888, 2.1605, 0.9552). Compare Matrix4d::inverse()
// with the analytic rigid inverse [R^T, -R^T t].
int main() {
  const double psi = 130.454 * M_PI / 180.0;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.block<3, 1>(0, 3) << 0.888, 2.1605, 0.9552;
  Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
  Ti.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
  Ti.block<3, 1>(0, 3) = -(T.block<3, 3>(0, 0).transpose() * T.block<3, 1>(0, 3));
  const Eigen::Matrix4d I = T.inverse();
  // Also a generic (non-rigid) 4x4 and a Matrix4f.
  Eigen::Matrix4d A; A << 4,1,2,0.5, 1,5,1,1, 2,1,6,1, 0.5,1,1,7;
  Eigen::Matrix4f Tf = T.cast<float>();
  std::cout << "max|inv(T)-rigid| " << (I - Ti).cwiseAbs().maxCoeff()
            << "  max|A*inv(A)-I| " << (A * A.inverse() - Eigen::Matrix4d::Identity()).cwiseAbs().maxCoeff()
            << "  max|Tf*inv(Tf)-I| (float) " << (Tf * Tf.inverse() - Eigen::Matrix4f::Identity()).cwiseAbs().maxCoeff()
            << "  max|Matrix3d inv| " << (T.block<3,3>(0,0).inverse() - Ti.block<3,3>(0,0)).cwiseAbs().maxCoeff() << std::endl;
  return 0;
}
