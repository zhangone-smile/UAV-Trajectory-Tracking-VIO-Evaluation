#include <ekf_model.h>
#include <algorithm>
#include <cmath>

namespace ekf_imu_vision {

namespace {

double clampCos(double v) {
  if (std::fabs(v) < 1e-6) return v >= 0.0 ? 1e-6 : -1e-6;
  return v;
}

double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

Mat3x3 eulerToRotation(const Vec3& euler) {
  const double phi = euler(0);
  const double theta = euler(1);
  const double psi = euler(2);

  return (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()))
      .toRotationMatrix();
}

Vec3 rotationToEuler(const Mat3x3& R) {
  const double phi = asin(std::max(-1.0, std::min(1.0, R(2, 1))));
  const double theta = atan2(-R(2, 0), R(2, 2));
  const double psi = atan2(-R(0, 1), R(1, 1));
  return Vec3(wrapAngle(phi), wrapAngle(theta), wrapAngle(psi));
}

Mat3x3 eulerRateMatrix(const Vec3& euler) {
  const double phi = euler(0);
  const double theta = euler(1);
  const double cphi = clampCos(std::cos(phi));
  const double tphi = std::sin(phi) / cphi;
  const double ctheta = std::cos(theta);
  const double stheta = std::sin(theta);

  Mat3x3 T;
  T << ctheta,           0.0, stheta,
       stheta * tphi,    1.0, -ctheta * tphi,
       -stheta / cphi,   0.0, ctheta / cphi;
  return T;
}

}  // namespace

Vec15 modelF(const Vec15& x, const Vec6& u, const Vec12& n) {
  Vec15 xdot = Vec15::Zero();

  const Vec3 euler = x.segment<3>(3);
  const Vec3 bg = x.segment<3>(9);
  const Vec3 ba = x.segment<3>(12);

  const Vec3 gyro = u.segment<3>(0) - bg - n.segment<3>(0);
  const Vec3 acc = u.segment<3>(3) - ba - n.segment<3>(3);

  xdot.segment<3>(0) = x.segment<3>(6);
  xdot.segment<3>(3) = eulerRateMatrix(euler) * gyro;
  xdot.segment<3>(6) = eulerToRotation(euler) * acc + Vec3(0.0, 0.0, -9.81);
  xdot.segment<3>(9) = n.segment<3>(6);
  xdot.segment<3>(12) = n.segment<3>(9);

  return xdot;
}

Mat15x15 jacobiFx(const Vec15& x, const Vec6& u, const Vec12& n) {
  Mat15x15 J = Mat15x15::Zero();
  const double eps = 1e-6;

  for (int i = 0; i < 15; ++i) {
    Vec15 xp = x;
    Vec15 xm = x;
    xp(i) += eps;
    xm(i) -= eps;
    Vec15 diff = modelF(xp, u, n) - modelF(xm, u, n);
    J.col(i) = diff / (2.0 * eps);
  }

  return J;
}

Mat15x12 jacobiFn(const Vec15& x, const Vec6& u, const Vec12& n) {
  Mat15x12 J = Mat15x12::Zero();
  const double eps = 1e-6;

  for (int i = 0; i < 12; ++i) {
    Vec12 np = n;
    Vec12 nm = n;
    np(i) += eps;
    nm(i) -= eps;
    Vec15 diff = modelF(x, u, np) - modelF(x, u, nm);
    J.col(i) = diff / (2.0 * eps);
  }

  return J;
}

/* ============================== model of stereo VO relative pose ============================== */

Vec6 modelG2(const Vec21& x, const Vec6& v) {
  const Vec3 p = x.segment<3>(0);
  const Vec3 euler = x.segment<3>(3);
  const Vec3 pk = x.segment<3>(15);
  const Vec3 euler_k = x.segment<3>(18);

  const Mat3x3 R = eulerToRotation(euler);
  const Mat3x3 Rk = eulerToRotation(euler_k);
  const Mat3x3 R_rel = Rk.transpose() * R;

  Vec6 z;
  z.segment<3>(0) = Rk.transpose() * (p - pk);
  z.segment<3>(3) = rotationToEuler(R_rel);
  z += v;

  for (int i = 3; i < 6; ++i) z(i) = wrapAngle(z(i));
  return z;
}

Mat6x21 jacobiG2x(const Vec21& x, const Vec6& v) {
  Mat6x21 J = Mat6x21::Zero();
  const double eps = 1e-6;

  for (int i = 0; i < 21; ++i) {
    Vec21 xp = x;
    Vec21 xm = x;
    xp(i) += eps;
    xm(i) -= eps;
    Vec6 diff = modelG2(xp, v) - modelG2(xm, v);
    for (int j = 3; j < 6; ++j) diff(j) = wrapAngle(diff(j));
    J.col(i) = diff / (2.0 * eps);
  }

  return J;
}

Mat6x6 jacobiG2v(const Vec21& x, const Vec6& v) {
  (void)x;
  (void)v;
  return Mat6x6::Identity();
}

}  // namespace ekf_imu_vision
