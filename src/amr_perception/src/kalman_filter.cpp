// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// CWNA 등속 칼만 필터 구현.

#include "amr_perception/kalman_filter.hpp"

#include <Eigen/Dense>

#include <memory>

namespace amr_perception
{

namespace
{
using Mat24 = Eigen::Matrix<double, 2, 4>;
using Mat42 = Eigen::Matrix<double, 4, 2>;

Mat24 observationMatrix()
{
  Mat24 h = Mat24::Zero();
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  return h;
}
}  // namespace

double Innovation::mahalanobis2() const
{
  return nu.dot(S.ldlt().solve(nu));
}

ConstantVelocityKalmanFilter::ConstantVelocityKalmanFilter(
  double q, const Vec2 & z0, const Mat2 & R0, double init_velocity_std)
: q_(q)
{
  x_ << z0.x(), z0.y(), 0.0, 0.0;
  P_.setZero();
  P_.topLeftCorner<2, 2>() = R0;
  const double var_v = init_velocity_std * init_velocity_std;
  P_(2, 2) = var_v;
  P_(3, 3) = var_v;
}

Mat4 ConstantVelocityKalmanFilter::transition(double dt)
{
  Mat4 f = Mat4::Identity();
  f(0, 2) = dt;
  f(1, 3) = dt;
  return f;
}

Mat4 ConstantVelocityKalmanFilter::processNoise(double q, double dt)
{
  // 연속 백색 가속도 모델의 이산화: ∫ F(s) G q G^T F(s)^T ds
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  Mat4 qm = Mat4::Zero();
  qm(0, 0) = qm(1, 1) = dt3 / 3.0;
  qm(0, 2) = qm(2, 0) = dt2 / 2.0;
  qm(1, 3) = qm(3, 1) = dt2 / 2.0;
  qm(2, 2) = qm(3, 3) = dt;
  return q * qm;
}

void ConstantVelocityKalmanFilter::predict(double dt)
{
  if (dt <= 0.0) {
    return;
  }
  const Mat4 f = transition(dt);
  x_ = f * x_;
  P_ = f * P_ * f.transpose() + processNoise(q_, dt);
  P_ = 0.5 * (P_ + P_.transpose());
}

Innovation ConstantVelocityKalmanFilter::innovation(const Vec2 & z, const Mat2 & R) const
{
  Innovation inn;
  inn.nu = z - x_.head<2>();
  inn.S = P_.topLeftCorner<2, 2>() + R;
  return inn;
}

void ConstantVelocityKalmanFilter::update(const Vec2 & z, const Mat2 & R)
{
  const Mat24 h = observationMatrix();
  const Innovation inn = innovation(z, R);
  // K = P H^T S^-1  (S 대칭 → LDLT 로 풀기)
  const Mat42 pht = P_ * h.transpose();
  const Mat42 k = inn.S.ldlt().solve(pht.transpose()).transpose();
  x_ += k * inn.nu;
  const Mat4 ikh = Mat4::Identity() - k * h;
  P_ = ikh * P_ * ikh.transpose() + k * R * k.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
}

Vec2 ConstantVelocityKalmanFilter::predictPosition(double tau) const
{
  return x_.head<2>() + tau * x_.tail<2>();
}

Mat2 ConstantVelocityKalmanFilter::predictPositionCovariance(double tau) const
{
  const Mat2 ppp = P_.topLeftCorner<2, 2>();
  const Mat2 ppv = P_.topRightCorner<2, 2>();
  const Mat2 pvv = P_.bottomRightCorner<2, 2>();
  return ppp + tau * (ppv + ppv.transpose()) + tau * tau * pvv +
         (q_ * tau * tau * tau / 3.0) * Mat2::Identity();
}

std::unique_ptr<TrackFilter> ConstantVelocityKalmanFilter::clone() const
{
  return std::make_unique<ConstantVelocityKalmanFilter>(*this);
}

void ConstantVelocityKalmanFilter::setState(const Vec4 & x, const Mat4 & P)
{
  x_ = x;
  P_ = P;
}

}  // namespace amr_perception
