// 휠 오도메트리 공분산 모델 구현. 유도는 docs/algorithms/kinematics.md §3.

#include "amr_localization/odometry_covariance.hpp"

#include <cmath>

namespace amr_localization
{

StepJacobians exactArcJacobians(double theta, const BodyIncrement & increment, double separation)
{
  const double half = 0.5 * increment.dtheta;
  const double phi = theta + half;
  const double s = sinc(half);
  const double dsinc = sincDerivative(half);  // d sinc(u)/du, u = Δθ/2 → d/dΔθ 에 1/2 곱
  const double c_phi = std::cos(phi);
  const double s_phi = std::sin(phi);
  const double ds = increment.ds;

  StepJacobians j;
  j.F_p << 1.0, 0.0, -ds * s * s_phi,
    0.0, 1.0, ds * s * c_phi,
    0.0, 0.0, 1.0;

  // G = ∂(x', y', θ')/∂(Δs, Δθ)
  Matrix32 g;
  g << s * c_phi, ds * (0.5 * dsinc * c_phi - 0.5 * s * s_phi),
    s * s_phi, ds * (0.5 * dsinc * s_phi + 0.5 * s * c_phi),
    0.0, 1.0;
  // M = ∂(Δs, Δθ)/∂(Δs_R, Δs_L)
  Eigen::Matrix2d m;
  m << 0.5, 0.5,
    1.0 / separation, -1.0 / separation;
  j.F_u = g * m;
  return j;
}

double stepDisplacementVariance(const WheelNoiseParams & noise, double ds)
{
  const double sigma = noise.slip_noise_stddev;
  return (sigma * sigma * noise.slip_reference_distance + noise.slip_distance_coeff) *
         std::abs(ds);
}

double quantizationVariance(double tick_distance)
{
  // 반올림 오차 q ~ U(−δ/2, δ/2): Var q = δ²/12. 구간 변위 오차 = q_end − q_start → δ²/6.
  return tick_distance * tick_distance / 6.0;
}

TwistCovariance computeTwistCovariance(const TwistCovarianceInput & in)
{
  TwistCovariance out;
  const double t2 = in.dt * in.dt;
  const double b = in.separation;
  const double sum = in.var_ds_right + in.var_ds_left;
  const double diff = in.var_ds_right - in.var_ds_left;
  out.var_v = sum / (4.0 * t2);
  out.var_w = sum / (b * b * t2);
  out.cov_vw = diff / (2.0 * b * t2);

  // 파라미터 불확실성: v = (b/2)(ψ_R φ̇_R + ψ_L φ̇_L), ω = ψ_R φ̇_R − ψ_L φ̇_L
  const Eigen::Vector2d h_v(0.5 * b * in.phidot_right, 0.5 * b * in.phidot_left);
  const Eigen::Vector2d h_w(in.phidot_right, -in.phidot_left);
  out.var_v += h_v.dot(in.sigma_psi * h_v);
  out.var_w += h_w.dot(in.sigma_psi * h_w);
  out.cov_vw += h_v.dot(in.sigma_psi * h_w);

  const double skid = in.lateral_skid_coeff * in.v * in.w;
  out.var_vy = in.lateral_stddev * in.lateral_stddev + skid * skid;
  return out;
}

OdometryCovariance::OdometryCovariance(double separation)
: separation_(separation),
  sigma_pp_(Eigen::Matrix3d::Zero()),
  sigma_ppsi_(Matrix32::Zero()),
  sigma_psi_(Eigen::Matrix2d::Zero())
{
}

void OdometryCovariance::reset()
{
  sigma_pp_.setZero();
  sigma_ppsi_.setZero();
}

void OdometryCovariance::setParameterCovariance(const Eigen::Matrix2d & sigma_psi)
{
  sigma_psi_ = sigma_psi;
}

void OdometryCovariance::applyParameterUpdate(
  const Eigen::Matrix2d & i_minus_k_ht, const Eigen::Matrix2d & sigma_psi)
{
  sigma_ppsi_ = sigma_ppsi_ * i_minus_k_ht.transpose();
  sigma_psi_ = sigma_psi;
}

void OdometryCovariance::propagate(
  double theta, const BodyIncrement & increment, double dphi_left, double dphi_right,
  double var_ds_left, double var_ds_right)
{
  const StepJacobians j = exactArcJacobians(theta, increment, separation_);
  // u = (Δs_R, Δs_L) = b̄ (ψ_R Δφ_R, ψ_L Δφ_L) → ∂u/∂ψ = diag(b̄ Δφ_R, b̄ Δφ_L)
  Eigen::Matrix2d du_dpsi = Eigen::Matrix2d::Zero();
  du_dpsi(0, 0) = separation_ * dphi_right;
  du_dpsi(1, 1) = separation_ * dphi_left;
  const Matrix32 f_psi = j.F_u * du_dpsi;

  Eigen::Matrix2d sigma_u = Eigen::Matrix2d::Zero();
  sigma_u(0, 0) = var_ds_right;
  sigma_u(1, 1) = var_ds_left;

  const Eigen::Matrix3d cross = j.F_p * sigma_ppsi_ * f_psi.transpose();
  Eigen::Matrix3d next = j.F_p * sigma_pp_ * j.F_p.transpose() + cross + cross.transpose() +
    f_psi * sigma_psi_ * f_psi.transpose() + j.F_u * sigma_u * j.F_u.transpose();
  // 반올림 누적으로 인한 비대칭 제거
  sigma_pp_ = 0.5 * (next + next.transpose());
  sigma_ppsi_ = j.F_p * sigma_ppsi_ + f_psi * sigma_psi_;
}

Eigen::Matrix3d OdometryCovariance::publishedPoseCovariance(
  double theta, double quant_var_left, double quant_var_right) const
{
  Matrix32 f0;
  const double c = 0.5 * std::cos(theta);
  const double s = 0.5 * std::sin(theta);
  f0 << c, c,
    s, s,
    1.0 / separation_, -1.0 / separation_;
  Eigen::Matrix2d q = Eigen::Matrix2d::Zero();
  q(0, 0) = quant_var_right;
  q(1, 1) = quant_var_left;
  return sigma_pp_ + f0 * q * f0.transpose();
}

}  // namespace amr_localization
