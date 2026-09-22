// 차동 구동 순기구학 구현. 수식 유도는 docs/algorithms/kinematics.md §1.

#include "amr_localization/diff_drive_kinematics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace amr_localization
{

namespace
{
// sinc 의 테일러 전개로 전환하는 경계. |x| < 1e-4 에서 x⁴/120 항은 1e-18 이하라 배정밀도 한계 아래.
constexpr double kSeriesThreshold = 1e-4;
}  // namespace

bool DiffDriveGeometry::valid() const
{
  auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};
  return positive(left_wheel_radius) && positive(right_wheel_radius) &&
         positive(wheel_separation);
}

IntegrationMethod parseIntegrationMethod(const std::string & name)
{
  if (name == "euler") {
    return IntegrationMethod::kEuler;
  }
  if (name == "midpoint") {
    return IntegrationMethod::kMidpoint;
  }
  if (name == "exact_arc") {
    return IntegrationMethod::kExactArc;
  }
  throw std::invalid_argument("unknown integration method: " + name);
}

std::string toString(IntegrationMethod method)
{
  switch (method) {
    case IntegrationMethod::kEuler:
      return "euler";
    case IntegrationMethod::kMidpoint:
      return "midpoint";
    case IntegrationMethod::kExactArc:
    default:
      return "exact_arc";
  }
}

BodyIncrement wheelDisplacementsToIncrement(
  double ds_left, double ds_right, double wheel_separation)
{
  BodyIncrement inc;
  inc.ds = 0.5 * (ds_right + ds_left);
  inc.dtheta = (ds_right - ds_left) / wheel_separation;
  return inc;
}

BodyIncrement wheelAnglesToIncrement(
  double dphi_left, double dphi_right, const DiffDriveGeometry & geometry)
{
  return wheelDisplacementsToIncrement(
    geometry.left_wheel_radius * dphi_left,
    geometry.right_wheel_radius * dphi_right,
    geometry.wheel_separation);
}

BodyVelocity forwardKinematics(
  double omega_left, double omega_right, const DiffDriveGeometry & geometry)
{
  const double v_left = geometry.left_wheel_radius * omega_left;
  const double v_right = geometry.right_wheel_radius * omega_right;
  BodyVelocity vel;
  vel.v = 0.5 * (v_right + v_left);
  vel.w = (v_right - v_left) / geometry.wheel_separation;
  return vel;
}

void inverseKinematics(
  const BodyVelocity & velocity, const DiffDriveGeometry & geometry,
  double & omega_left, double & omega_right)
{
  const double half_b = 0.5 * geometry.wheel_separation;
  omega_right = (velocity.v + velocity.w * half_b) / geometry.right_wheel_radius;
  omega_left = (velocity.v - velocity.w * half_b) / geometry.left_wheel_radius;
}

double sinc(double x)
{
  if (std::abs(x) < kSeriesThreshold) {
    const double x2 = x * x;
    return 1.0 - x2 / 6.0 + x2 * x2 / 120.0;
  }
  return std::sin(x) / x;
}

double sincDerivative(double x)
{
  if (std::abs(x) < kSeriesThreshold) {
    return -x / 3.0 + x * x * x / 30.0;
  }
  return (x * std::cos(x) - std::sin(x)) / (x * x);
}

double normalizeAngle(double angle)
{
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a <= 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

Pose2D integrate(const Pose2D & pose, const BodyIncrement & increment, IntegrationMethod method)
{
  switch (method) {
    case IntegrationMethod::kEuler:
      return integrateEuler(pose, increment);
    case IntegrationMethod::kMidpoint:
      return integrateMidpoint(pose, increment);
    case IntegrationMethod::kExactArc:
    default:
      return integrateExactArc(pose, increment);
  }
}

Pose2D integrateEuler(const Pose2D & pose, const BodyIncrement & increment)
{
  Pose2D out;
  out.x = pose.x + increment.ds * std::cos(pose.theta);
  out.y = pose.y + increment.ds * std::sin(pose.theta);
  out.theta = pose.theta + increment.dtheta;
  return out;
}

Pose2D integrateMidpoint(const Pose2D & pose, const BodyIncrement & increment)
{
  const double phi = pose.theta + 0.5 * increment.dtheta;
  Pose2D out;
  out.x = pose.x + increment.ds * std::cos(phi);
  out.y = pose.y + increment.ds * std::sin(phi);
  out.theta = pose.theta + increment.dtheta;
  return out;
}

Pose2D integrateExactArc(const Pose2D & pose, const BodyIncrement & increment)
{
  // 등속 원호: 반지름 R = Δs/Δθ, 현 길이 2R sin(Δθ/2) = Δs sinc(Δθ/2), 현 방향 θ + Δθ/2.
  const double phi = pose.theta + 0.5 * increment.dtheta;
  const double chord = increment.ds * sinc(0.5 * increment.dtheta);
  Pose2D out;
  out.x = pose.x + chord * std::cos(phi);
  out.y = pose.y + chord * std::sin(phi);
  out.theta = pose.theta + increment.dtheta;
  return out;
}

}  // namespace amr_localization
