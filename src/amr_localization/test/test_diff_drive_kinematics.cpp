// 순기구학·적분 단위 테스트: 해석해(직진/원호/제자리 회전)와 원호 적분의 정확성.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "amr_localization/diff_drive_kinematics.hpp"

using amr_localization::BodyIncrement;
using amr_localization::BodyVelocity;
using amr_localization::DiffDriveGeometry;
using amr_localization::IntegrationMethod;
using amr_localization::Pose2D;

namespace
{
const DiffDriveGeometry kGeom{0.0825, 0.0825, 0.36};

/// 등속 (v, ω) 로 T 초 주행한 해석해 (시작 자세 p0).
Pose2D analyticPose(const Pose2D & p0, double v, double w, double t)
{
  Pose2D p;
  if (std::abs(w) < 1e-12) {
    p.x = p0.x + v * t * std::cos(p0.theta);
    p.y = p0.y + v * t * std::sin(p0.theta);
    p.theta = p0.theta;
    return p;
  }
  const double r = v / w;
  p.theta = p0.theta + w * t;
  p.x = p0.x + r * (std::sin(p.theta) - std::sin(p0.theta));
  p.y = p0.y - r * (std::cos(p.theta) - std::cos(p0.theta));
  return p;
}

/// 등속 운동을 dt 간격으로 적분.
Pose2D simulate(double v, double w, double t_total, double dt, IntegrationMethod m)
{
  double wl = 0.0;
  double wr = 0.0;
  amr_localization::inverseKinematics(BodyVelocity{v, w}, kGeom, wl, wr);
  Pose2D p;
  // 스텝 수를 반올림하고 실제 간격을 t_total / steps 로 맞춰 총 시간이 정확히 t_total 이 되게 한다
  const int steps = std::max(1, static_cast<int>(std::lround(t_total / dt)));
  const double h = t_total / steps;
  for (int i = 0; i < steps; ++i) {
    const BodyIncrement inc = amr_localization::wheelAnglesToIncrement(wl * h, wr * h, kGeom);
    p = amr_localization::integrate(p, inc, m);
  }
  return p;
}

double positionError(const Pose2D & a, const Pose2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
}  // namespace

TEST(ForwardKinematics, StraightLine)
{
  // 두 바퀴 같은 각속도 → ω = 0, v = r ω_wheel
  const BodyVelocity vel = amr_localization::forwardKinematics(10.0, 10.0, kGeom);
  EXPECT_NEAR(vel.v, 0.825, 1e-12);
  EXPECT_NEAR(vel.w, 0.0, 1e-12);
}

TEST(ForwardKinematics, PureRotation)
{
  // 반대 방향 같은 크기 → v = 0, ω = 2 r ω_wheel / b
  const BodyVelocity vel = amr_localization::forwardKinematics(-5.0, 5.0, kGeom);
  EXPECT_NEAR(vel.v, 0.0, 1e-12);
  EXPECT_NEAR(vel.w, 2.0 * 0.0825 * 5.0 / 0.36, 1e-12);
}

TEST(ForwardKinematics, ArcMatchesFormula)
{
  // v = r(ω_R + ω_L)/2, ω = r(ω_R − ω_L)/b
  const double wl = 4.0;
  const double wr = 7.0;
  const BodyVelocity vel = amr_localization::forwardKinematics(wl, wr, kGeom);
  EXPECT_NEAR(vel.v, 0.0825 * (wr + wl) / 2.0, 1e-12);
  EXPECT_NEAR(vel.w, 0.0825 * (wr - wl) / 0.36, 1e-12);
}

TEST(ForwardKinematics, UnequalRadii)
{
  const DiffDriveGeometry g{0.08, 0.085, 0.36};
  const BodyVelocity vel = amr_localization::forwardKinematics(10.0, 10.0, g);
  EXPECT_NEAR(vel.v, 0.5 * (0.85 + 0.8), 1e-12);
  EXPECT_NEAR(vel.w, (0.85 - 0.8) / 0.36, 1e-12);
}

TEST(ForwardKinematics, InverseRoundTrip)
{
  for (double v : {-0.5, 0.0, 0.7, 2.0}) {
    for (double w : {-1.5, 0.0, 0.3, 1.5}) {
      double wl = 0.0;
      double wr = 0.0;
      amr_localization::inverseKinematics(BodyVelocity{v, w}, kGeom, wl, wr);
      const BodyVelocity back = amr_localization::forwardKinematics(wl, wr, kGeom);
      EXPECT_NEAR(back.v, v, 1e-12);
      EXPECT_NEAR(back.w, w, 1e-12);
    }
  }
}

TEST(ForwardKinematics, IncrementFromWheelAngles)
{
  const BodyIncrement inc = amr_localization::wheelAnglesToIncrement(0.2, 0.4, kGeom);
  EXPECT_NEAR(inc.ds, 0.0825 * 0.3, 1e-12);
  EXPECT_NEAR(inc.dtheta, 0.0825 * 0.2 / 0.36, 1e-12);
}

TEST(Geometry, Validity)
{
  EXPECT_TRUE(kGeom.valid());
  EXPECT_FALSE((DiffDriveGeometry{0.0, 0.08, 0.36}).valid());
  EXPECT_FALSE((DiffDriveGeometry{0.08, 0.08, -1.0}).valid());
  EXPECT_FALSE((DiffDriveGeometry{0.08, NAN, 0.36}).valid());
}

TEST(Integration, ParseAndPrint)
{
  EXPECT_EQ(amr_localization::parseIntegrationMethod("euler"), IntegrationMethod::kEuler);
  EXPECT_EQ(amr_localization::parseIntegrationMethod("midpoint"), IntegrationMethod::kMidpoint);
  EXPECT_EQ(amr_localization::parseIntegrationMethod("exact_arc"), IntegrationMethod::kExactArc);
  EXPECT_THROW(amr_localization::parseIntegrationMethod("rk4"), std::invalid_argument);
  EXPECT_EQ(amr_localization::toString(IntegrationMethod::kEuler), "euler");
  EXPECT_EQ(amr_localization::toString(IntegrationMethod::kMidpoint), "midpoint");
  EXPECT_EQ(amr_localization::toString(IntegrationMethod::kExactArc), "exact_arc");
}

TEST(Integration, SincAndDerivative)
{
  EXPECT_DOUBLE_EQ(amr_localization::sinc(0.0), 1.0);
  for (double x : {1e-6, 1e-3, 0.1, 1.0, 3.0}) {
    EXPECT_NEAR(amr_localization::sinc(x), std::sin(x) / x, 1e-14);
    const double h = 1e-6;
    const double numeric =
      (amr_localization::sinc(x + h) - amr_localization::sinc(x - h)) / (2.0 * h);
    EXPECT_NEAR(amr_localization::sincDerivative(x), numeric, 1e-8);
  }
  EXPECT_DOUBLE_EQ(amr_localization::sincDerivative(0.0), 0.0);
}

TEST(Integration, NormalizeAngle)
{
  EXPECT_NEAR(amr_localization::normalizeAngle(3.0 * M_PI), M_PI, 1e-12);
  EXPECT_NEAR(amr_localization::normalizeAngle(-M_PI), M_PI, 1e-12);
  EXPECT_NEAR(amr_localization::normalizeAngle(2.0 * M_PI + 0.1), 0.1, 1e-12);
  EXPECT_NEAR(amr_localization::normalizeAngle(-0.5), -0.5, 1e-12);
}

TEST(Integration, StraightIsExactForAllMethods)
{
  const Pose2D truth = analyticPose(Pose2D{}, 1.0, 0.0, 5.0);
  for (auto m : {IntegrationMethod::kEuler, IntegrationMethod::kMidpoint,
      IntegrationMethod::kExactArc})
  {
    const Pose2D p = simulate(1.0, 0.0, 5.0, 0.02, m);
    EXPECT_NEAR(positionError(p, truth), 0.0, 1e-9);
  }
}

TEST(Integration, PureRotationDoesNotTranslate)
{
  const Pose2D p = simulate(0.0, 1.0, 2.0 * M_PI, 0.02, IntegrationMethod::kExactArc);
  EXPECT_NEAR(p.x, 0.0, 1e-12);
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  EXPECT_NEAR(p.theta, 2.0 * M_PI, 1e-9);
}

TEST(Integration, ExactArcMatchesAnalyticCircle)
{
  // 0.5 m/s, 0.5 rad/s → 반지름 1 m 원을 π/2 만큼 (90° 원호)
  const double t = (M_PI / 2.0) / 0.5;
  const Pose2D truth = analyticPose(Pose2D{}, 0.5, 0.5, t);
  EXPECT_NEAR(truth.x, 1.0, 1e-12);
  EXPECT_NEAR(truth.y, 1.0, 1e-12);
  const Pose2D p = simulate(0.5, 0.5, t, 0.02, IntegrationMethod::kExactArc);
  EXPECT_NEAR(positionError(p, truth), 0.0, 1e-9);
  EXPECT_NEAR(p.theta, truth.theta, 1e-9);
  // 한 스텝으로도 정확하다 (원호 해는 스텝 크기와 무관)
  const Pose2D one = amr_localization::integrateExactArc(
    Pose2D{}, BodyIncrement{0.5 * t, 0.5 * t});
  EXPECT_NEAR(positionError(one, truth), 0.0, 1e-12);
}

TEST(Integration, EulerHasFirstOrderErrorAndArcDoesNot)
{
  // 곡선에서 오일러는 O(dt) 편향, 중점은 O(dt²), 원호는 0
  const double t = 10.0;
  const Pose2D truth = analyticPose(Pose2D{}, 1.0, 1.0, t);
  const double e_euler_20ms = positionError(
    simulate(1.0, 1.0, t, 0.02, IntegrationMethod::kEuler), truth);
  const double e_euler_10ms = positionError(
    simulate(1.0, 1.0, t, 0.01, IntegrationMethod::kEuler), truth);
  const double e_mid_20ms = positionError(
    simulate(1.0, 1.0, t, 0.02, IntegrationMethod::kMidpoint), truth);
  const double e_mid_10ms = positionError(
    simulate(1.0, 1.0, t, 0.01, IntegrationMethod::kMidpoint), truth);
  const double e_arc = positionError(
    simulate(1.0, 1.0, t, 0.02, IntegrationMethod::kExactArc), truth);
  EXPECT_GT(e_euler_20ms, 1e-3);                         // 수 mm 편향
  EXPECT_NEAR(e_euler_20ms / e_euler_10ms, 2.0, 0.1);    // 1차 수렴
  EXPECT_NEAR(e_mid_20ms / e_mid_10ms, 4.0, 0.3);        // 2차 수렴
  EXPECT_LT(e_mid_20ms, e_euler_20ms / 100.0);
  EXPECT_LT(e_arc, 1e-9);
}

TEST(Integration, MidpointStepErrorBound)
{
  // 중점식 단계 오차 = Δs [1 − sinc(Δθ/2)] ≤ Δs Δθ²/24 (docs/algorithms/kinematics.md §1.1)
  for (double ds : {0.001, 0.02, 0.04}) {
    for (double dth : {0.001, 0.01, 0.03, 0.2}) {
      const BodyIncrement inc{ds, dth};
      const Pose2D a = amr_localization::integrateMidpoint(Pose2D{}, inc);
      const Pose2D b = amr_localization::integrateExactArc(Pose2D{}, inc);
      const double err = positionError(a, b);
      EXPECT_LE(err, ds * dth * dth / 24.0 * (1.0 + 1e-9) + 1e-15);
      EXPECT_NEAR(err, ds * (1.0 - amr_localization::sinc(0.5 * dth)), 1e-15);
    }
  }
}

TEST(Integration, DispatchMatchesDirectCalls)
{
  const Pose2D p0{1.0, -2.0, 0.3};
  const BodyIncrement inc{0.05, 0.02};
  const auto e = amr_localization::integrate(p0, inc, IntegrationMethod::kEuler);
  const auto e2 = amr_localization::integrateEuler(p0, inc);
  EXPECT_DOUBLE_EQ(e.x, e2.x);
  const auto m = amr_localization::integrate(p0, inc, IntegrationMethod::kMidpoint);
  const auto m2 = amr_localization::integrateMidpoint(p0, inc);
  EXPECT_DOUBLE_EQ(m.y, m2.y);
  const auto a = amr_localization::integrate(p0, inc, IntegrationMethod::kExactArc);
  const auto a2 = amr_localization::integrateExactArc(p0, inc);
  EXPECT_DOUBLE_EQ(a.x, a2.x);
  EXPECT_DOUBLE_EQ(a.theta, p0.theta + inc.dtheta);
}
