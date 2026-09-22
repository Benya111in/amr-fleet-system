// 휠 오도메트리 코어 단위 테스트: 합성 조인트 각으로 직진 5 m / 90° 원호가 해석해와
// 양자화 오차 안에서 일치하는지, 발행 주기, 메시지 누락 내성, 시간 역행 리셋,
// 공분산 일관성(NEES).

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "amr_localization/wheel_odometry.hpp"

using amr_localization::Pose2D;
using amr_localization::WheelOdometry;
using amr_localization::WheelOdometryOutput;
using amr_localization::WheelOdometryParams;

namespace
{
constexpr double kR = 0.0825;
constexpr double kB = 0.36;

WheelOdometryParams quantOnlyParams()
{
  WheelOdometryParams p;
  p.encoder.slip_noise = false;
  p.noise.slip_noise_stddev = 0.0;
  return p;
}

/// 등속 (v, ω) 의 조인트 각을 rate 로 흘려 넣고 마지막 출력 반환.
struct RunResult
{
  WheelOdometryOutput last;
  int outputs{0};
  double t_last{0.0};  ///< 마지막 출력의 경과 시간 [s]
};

RunResult run(
  WheelOdometry & odom, double v, double w, double duration, double rate, int skip_every = 0)
{
  RunResult res;
  const double wr = (v + 0.5 * w * kB) / kR;
  const double wl = (v - 0.5 * w * kB) / kR;
  const int n = static_cast<int>(std::lround(duration * rate));
  for (int i = 0; i <= n; ++i) {
    if (skip_every > 0 && i % skip_every == 1 && i != n) {
      continue;  // 메시지 누락
    }
    const double t = static_cast<double>(i) / rate;
    WheelOdometryOutput out;
    if (odom.update(100.0 + t, wl * t, wr * t, out)) {
      res.last = out;
      res.t_last = t;
      ++res.outputs;
    }
  }
  return res;
}

double tickDistance()
{
  return kR * 2.0 * M_PI / 4096.0;
}
}  // namespace

TEST(WheelOdometry, RejectsBadGeometry)
{
  WheelOdometryParams p;
  p.geometry.wheel_separation = 0.0;
  EXPECT_THROW(WheelOdometry{p}, std::invalid_argument);
  WheelOdometryParams q;
  q.publish_period = -1.0;
  EXPECT_THROW(WheelOdometry{q}, std::invalid_argument);
}

TEST(WheelOdometry, Straight5mWithinQuantization)
{
  WheelOdometry odom(quantOnlyParams());
  const RunResult r = run(odom, 1.0, 0.0, 5.0, 50.0);
  EXPECT_NEAR(r.last.pose.x, 5.0, 0.5 * tickDistance() + 1e-9);
  EXPECT_NEAR(r.last.pose.y, 0.0, 1e-9);
  EXPECT_NEAR(r.last.pose.theta, 0.0, 2.0 * tickDistance() / kB);
  EXPECT_NEAR(r.last.twist.v, 1.0, 2.0 * tickDistance() / 0.02);
  EXPECT_NEAR(r.last.twist.w, 0.0, 2.0 * tickDistance() / (kB * 0.02));
  EXPECT_EQ(r.outputs, 250);
}

TEST(WheelOdometry, Arc90DegWithinQuantization)
{
  // 반지름 1 m, v = 0.5 m/s, ω = 0.5 rad/s, π s → (1, 1, π/2). 샘플 시각에 맞춘 해석해와 비교
  WheelOdometry odom(quantOnlyParams());
  const RunResult r = run(odom, 0.5, 0.5, M_PI, 100.0);
  const double th = 0.5 * r.t_last;
  EXPECT_NEAR(th, M_PI / 2.0, 0.005);
  const double heading_bound = 2.0 * tickDistance() / kB;
  EXPECT_NEAR(r.last.pose.theta, th, heading_bound);
  EXPECT_NEAR(r.last.pose.x, std::sin(th), 1e-3);
  EXPECT_NEAR(r.last.pose.y, 1.0 - std::cos(th), 1e-3);
  EXPECT_NEAR(r.last.twist.v, 0.5, 0.01);
  EXPECT_NEAR(r.last.twist.w, 0.5, 0.05);
}

TEST(WheelOdometry, PublishScheduling)
{
  WheelOdometryParams p = quantOnlyParams();
  p.publish_period = 0.02;
  WheelOdometry a(p);
  EXPECT_EQ(run(a, 1.0, 0.0, 1.0, 50.0).outputs, 50);   // 50 Hz 입력 → 매번
  WheelOdometry b(p);
  const RunResult rb = run(b, 1.0, 0.0, 1.0, 100.0);    // 100 Hz 입력 → 격번
  EXPECT_EQ(rb.outputs, 50);
  EXPECT_NEAR(rb.last.interval, 0.02, 1e-9);
  p.publish_period = 0.0;
  WheelOdometry c(p);
  EXPECT_EQ(run(c, 1.0, 0.0, 1.0, 100.0).outputs, 100);  // 0 → 입력마다
}

TEST(WheelOdometry, RobustToDroppedMessages)
{
  // 위치(누적 각) 기반이라 누락돼도 이동량이 보존된다
  WheelOdometry odom(quantOnlyParams());
  const RunResult r = run(odom, 1.0, 0.2, 5.0, 50.0, 3);
  WheelOdometry ref(quantOnlyParams());
  const RunResult rr = run(ref, 1.0, 0.2, 5.0, 50.0);
  EXPECT_NEAR(r.last.pose.x, rr.last.pose.x, 1e-3);
  EXPECT_NEAR(r.last.pose.y, rr.last.pose.y, 1e-3);
  EXPECT_NEAR(r.last.pose.theta, rr.last.pose.theta, 1e-3);
}

TEST(WheelOdometry, TimeReversalResets)
{
  WheelOdometry odom(quantOnlyParams());
  run(odom, 1.0, 0.0, 2.0, 50.0);
  EXPECT_GT(odom.pose().x, 1.9);
  WheelOdometryOutput out;
  EXPECT_FALSE(odom.update(50.0, 0.0, 0.0, out));  // 시간 역행 → 새 기준점
  EXPECT_DOUBLE_EQ(odom.pose().x, 0.0);
  EXPECT_FALSE(odom.update(50.02, 0.0, 0.0, out) && out.pose.x != 0.0);
}

TEST(WheelOdometry, ResetPoseKeepsEncoderReference)
{
  WheelOdometry odom(quantOnlyParams());
  run(odom, 1.0, 0.0, 1.0, 50.0);
  odom.resetPose(Pose2D{1.0, 2.0, 0.5});
  EXPECT_DOUBLE_EQ(odom.pose().y, 2.0);
  EXPECT_DOUBLE_EQ(odom.covariance().poseCovariance().norm(), 0.0);
}

TEST(WheelOdometry, TwistCovarianceFollowsSlipModel)
{
  // 직진 1 m/s, 50 Hz: Var v = 2σ_s²Δs²/(4T²) + 양자화 2(δ²/6)/(4T²)
  WheelOdometryParams p;
  p.seed = 11;
  WheelOdometry odom(p);
  const RunResult r = run(odom, 1.0, 0.0, 1.0, 50.0);
  const double ds = 0.02;
  const double q = tickDistance() * tickDistance() / 6.0;
  const double expected_v = (2.0 * 1e-4 * ds * ds + 2.0 * q) / (4.0 * 0.02 * 0.02);
  // 측정 Δs 는 잡음이 섞여 있어 σ_s²Δs² 가 약간 달라진다 → 5 %
  EXPECT_NEAR(r.last.twist_covariance.var_v / expected_v, 1.0, 0.05);
  EXPECT_NEAR(
    r.last.twist_covariance.var_w / (expected_v * 4.0 / (kB * kB)), 1.0, 0.05);
  EXPECT_GT(r.last.pose_covariance(2, 2), 0.0);
}

TEST(WheelOdometry, PoseCovarianceIsConsistent)
{
  // 서로 다른 seed 300 회: 최종 자세 오차의 NEES 평균 ≈ 3 (χ²₃), 공분산이 과신/과소가 아님
  const double duration = 5.0;
  double nees_sum = 0.0;
  const int runs = 300;
  // 해석해: v = 0.8, ω = 0.3
  const double v = 0.8;
  const double w = 0.3;
  const double th = w * duration;
  const Eigen::Vector3d truth(v / w * std::sin(th), v / w * (1.0 - std::cos(th)), th);
  for (int s = 0; s < runs; ++s) {
    WheelOdometryParams p;
    p.seed = 1000 + 2 * static_cast<std::uint64_t>(s);
    WheelOdometry odom(p);
    const RunResult r = run(odom, v, w, duration, 50.0);
    const Eigen::Vector3d est(r.last.pose.x, r.last.pose.y, r.last.pose.theta);
    const Eigen::Vector3d e = est - truth;
    nees_sum += e.dot(r.last.pose_covariance.ldlt().solve(e));
  }
  const double mean_nees = nees_sum / runs;
  EXPECT_GT(mean_nees, 2.5);
  EXPECT_LT(mean_nees, 3.5);
}

TEST(WheelOdometry, ExtensionPointsUpdateGeometry)
{
  WheelOdometry odom(quantOnlyParams());
  odom.setWheelRadii(0.08, 0.085);
  EXPECT_DOUBLE_EQ(odom.params().geometry.left_wheel_radius, 0.08);
  EXPECT_THROW(odom.setWheelRadii(-1.0, 0.08), std::invalid_argument);
  odom.setParameterCovariance(Eigen::Matrix2d::Identity() * 1e-6);
  EXPECT_DOUBLE_EQ(odom.covariance().parameterCovariance()(1, 1), 1e-6);
}
