// DockingController 단위 시험: 기하, 추적 예측, 수렴(두 제어 법칙), 속도 상한, 정지 대역, 재시도
// 계수.
#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "amr_behavior/docking/docking_controller.hpp"

using amr_behavior::docking::Command;
using amr_behavior::docking::computeErrors;
using amr_behavior::docking::ControlLaw;
using amr_behavior::docking::DockErrors;
using amr_behavior::docking::DockingController;
using amr_behavior::docking::MarkerObservation;
using amr_behavior::docking::MarkerTracker;
using amr_behavior::docking::Params;
using amr_behavior::docking::Phase;

namespace
{

constexpr double kDeg = M_PI / 180.0;

/// 도크 프레임 D 에서의 로봇 자세 (x_r, y_r, ψ). 도킹 완료 = (0, 0, 0).
struct Pose2
{
  double x;
  double y;
  double yaw;
};

/// 로봇 자세 → base_link 기준 마커 관측 (마커는 D 의 (standoff, 0), 법선 −x).
MarkerObservation observe(const Pose2 & r, double standoff)
{
  const double dx = standoff - r.x;
  const double dy = 0.0 - r.y;
  const double c = std::cos(r.yaw);
  const double s = std::sin(r.yaw);
  MarkerObservation m;
  m.x = c * dx + s * dy;
  m.y = -s * dx + c * dy;
  m.normal_yaw = std::atan2(std::sin(M_PI - r.yaw), std::cos(M_PI - r.yaw));
  return m;
}

void integrate(Pose2 & r, const Command & cmd, double dt)
{
  const double mid = r.yaw + 0.5 * cmd.angular * dt;
  r.x += cmd.linear * dt * std::cos(mid);
  r.y += cmd.linear * dt * std::sin(mid);
  r.yaw += cmd.angular * dt;
}

struct SimResult
{
  bool succeeded{false};
  Phase phase{Phase::kIdle};
  int attempts{0};
  double true_position_error{0.0};
  double true_angle_error{0.0};
  double max_linear{0.0};
  double max_angular{0.0};
  double duration{0.0};
  std::vector<std::string> phases;
};

/// 폐루프 모사: 20 Hz 제어, 30 Hz 검출(노이즈 σ), 선택적 관측 끊김 구간.
SimResult simulate(
  const Params & p, Pose2 start, int attempts, double noise_pos = 0.0, double noise_yaw = 0.0,
  double blackout_from = -1.0, double blackout_to = -1.0, double max_time = 120.0)
{
  DockingController ctl(p);
  std::mt19937 rng(7);
  std::normal_distribution<double> np(0.0, noise_pos);
  std::normal_distribution<double> ny(0.0, noise_yaw);
  Pose2 r = start;
  const double dt = 0.05;
  double next_obs = 0.0;
  ctl.start(0.0, attempts);
  SimResult out;
  double t = 0.0;
  for (; t < max_time && !ctl.finished(); t += dt) {
    std::optional<MarkerObservation> obs;
    const bool dark = t >= blackout_from && t < blackout_to;
    if (t + 1e-9 >= next_obs) {
      next_obs += 1.0 / 30.0;
      if (!dark) {
        MarkerObservation m = observe(r, p.standoff);
        if (noise_pos > 0.0) {
          m.x += np(rng);
          m.y += np(rng);
          m.normal_yaw += ny(rng);
        }
        obs = m;
      }
    }
    const Command cmd = ctl.update(t, obs);
    out.max_linear = std::max(out.max_linear, std::fabs(cmd.linear));
    out.max_angular = std::max(out.max_angular, std::fabs(cmd.angular));
    const std::string name = amr_behavior::docking::phaseName(ctl.phase());
    if (out.phases.empty() || out.phases.back() != name) {
      out.phases.push_back(name);
    }
    integrate(r, cmd, dt);
  }
  out.succeeded = ctl.succeeded();
  out.phase = ctl.phase();
  out.attempts = ctl.attempt();
  out.true_position_error = std::hypot(r.x, r.y);
  out.true_angle_error = std::fabs(std::atan2(std::sin(r.yaw), std::cos(r.yaw)));
  out.duration = t;
  return out;
}

}  // namespace

TEST(DockGeometry, ErrorsAtDockedPoseAreZero)
{
  const MarkerObservation m{0.65, 0.0, M_PI};
  const DockErrors e = computeErrors(m, 0.65);
  EXPECT_NEAR(e.longitudinal, 0.0, 1e-12);
  EXPECT_NEAR(e.lateral, 0.0, 1e-12);
  EXPECT_NEAR(e.heading, 0.0, 1e-12);
  EXPECT_NEAR(e.position(), 0.0, 1e-12);
  EXPECT_TRUE(e.valid());
  EXPECT_FALSE(DockErrors{}.valid());
}

TEST(DockGeometry, ErrorsMatchRobotPoseInDockFrame)
{
  const std::vector<Pose2> poses = {
    {-0.85, 0.0, 0.0}, {-0.5, 0.12, 0.0}, {-0.3, -0.07, 10 * kDeg}, {0.02, 0.01, -3 * kDeg}};
  for (const auto & r : poses) {
    const DockErrors e = computeErrors(observe(r, 0.65), 0.65);
    EXPECT_NEAR(e.longitudinal, -r.x, 1e-9);
    EXPECT_NEAR(e.lateral, r.y, 1e-9);
    EXPECT_NEAR(e.heading, r.yaw, 1e-9);
  }
}

TEST(MarkerTracker, PredictMatchesMotion)
{
  const Pose2 start{-0.8, 0.1, 5 * kDeg};
  Pose2 moved = start;
  MarkerTracker tracker;
  EXPECT_TRUE(std::isinf(tracker.age(1.0)));
  tracker.predict(0.1, 0.1, 0.05);   // 추정 없음 → 무시
  EXPECT_FALSE(tracker.hasEstimate());
  tracker.correct(observe(start, 0.65), 0.0, 1.0);
  for (int i = 0; i < 20; ++i) {
    const Command cmd{0.1, 0.2};
    tracker.predict(cmd.linear, cmd.angular, 0.05);
    integrate(moved, cmd, 0.05);
  }
  const MarkerObservation truth = observe(moved, 0.65);
  EXPECT_NEAR(tracker.estimate().x, truth.x, 1e-3);
  EXPECT_NEAR(tracker.estimate().y, truth.y, 1e-3);
  EXPECT_NEAR(tracker.estimate().normal_yaw, truth.normal_yaw, 1e-3);
  EXPECT_NEAR(tracker.age(1.0), 1.0, 1e-12);
}

TEST(MarkerTracker, LowPassBlendsObservations)
{
  MarkerTracker tracker;
  tracker.correct(MarkerObservation{1.0, 0.0, M_PI}, 0.0, 0.5);
  tracker.correct(MarkerObservation{2.0, 1.0, -M_PI + 0.2}, 0.1, 0.5);
  EXPECT_NEAR(tracker.estimate().x, 1.5, 1e-12);
  EXPECT_NEAR(tracker.estimate().y, 0.5, 1e-12);
  // 각도는 ±π 경계를 넘어 짧은 쪽으로 섞는다
  EXPECT_NEAR(std::fabs(tracker.estimate().normal_yaw), M_PI - 0.1, 1e-9);
}

TEST(ControlLaw, ParseNames)
{
  EXPECT_EQ(amr_behavior::docking::parseControlLaw("graceful"), ControlLaw::kGraceful);
  EXPECT_EQ(amr_behavior::docking::parseControlLaw("proportional"), ControlLaw::kProportional);
  EXPECT_FALSE(amr_behavior::docking::parseControlLaw("pid").has_value());
  EXPECT_STREQ(amr_behavior::docking::phaseName(Phase::kIdle), "idle");
  EXPECT_STREQ(amr_behavior::docking::phaseName(Phase::kBackup), "backup");
}

TEST(ControlLaw, ProportionalRespectsCapsAndDeadband)
{
  Params p;
  p.law = ControlLaw::kProportional;
  DockingController ctl(p);
  DockErrors e;
  e.longitudinal = 2.0;
  e.lateral = 0.0;
  e.heading = 0.0;
  Command c = ctl.proportional(e, p.max_linear_speed);
  EXPECT_NEAR(c.linear, p.max_linear_speed, 1e-12);   // 상한 포화
  EXPECT_DOUBLE_EQ(c.angular, 0.0);
  e.longitudinal = 0.002;   // 정지 대역 안
  c = ctl.proportional(e, p.max_linear_speed);
  EXPECT_DOUBLE_EQ(c.linear, 0.0);
  e.longitudinal = 0.5;
  e.lateral = 0.3;
  e.heading = 1.5;   // 큰 방위 오차 → ω 포화, 전진 억제 (cos 계수)
  c = ctl.proportional(e, p.max_linear_speed);
  EXPECT_NEAR(std::fabs(c.angular), p.max_angular_speed, 1e-12);
  EXPECT_LT(c.linear, 0.05);
}

TEST(ControlLaw, GracefulSteersTowardLine)
{
  Params p;
  DockingController ctl(p);
  DockErrors e;
  e.longitudinal = 0.8;
  e.lateral = 0.1;   // 로봇이 +y 쪽 → 오른쪽(−ω)으로 돌아야 한다
  e.heading = 0.0;
  const Command c = ctl.graceful(e, p.max_linear_speed);
  EXPECT_GT(c.linear, 0.0);
  EXPECT_LE(c.linear, p.max_linear_speed + 1e-12);
  EXPECT_LT(c.angular, 0.0);
  EXPECT_LE(std::fabs(c.angular), p.max_angular_speed + 1e-12);
  e.longitudinal = 0.0;
  e.lateral = 0.0;
  const Command z = ctl.graceful(e, p.max_linear_speed);
  EXPECT_DOUBLE_EQ(z.linear, 0.0);
  EXPECT_DOUBLE_EQ(z.angular, 0.0);
}

/// staging 오프셋 격자(횡 ±0.15 m, 방위 ±10°, 종 0.85 m)에서 두 법칙 모두 2 cm / 1° 로 수렴.
class ConvergenceTest : public ::testing::TestWithParam<ControlLaw> {};

TEST_P(ConvergenceTest, ConvergesFromStagingOffsets)
{
  Params p;
  p.law = GetParam();
  for (double y : {-0.15, 0.0, 0.15}) {
    for (double yaw : {-10.0, 0.0, 10.0}) {
      const SimResult r = simulate(p, Pose2{-0.85, y, yaw * kDeg}, 1);
      EXPECT_TRUE(r.succeeded) << "y=" << y << " yaw=" << yaw;
      EXPECT_EQ(r.attempts, 1);
      EXPECT_LE(r.true_position_error, p.position_tolerance) << "y=" << y << " yaw=" << yaw;
      EXPECT_LE(r.true_angle_error, p.angle_tolerance) << "y=" << y << " yaw=" << yaw;
      EXPECT_LE(r.max_linear, p.max_linear_speed + 1e-9);
      EXPECT_LE(r.max_angular, p.max_angular_speed + 1e-9);
      EXPECT_LT(r.duration, p.attempt_timeout);
    }
  }
}

TEST_P(ConvergenceTest, ConvergesWithMeasurementNoise)
{
  Params p;
  p.law = GetParam();
  // 검출 노이즈 σ = 2 mm / 0.3° (브리프 §3.2 pre-dock 수준)
  const SimResult r = simulate(p, Pose2{-0.85, 0.1, 5 * kDeg}, 1, 0.002, 0.3 * kDeg);
  EXPECT_TRUE(r.succeeded);
  EXPECT_LE(r.true_position_error, p.position_tolerance);
  EXPECT_LE(r.true_angle_error, p.angle_tolerance);
}

/// 판정 여유: 허용오차 경계(2 cm)에서 멈추지 않고 도달 대역(|e_x| ≤ stop_distance) 안까지 들어간다.
TEST_P(ConvergenceTest, StopsInsideArrivalBandNotAtToleranceEdge)
{
  Params p;
  p.law = GetParam();
  for (double y : {-0.1, 0.0, 0.1}) {
    const SimResult r = simulate(p, Pose2{-0.85, y, 0.0}, 1);
    ASSERT_TRUE(r.succeeded) << "y=" << y;
    EXPECT_LE(r.true_position_error, 0.5 * p.position_tolerance) << "y=" << y;
    EXPECT_LE(r.true_angle_error, p.angle_tolerance) << "y=" << y;
  }
}

INSTANTIATE_TEST_SUITE_P(
  Laws, ConvergenceTest,
  ::testing::Values(ControlLaw::kGraceful, ControlLaw::kProportional));

TEST(DockingController, LargeHeadingErrorAlignsFirst)
{
  Params p;
  p.law = ControlLaw::kProportional;
  const SimResult r = simulate(p, Pose2{-0.85, 0.0, 30 * kDeg}, 1);
  ASSERT_GE(r.phases.size(), 4U);
  EXPECT_EQ(r.phases[0], "align");   // 첫 관측에서 search → align (진행 방향 오차 30° > 20°)
  EXPECT_EQ(r.phases[1], "approach");
  EXPECT_EQ(r.phases.back(), "succeeded");
  EXPECT_TRUE(r.succeeded);
}

TEST(DockingController, MarkerLossTriggersBackupAndRetry)
{
  Params p;
  // 1회차 접근 중 3 s 동안 관측 끊김 → marker_lost → 후진 → 2회차 성공
  const SimResult r = simulate(p, Pose2{-0.85, 0.05, 0.0}, 3, 0.0, 0.0, 1.0, 4.0);
  EXPECT_TRUE(r.succeeded);
  EXPECT_EQ(r.attempts, 2);
  bool saw_backup = false;
  for (const auto & ph : r.phases) {
    saw_backup = saw_backup || ph == "backup";
  }
  EXPECT_TRUE(saw_backup);
}

TEST(DockingController, FailsAfterMaxAttemptsWithoutMarker)
{
  Params p;
  p.search_timeout = 1.0;
  DockingController ctl(p);
  ctl.start(0.0, 3);
  double t = 0.0;
  int backups = 0;
  Phase prev = ctl.phase();
  for (; t < 60.0 && !ctl.finished(); t += 0.05) {
    const Command c = ctl.update(t, std::nullopt);
    if (ctl.phase() == Phase::kSearch) {
      EXPECT_DOUBLE_EQ(c.linear, 0.0);   // 탐색은 제자리 회전
      EXPECT_LE(std::fabs(c.angular), p.search_angular_speed + 1e-12);
    }
    if (ctl.phase() == Phase::kBackup && prev != Phase::kBackup) {
      ++backups;
    }
    prev = ctl.phase();
  }
  EXPECT_TRUE(ctl.finished());
  EXPECT_FALSE(ctl.succeeded());
  EXPECT_EQ(ctl.attempt(), 3);
  EXPECT_EQ(backups, 2);   // 시도 사이에만 후진
  EXPECT_EQ(ctl.failureReason(), "search_timeout");
  EXPECT_TRUE(std::isnan(ctl.distanceRemaining()));
  // 끝난 뒤에는 정지 지령만
  const Command c = ctl.update(t + 1.0, MarkerObservation{0.65, 0.0, M_PI});
  EXPECT_DOUBLE_EQ(c.linear, 0.0);
  EXPECT_DOUBLE_EQ(c.angular, 0.0);
}

TEST(DockingController, AttemptTimeoutAndOvershoot)
{
  Params p;
  p.attempt_timeout = 0.5;
  DockingController ctl(p);
  ctl.start(0.0, 1);
  const MarkerObservation far{2.5, 0.0, M_PI};
  for (double t = 0.0; t < 1.0 && !ctl.finished(); t += 0.05) {
    ctl.update(t, far);
  }
  EXPECT_FALSE(ctl.succeeded());
  EXPECT_EQ(ctl.failureReason(), "attempt_timeout");

  DockingController over(Params{});
  over.start(0.0, 1);
  // 마커가 standoff 보다 10 cm 가까움 → e_x = −0.10 < −max_overshoot
  over.update(0.0, MarkerObservation{0.55, 0.0, M_PI});
  EXPECT_TRUE(over.finished());
  EXPECT_EQ(over.failureReason(), "overshoot");
}

TEST(DockingController, LateralResidualFailsAttempt)
{
  Params p;
  p.law = ControlLaw::kProportional;
  DockingController ctl(p);
  ctl.start(0.0, 1);
  // 종방향은 도달, 횡오차 5 cm 고정 (관측이 움직이지 않는 상황)
  const MarkerObservation m{0.65, -0.05, M_PI};
  double t = 0.0;
  for (; t < 10.0 && !ctl.finished(); t += 0.05) {
    ctl.update(t, m);
  }
  EXPECT_TRUE(ctl.finished());
  EXPECT_EQ(ctl.failureReason(), "lateral");
}

TEST(DockingController, SettleRequiresConsecutiveFrames)
{
  Params p;
  DockingController ctl(p);
  ctl.start(0.0, 1);
  const MarkerObservation docked{0.65, 0.0, M_PI};
  double t = 0.0;
  for (int i = 0; i < p.settle_frames - 1; ++i, t += 0.05) {
    const Command c = ctl.update(t, docked);
    EXPECT_DOUBLE_EQ(c.linear, 0.0);
    EXPECT_DOUBLE_EQ(c.angular, 0.0);
  }
  EXPECT_FALSE(ctl.finished());
  EXPECT_EQ(ctl.phase(), Phase::kFinal);
  ctl.update(t, docked);
  EXPECT_TRUE(ctl.succeeded());
  EXPECT_NEAR(ctl.errors().position(), 0.0, 1e-9);
}

TEST(DockingController, KeepsApproachingInsideToleranceUntilArrived)
{
  Params p;
  DockingController ctl(p);
  ctl.start(0.0, 1);
  // e_x = 15 mm: 허용오차(2 cm) 안이지만 도달 대역(8 mm) 밖 → 판정하지 않고 계속 전진
  double t = 0.0;
  Command c = ctl.update(t, MarkerObservation{p.standoff + 0.015, 0.0, M_PI});
  EXPECT_GT(c.linear, 0.0);
  EXPECT_EQ(ctl.phase(), Phase::kFinal);
  // 도달(e_x = 5 mm) → 정지하고 판정 시작. 추정이 12 mm 로 흔들려도
  // 해제 경계(8 + 2·4 = 16 mm) 안이면 전진을 재개하지 않고 판정을 이어 간다.
  for (int i = 0; i < p.settle_frames; ++i) {
    t += 0.05;
    const double ex = (i % 2 == 0) ? 0.005 : 0.012;
    c = ctl.update(t, MarkerObservation{p.standoff + ex, 0.0, M_PI});
    EXPECT_DOUBLE_EQ(c.linear, 0.0) << i;
  }
  EXPECT_TRUE(ctl.succeeded());
  // 해제 경계 밖(20 mm)으로 밀리면 도달이 풀려 다시 전진한다
  DockingController ctl2(p);
  ctl2.start(0.0, 1);
  ctl2.update(0.0, MarkerObservation{p.standoff + 0.005, 0.0, M_PI});
  c = ctl2.update(0.05, MarkerObservation{p.standoff + 0.020, 0.0, M_PI});
  EXPECT_GT(c.linear, 0.0);
  EXPECT_FALSE(ctl2.finished());
}

TEST(DockingController, CancelStops)
{
  DockingController ctl(Params{});
  EXPECT_EQ(ctl.phase(), Phase::kIdle);
  const Command idle = ctl.update(0.0, std::nullopt);
  EXPECT_DOUBLE_EQ(idle.angular, 0.0);
  ctl.start(0.0, 0);   // 0 → 1 회
  EXPECT_EQ(ctl.maxAttempts(), 1);
  ctl.update(0.05, MarkerObservation{1.5, 0.0, M_PI});
  ctl.cancel();
  EXPECT_TRUE(ctl.finished());
  EXPECT_FALSE(ctl.succeeded());
  EXPECT_EQ(ctl.failureReason(), "canceled");
}

TEST(DockingController, OvershootWithinLimitBacksUpSlowly)
{
  Params p;
  DockingController ctl(p);
  ctl.start(0.0, 1);
  // e_x = −0.03 (지나침, 허용 −0.05 안) → 느린 후진
  const Command c = ctl.update(0.0, MarkerObservation{0.62, 0.0, M_PI});
  EXPECT_LT(c.linear, 0.0);
  EXPECT_GE(c.linear, -p.final_linear_speed - 1e-12);
}
