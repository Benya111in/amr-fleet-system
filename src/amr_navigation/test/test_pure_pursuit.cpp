// Pure Pursuit 와 경로 속도 프로파일(곡률 상한·감속 원뿔·주행 시간 예측) 단위 테스트.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/jerk_limiter.hpp"
#include "amr_navigation/core/pid.hpp"
#include "amr_navigation/core/pure_pursuit.hpp"
#include "amr_navigation/core/speed_profile.hpp"

using amr_navigation::core::Point2D;
using amr_navigation::core::Pose2D;
using amr_navigation::core::Projection;
using amr_navigation::core::PurePursuit;
using amr_navigation::core::PurePursuitConfig;
using amr_navigation::core::PurePursuitMode;
using amr_navigation::core::PurePursuitOutput;
using amr_navigation::core::SpeedProfile;
using amr_navigation::core::SpeedProfileConfig;
using amr_navigation::core::cumulativeLength;
using amr_navigation::core::integrateArc;
using amr_navigation::core::kPi;
using amr_navigation::core::projectOntoPath;
using amr_navigation::core::wrapAngle;

namespace
{
constexpr double kStep = 0.05;

void appendStraight(std::vector<Pose2D> & p, double len)
{
  if (p.empty()) {
    p.push_back({0.0, 0.0, 0.0});
  }
  Pose2D q = p.back();
  const int n = static_cast<int>(std::round(len / kStep));
  for (int i = 1; i <= n; ++i) {
    q.x += kStep * std::cos(q.theta);
    q.y += kStep * std::sin(q.theta);
    p.push_back(q);
  }
}

void appendArc(std::vector<Pose2D> & p, double R, double angle)
{
  const Pose2D o = p.back();
  const int n = static_cast<int>(std::round(std::abs(angle) * R / kStep));
  const double sgn = angle >= 0.0 ? 1.0 : -1.0;
  for (int i = 1; i <= n; ++i) {
    const double t = static_cast<double>(i) * kStep;
    p.push_back(integrateArc(o, 1.0, sgn / R, t));
  }
}

std::vector<Pose2D> straight(double len)
{
  std::vector<Pose2D> p;
  appendStraight(p, len);
  return p;
}

struct TrackStats
{
  double max_cte_straight{0.0};
  double max_cte_curve{0.0};
  double mean_cte_straight{0.0};
  double mean_cte_curve{0.0};
  double time{0.0};
  bool reached{false};
};

// 이상 차동구동(명령 즉시 추종) 폐루프. curve_mask(s): 호길이 s 가 곡선 구간인지.
TrackStats simulate(
  PurePursuit & pp, const std::vector<Pose2D> & path, Pose2D pose, double settle_s,
  const std::vector<std::pair<double, double>> & curves)
{
  pp.setPath(path);
  const auto cum = cumulativeLength(path);
  TrackStats st;
  double v = 0.0;
  const double dt = 0.05;
  int n_s = 0;
  int n_c = 0;
  for (int k = 0; k < 4000; ++k) {
    const PurePursuitOutput out = pp.compute(pose, v);
    if (out.mode == PurePursuitMode::kGoalReached) {
      st.reached = true;
      break;
    }
    v = out.v;
    pose = integrateArc(pose, out.v, out.w, dt);
    st.time += dt;
    const Projection pr = projectOntoPath(path, {pose.x, pose.y}, 0, 0, &cum);
    if (pr.s < settle_s || pr.s > cum.back() - 0.3) {
      continue;
    }
    // 곡선 구간 = 곡선 호길이 ± 0.5 m (천이대 포함, 보수적)
    bool curve = false;
    for (const auto & c : curves) {
      curve = curve || (pr.s >= c.first - 0.5 && pr.s <= c.second + 0.5);
    }
    if (curve) {
      st.max_cte_curve = std::max(st.max_cte_curve, std::abs(pr.cte));
      st.mean_cte_curve += std::abs(pr.cte);
      ++n_c;
    } else {
      st.max_cte_straight = std::max(st.max_cte_straight, std::abs(pr.cte));
      st.mean_cte_straight += std::abs(pr.cte);
      ++n_s;
    }
  }
  st.mean_cte_straight /= std::max(1, n_s);
  st.mean_cte_curve /= std::max(1, n_c);
  return st;
}
}  // namespace

TEST(PurePursuit, CurvatureFormula)
{
  // 원점에서 +x 로 접하는 반지름 R 원 위의 점 → κ = 1/R (양쪽 부호)
  for (double R : {0.5, 1.0, 3.0}) {
    for (double a : {0.2, 0.7, 1.5}) {
      EXPECT_NEAR(
        PurePursuit::curvatureToPoint(R * std::sin(a), R * (1.0 - std::cos(a))), 1.0 / R,
        1e-12);
      EXPECT_NEAR(
        PurePursuit::curvatureToPoint(R * std::sin(a), -R * (1.0 - std::cos(a))), -1.0 / R, 1e-12);
    }
  }
  EXPECT_DOUBLE_EQ(PurePursuit::curvatureToPoint(0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(PurePursuit::curvatureToPoint(2.0, 0.0), 0.0);
}

TEST(PurePursuit, LookaheadClamping)
{
  PurePursuitConfig c;   // k = 0.8, [0.4, 1.8]
  EXPECT_DOUBLE_EQ(PurePursuit::lookaheadDistance(0.0, c), 0.4);
  EXPECT_DOUBLE_EQ(PurePursuit::lookaheadDistance(0.3, c), 0.4);
  EXPECT_NEAR(PurePursuit::lookaheadDistance(1.0, c), 0.8, 1e-12);
  EXPECT_NEAR(PurePursuit::lookaheadDistance(-1.5, c), 1.2, 1e-12);
  EXPECT_DOUBLE_EQ(PurePursuit::lookaheadDistance(3.0, c), 1.8);
}

TEST(PurePursuit, LookaheadPointOnCircle)
{
  const auto p = straight(4.0);
  const Point2D g = PurePursuit::findLookahead(p, {1.0, 0.3}, 1.0, 0);
  EXPECT_NEAR(g.x, 1.0 + std::sqrt(1.0 - 0.09), 1e-9);
  EXPECT_NEAR(g.y, 0.0, 1e-12);
  EXPECT_NEAR(std::hypot(g.x - 1.0, g.y - 0.3), 1.0, 1e-9);
  // 원이 경로 끝을 넘으면 끝점
  const Point2D end = PurePursuit::findLookahead(p, {3.8, 0.0}, 1.0, 75);
  EXPECT_NEAR(end.x, 4.0, 1e-9);
  EXPECT_NEAR(PurePursuit::findLookahead({}, {1.0, 2.0}, 1.0, 0).y, 2.0, 1e-12);
}

TEST(PurePursuit, RotationCommand)
{
  EXPECT_NEAR(PurePursuit::rotationCommand(1.0, 1.5, 2.0), 1.5, 1e-12);
  EXPECT_NEAR(PurePursuit::rotationCommand(-0.1, 1.5, 2.0), -std::sqrt(0.4), 1e-12);
  EXPECT_NEAR(PurePursuit::rotationCommand(0.0, 1.5, 2.0), 0.0, 1e-12);
}

TEST(PurePursuit, SteersTowardPathAndModes)
{
  PurePursuit pp;
  EXPECT_EQ(pp.compute({0.0, 0.0, 0.0}, 0.0).mode, PurePursuitMode::kNoPath);
  pp.setPath(straight(5.0));
  // 경로 왼쪽 0.2 m → 우회전, CTE +0.2
  PurePursuitOutput o = pp.compute({0.5, 0.2, 0.0}, 0.5);
  EXPECT_EQ(o.mode, PurePursuitMode::kTrack);
  EXPECT_LT(o.w, 0.0);
  EXPECT_NEAR(o.cte, 0.2, 1e-9);
  EXPECT_GT(o.v, 0.0);
  EXPECT_NEAR(o.w, o.v * o.curvature, 1e-12);
  // 경로가 뒤쪽이고 거의 정지 → 제자리 회전
  pp.setPath(straight(5.0));
  o = pp.compute({1.0, 0.0, kPi}, 0.0);
  EXPECT_EQ(o.mode, PurePursuitMode::kRotateToPath);
  EXPECT_DOUBLE_EQ(o.v, 0.0);
  EXPECT_NEAR(std::abs(o.w), 1.5, 1e-12);
  // 목표 근처(0.05 m), 방향 오차 0.5 rad → 목표 방향 정렬
  pp.setPath(straight(5.0));
  o = pp.compute({4.95, 0.0, -0.5}, 0.05);
  EXPECT_EQ(o.mode, PurePursuitMode::kRotateToGoal);
  EXPECT_GT(o.w, 0.0);
  EXPECT_DOUBLE_EQ(o.v, 0.0);
  o = pp.compute({4.97, 0.0, 0.01}, 0.0);
  EXPECT_EQ(o.mode, PurePursuitMode::kGoalReached);
}

TEST(PurePursuit, SpeedRegulation)
{
  PurePursuitConfig c;
  c.desired_linear_vel = 1.5;
  PurePursuit pp(c);
  std::vector<Pose2D> p = straight(2.0);
  appendArc(p, 0.8, kPi / 2.0);
  appendStraight(p, 2.0);
  pp.setPath(p);
  // 곡선 진입 전 감속: 곡선 상한 √(0.8/1.25)=0.8 m/s 를 앞보기 원뿔로 반영. 곡률 창(0.25 m) 때문에
  // 완전 곡률은 호 시작 0.25 m 뒤(s = 2.25)부터 → 로봇(s = 1.5) 허용 속도 ≤ √(0.8² + 2·1·0.75)
  const PurePursuitOutput o = pp.compute({1.5, 0.0, 0.0}, 1.2);
  EXPECT_LT(o.v, 1.5);
  EXPECT_LE(o.v, std::sqrt(0.8 * 0.8 + 2.0 * 1.0 * 0.75) + 1e-6);
  // 외부 속도 제한
  pp.setPath(straight(5.0));
  EXPECT_LE(pp.compute({0.5, 0.0, 0.0}, 0.5, 0.3).v, 0.3 + 1e-12);
  // 목표 접근: 0.3 m 남으면 √(2·1·0.3) 이하
  EXPECT_LE(pp.compute({4.7, 0.0, 0.0}, 0.5).v, std::sqrt(0.6) + 1e-6);
}

TEST(PurePursuit, ClosedLoopStraightConvergesFromOffset)
{
  PurePursuit pp;
  const auto st = simulate(pp, straight(12.0), {0.0, 0.3, 0.0}, 5.0, {});
  EXPECT_TRUE(st.reached);
  std::printf(
    "[ info ] straight from 0.3 m offset: max CTE after 5 m %.4f m\n", st.max_cte_straight);
  EXPECT_LT(st.max_cte_straight, 0.02);
}

TEST(PurePursuit, ClosedLoopCurveWithinSpec)
{
  // 직선 3 m → R = 2 m 좌 90° → 직선 3 m → R = 1.5 m 우 90°
  //   → 직선 3 m (명세: 직선 5 cm, 곡선 10 cm)
  std::vector<Pose2D> p = straight(3.0);
  const double s1 = 3.0;
  appendArc(p, 2.0, kPi / 2.0);
  const double e1 = s1 + kPi;
  appendStraight(p, 3.0);
  const double s2 = e1 + 3.0;
  appendArc(p, 1.5, -kPi / 2.0);
  const double e2 = s2 + 0.75 * kPi;
  appendStraight(p, 3.0);
  for (bool cc : {false, true}) {
    PurePursuitConfig c;
    c.use_chord_correction = cc;
    PurePursuit pp(c);
    const auto st = simulate(pp, p, {0.0, 0.0, 0.0}, 0.5, {{s1, e1}, {s2, e2}});
    std::printf(
      "[ info ] %s: straight max/mean %.4f/%.4f m, curve max/mean %.4f/%.4f m, %.2f s\n",
      cc ? "CC-PP" : "PP", st.max_cte_straight, st.mean_cte_straight, st.max_cte_curve,
      st.mean_cte_curve, st.time);
    EXPECT_TRUE(st.reached);
    EXPECT_LT(st.mean_cte_straight, 0.05);
    EXPECT_LT(st.mean_cte_curve, 0.10);
    EXPECT_LT(st.max_cte_curve, 0.10);
  }
}

TEST(SpeedProfile, CurvatureCapAndCone)
{
  SpeedProfileConfig c;   // v 1.0, ω 1.5, a_lat 0.8
  EXPECT_DOUBLE_EQ(SpeedProfile::curvatureCap(0.0, c), 1.0);
  EXPECT_NEAR(SpeedProfile::curvatureCap(1.0, c), std::sqrt(0.8), 1e-12);
  EXPECT_NEAR(SpeedProfile::curvatureCap(-2.0, c), std::sqrt(0.4), 1e-12);
  c.max_lateral_accel = 0.0;
  EXPECT_NEAR(SpeedProfile::curvatureCap(3.0, c), 0.5, 1e-12);   // ω 한계만

  SpeedProfile prof;
  EXPECT_TRUE(prof.empty());
  EXPECT_DOUBLE_EQ(prof.allowedSpeed(0.0), 0.0);
  prof.build(straight(4.0), SpeedProfileConfig());
  EXPECT_EQ(prof.size(), 81U);
  EXPECT_NEAR(prof.totalLength(), 4.0, 1e-9);
  EXPECT_DOUBLE_EQ(prof.caps().back(), 0.0);
  EXPECT_NEAR(prof.allowedSpeed(3.2), 1.0, 1e-9);   // 저크 정지거리 d_stop(1.0) = 0.74 m 밖
  // 목표 정점: 저크 제한 정지거리의 역함수 (사다리꼴 √(2·a·0.1) = 0.447 보다 느리다)
  EXPECT_NEAR(prof.allowedSpeed(3.9), SpeedProfile::maxSpeedForStop(0.1, 1.0, 2.0, 0.0), 1e-9);
  EXPECT_NEAR(prof.allowedSpeed(3.9), 0.2823, 1e-3);
  EXPECT_NEAR(prof.allowedSpeed(1.0, 1.0), 1.0, 1e-12);   // 지평 안에 감속점 없음
  // 끝이 목표가 아니면 상한 유지
  prof.build(straight(4.0), SpeedProfileConfig(), false);
  EXPECT_DOUBLE_EQ(prof.caps().back(), 1.0);
}

// 제어기(20 Hz) → 저크 제한 필터(50 Hz, velocity_profiler_node) → 1차 지연 + 순수 지연
// 서보(100 Hz) 체인.
// 목표 도달 판정은 controller_server 의 SimpleGoalChecker 처럼 d_goal ≤ 0.10 m 에서 명령 0.
// 사다리꼴 접근(√(2ad))은 하류 필터가 늦게 감속해 목표를 지나친다
// → 저크·지연 정지거리 역함수로 감속.
struct ApproachResult
{
  double overshoot{0.0};   // [m] 정지 위치 − 목표 (+ = 지나침)
  double time{0.0};        // [s] 출발 → 정지
};

ApproachResult approach(double approach_latency, double jerk)
{
  using amr_navigation::core::FirstOrderPlant;
  using amr_navigation::core::JerkLimitedFilter;
  using amr_navigation::core::JerkLimits;
  PurePursuitConfig c;
  c.approach_latency = approach_latency;
  c.max_linear_jerk = jerk;
  PurePursuit pp(c);
  pp.setPath(straight(5.0));
  JerkLimitedFilter filt(JerkLimits{-0.5, 2.0, 1.0, 2.0});
  FirstOrderPlant plant(1.0, 0.08, 0.04, 0.01);
  Pose2D pose{0.0, 0.0, 0.0};
  double v_cmd = 0.0;
  double v_ref = 0.0;
  double v = 0.0;
  bool reached = false;
  ApproachResult r;
  for (int k = 0; k < 3000; ++k) {
    if (k % 5 == 0 && !reached) {
      const PurePursuitOutput out = pp.compute(pose, v_ref);
      reached = out.d_goal <= 0.10;
      v_cmd = reached ? 0.0 : out.v;
    }
    if (k % 2 == 0) {
      v_ref = filt.step(v_cmd, 0.02);
    }
    v = plant.step(v_ref);
    pose = integrateArc(pose, v, 0.0, 0.01);
    r.time += 0.01;
    if (reached && std::abs(v) < 1e-4 && std::abs(v_ref) < 1e-4) {
      break;
    }
  }
  EXPECT_TRUE(reached);
  r.overshoot = pose.x - 5.0;
  return r;
}

TEST(PurePursuit, GoalApproachThroughJerkFilterStopsInsideTolerance)
{
  // 튜닝 표 (path_tracking.md §1.4): 사다리꼴(저크 0, 지연 0) 대비 approach_latency 별 통과량·시간
  const ApproachResult trap = approach(0.0, 0.0);
  std::printf("[ info ] trapezoid: overshoot %.3f m, %.2f s\n", trap.overshoot, trap.time);
  for (double tc : {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}) {
    const ApproachResult r = approach(tc, 2.0);
    std::printf(
      "[ info ] jerk-aware t_c %.1f s: overshoot %.3f m, %.2f s\n", tc, r.overshoot, r.time);
  }
  PurePursuitConfig def;
  const ApproachResult r = approach(def.approach_latency, def.max_linear_jerk);
  EXPECT_LT(r.overshoot, 0.02);
  EXPECT_GT(r.overshoot, -0.10);
  EXPECT_GT(trap.overshoot, 0.2);
}

TEST(SpeedProfile, StopSpeedInvertsStoppingDistance)
{
  // 저크 없음·지연 없음 → 사다리꼴 √(2ad)
  EXPECT_NEAR(SpeedProfile::maxSpeedForStop(0.5, 1.0, 0.0, 0.0), 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(SpeedProfile::maxSpeedForStop(0.0, 1.0, 2.0, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(SpeedProfile::maxSpeedForStop(1.0, 0.0, 2.0, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(SpeedProfile::stoppingDistance(0.0, 1.0, 2.0, 0.1), 0.0);
  EXPECT_TRUE(std::isinf(SpeedProfile::stoppingDistance(1.0, 0.0, 2.0, 0.1)));
  // 램프 도중 정지 구간(v ≤ a²/2j = 0.25)과 등감속 구간 모두에서 역함수 관계, 지연이 클수록 느리다
  double prev = 0.0;
  for (double d = 0.01; d < 3.0; d += 0.037) {
    const double v = SpeedProfile::maxSpeedForStop(d, 1.0, 2.0, 0.1);
    EXPECT_NEAR(SpeedProfile::stoppingDistance(v, 1.0, 2.0, 0.1), d, 1e-6);
    EXPECT_LT(v, std::sqrt(2.0 * d));
    EXPECT_LE(v, SpeedProfile::maxSpeedForStop(d, 1.0, 2.0, 0.0));
    EXPECT_GT(v, prev);   // 단조 증가
    prev = v;
  }
}

TEST(SpeedProfile, VelocityProfileRespectsAccel)
{
  std::vector<Pose2D> p = straight(3.0);
  appendArc(p, 1.0, kPi / 2.0);
  appendStraight(p, 3.0);
  SpeedProfileConfig c;
  c.desired_speed = 1.5;
  SpeedProfile prof;
  prof.build(p, c);
  const auto v = prof.velocityProfile(0.0);
  const auto & s = prof.arcLength();
  ASSERT_EQ(v.size(), p.size());
  EXPECT_DOUBLE_EQ(v.front(), 0.0);
  EXPECT_DOUBLE_EQ(v.back(), 0.0);
  for (std::size_t i = 1; i < v.size(); ++i) {
    const double ds = s[i] - s[i - 1];
    EXPECT_LE(v[i] * v[i] - v[i - 1] * v[i - 1], 2.0 * c.max_accel * ds + 1e-9);
    EXPECT_LE(v[i - 1] * v[i - 1] - v[i] * v[i], 2.0 * c.max_decel * ds + 1e-9);
    EXPECT_LE(v[i], prof.caps()[i] + 1e-12);
  }
}

TEST(SpeedProfile, TravelTimePrediction)
{
  SpeedProfile prof;
  SpeedProfileConfig c;   // v 1, a = d = 1, j 2
  prof.build(straight(4.0), c);
  // 사다리꼴: L/v + v/a = 5 s, S-curve 보정 a/j = 0.5 s
  EXPECT_NEAR(prof.predictTravelTime(0.0), 5.5, 1e-6);
  c.max_jerk = 0.0;
  prof.build(straight(4.0), c);
  EXPECT_NEAR(prof.predictTravelTime(0.0), 5.0, 1e-6);
  // 제자리 회전 시간: 사다리꼴 / 삼각
  EXPECT_NEAR(SpeedProfile::rotationTime(kPi / 2.0, 1.5, 2.0), kPi / 3.0 + 0.75, 1e-12);
  EXPECT_NEAR(SpeedProfile::rotationTime(-0.5, 1.5, 2.0), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(SpeedProfile::rotationTime(0.0, 1.5, 2.0), 0.0);
  EXPECT_NEAR(prof.predictTravelTime(0.0, kPi / 2.0, 0.5), 5.0 + kPi / 3.0 + 0.75 + 1.0, 1e-6);
  // 경로가 점 하나면 회전 시간만
  SpeedProfile one;
  one.build({{0.0, 0.0, 0.0}}, c);
  EXPECT_NEAR(one.predictTravelTime(0.0, 0.5, 0.0), 1.0, 1e-12);
}
