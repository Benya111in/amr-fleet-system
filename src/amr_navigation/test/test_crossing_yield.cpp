// 횡단 양보(가상 정지선) 코어 단위 테스트: 교차 구간 기하, 점유 시각, 통과·양보·진입 판정.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "amr_navigation/core/crossing_yield.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/speed_profile.hpp"

using amr_navigation::core::DynamicObstacle;
using amr_navigation::core::Pose2D;
using amr_navigation::core::SpeedProfile;
using amr_navigation::core::YieldConfig;
using amr_navigation::core::YieldDecision;
using amr_navigation::core::YieldResult;
using amr_navigation::core::YieldState;
using amr_navigation::core::cumulativeLength;
using amr_navigation::core::evaluateYield;
using amr_navigation::core::travelTime;

namespace
{
std::vector<Pose2D> straightPath(double x0, double y0, double len, double theta = 0.0)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    p.push_back({x0 + s * std::cos(theta), y0 + s * std::sin(theta), theta});
  }
  return p;
}

// 경로 y = 0 위 로봇 (s_robot = x), 장애물 하나.
YieldResult run(const DynamicObstacle & o, double x_robot, double v, const YieldConfig & cfg)
{
  const auto path = straightPath(0.0, 0.0, 12.0);
  const auto cum = cumulativeLength(path);
  const Pose2D robot{x_robot, 0.0, 0.0};
  return evaluateYield(path, cum, robot, x_robot, v, {o}, cfg);
}
}  // namespace

TEST(CrossingYield, TravelTimeAcceleratesToCap)
{
  EXPECT_NEAR(travelTime(0.0, 0.0, 1.0, 1.0), 0.0, 1e-12);
  // 정지에서 a = 1 로 v_max = 1 까지: 0.5 m 를 1 s 에
  EXPECT_NEAR(travelTime(0.5, 0.0, 1.0, 1.0), 1.0, 1e-9);
  // 그 뒤는 등속
  EXPECT_NEAR(travelTime(2.5, 0.0, 1.0, 1.0), 1.0 + 2.0, 1e-9);
  // 이미 상한이면 등속
  EXPECT_NEAR(travelTime(3.0, 1.0, 1.0, 1.0), 3.0, 1e-9);
  // 가속이 0 이면 현재 속도로 (상한까지 올라간다고 보면 도달 시각을 낙관해 양보를 건너뛴다)
  EXPECT_NEAR(travelTime(2.0, 0.5, 0.0, 1.0), 4.0, 1e-9);
  // 가속도 속도도 0 이면 영영 못 간다
  EXPECT_GT(travelTime(1.0, 0.0, 0.0, 1.0), 1e6);
}

TEST(CrossingYield, CrossingGivesStopLineOutsideCorridor)
{
  YieldConfig cfg;   // R_c = 0.361 + 0.25 + 0.50 = 1.111 m
  const DynamicObstacle o{5.0, 5.0, 0.0, -1.0, 0.25};   // x = 5 차선을 −y 로 1 m/s
  const double R = cfg.robot_radius + o.radius + cfg.corridor_margin;
  YieldResult r = run(o, 0.0, 1.0, cfg);
  EXPECT_EQ(r.state, YieldState::kYield);
  EXPECT_EQ(r.obstacle, 0);
  // 교차 구간 = 차선 폭 ±R (경로 간격 0.05 m 만큼 보수적으로 일찍)
  EXPECT_NEAR(r.zone_entry, 5.0 - R, 0.06);
  EXPECT_NEAR(r.zone_exit, 5.0 + R, 0.06);        // 창(4 m) 밖은 접선 외삽
  EXPECT_NEAR(r.stop_distance, r.zone_entry - cfg.stop_margin, 1e-9);
  // 장애물 점유 시각 [ (5 − R)/1, (5 + R)/1 ]
  EXPECT_NEAR(r.obstacle_in, 5.0 - R, 1e-6);
  EXPECT_NEAR(r.obstacle_out, 5.0 + R, 1e-6);
  // 아직 멀어 속도 상한이 운용 속도보다 높다 (감속 없음)
  EXPECT_GT(r.speed_limit, cfg.v_max);

  // 정지선 0.6 m 앞: 상한이 정지거리 역함수와 같다
  r = run(o, 5.0 - R - 0.85, 1.0, cfg);
  EXPECT_EQ(r.state, YieldState::kYield);
  EXPECT_NEAR(r.stop_distance, 0.6, 0.06);
  EXPECT_NEAR(
    r.speed_limit,
    SpeedProfile::maxSpeedForStop(r.stop_distance, cfg.decel, cfg.jerk, cfg.latency), 1e-9);
  EXPECT_LT(r.speed_limit, cfg.v_max);
  // 정지선 위에서는 상한 0 (그대로 선다)
  EXPECT_NEAR(run(o, 5.0 - R - cfg.stop_margin, 0.0, cfg).speed_limit, 0.0, 1e-9);
}

TEST(CrossingYield, ObstaclePassedBehindIsClear)
{
  YieldConfig cfg;
  // 경로를 이미 지나 멀어지는 중 (통로는 진행 방향 앞쪽만)
  const YieldResult r = run({5.0, -1.5, 0.0, -1.0, 0.25}, 0.0, 1.0, cfg);
  EXPECT_EQ(r.state, YieldState::kClear);
  EXPECT_FALSE(std::isfinite(r.speed_limit) && r.speed_limit < cfg.v_max);
  EXPECT_EQ(r.obstacle, -1);
}

TEST(CrossingYield, ObstacleLeavesZoneBeforeRobotArrivesIsClear)
{
  YieldConfig cfg;
  // 아직 통로가 경로에 걸쳐 있으나(중심 −1.0 m) 0.11 s 뒤면 빠져나간다 — 로봇은 3.9 s 뒤 도착
  const YieldResult r = run({5.0, -1.0, 0.0, -1.0, 0.25}, 0.0, 1.0, cfg);
  EXPECT_EQ(r.state, YieldState::kClear);
}

TEST(CrossingYield, CorridorThatMissesThePathIsClear)
{
  YieldConfig cfg;
  // 경로와 나란한 차선이 4 m 옆 — 통로가 경로를 덮지 않는다
  EXPECT_EQ(run({3.0, 4.0, 1.0, 0.0, 0.25}, 0.0, 1.0, cfg).state, YieldState::kClear);
  // 느린 트랙은 정적 취급 (코스트맵이 처리)
  EXPECT_EQ(run({5.0, 5.0, 0.0, -0.1, 0.25}, 0.0, 1.0, cfg).state, YieldState::kClear);
  // 경로 창(lookahead 4 m) 밖의 교차
  EXPECT_EQ(run({9.0, 5.0, 0.0, -1.0, 0.25}, 0.0, 1.0, cfg).state, YieldState::kClear);
  // 장애물이 없으면 판정도 없다
  const auto path = straightPath(0.0, 0.0, 12.0);
  EXPECT_EQ(
    evaluateYield(path, cumulativeLength(path), {0.0, 0.0, 0.0}, 0.0, 1.0, {}, cfg).state,
    YieldState::kClear);
}

TEST(CrossingYield, RobotThatClearsFirstDoesNotYield)
{
  YieldConfig cfg;
  // 1.5 m 앞 차선, 장애물은 5 m 위 — 로봇이 2.6 s 에 빠져나가고 장애물은 3.9 s 에 온다
  const YieldResult r = run({1.5, 5.0, 0.0, -1.0, 0.25}, 0.0, 1.0, cfg);
  EXPECT_EQ(r.state, YieldState::kClear);
  // 같은 배치라도 로봇이 서 있으면(늦게 도착) 양보한다
  EXPECT_EQ(run({1.5, 5.0, 0.0, -1.0, 0.25}, 0.0, 0.0, cfg).state, YieldState::kYield);
}

TEST(CrossingYield, HeadOnHasNoStopLine)
{
  YieldConfig cfg;
  // 정면으로 마주 오는 장애물의 통로는 로봇 자신의 차선과 나란하다 → 통로 밖에 설 곳이 없으므로
  // 정지선을 두지 않는다 (VO/TTC 가 맡는다 — dwa.md §2.1).
  const YieldResult r = run({3.0, 0.0, -1.0, 0.0, 0.25}, 0.0, 0.8, cfg);
  EXPECT_EQ(r.state, YieldState::kClear);
  EXPECT_FALSE(std::isfinite(r.speed_limit));
}

TEST(CrossingYield, InsideTheZoneIsCommittedNotStopped)
{
  YieldConfig cfg;
  cfg.lookahead = 8.0;
  // 비스듬한 교차 구간 **안**에 이미 들어선 로봇: 멈추면 차선 안이므로 상한을 두지 않는다
  const double th = -std::atan2(4.0, 8.0);
  const auto path = straightPath(0.0, 0.0, 12.0, th);
  const auto cum = cumulativeLength(path);
  const DynamicObstacle o{3.0, -3.0, 1.0, 0.0, 0.25};   // 곧 닿는다 (먼저 빠져나가지 못한다)
  const double s = 6.0;   // 교차 구간(≈ 4.2 ~ 9.2 m) 안
  const Pose2D robot{s * std::cos(th), s * std::sin(th), th};
  const YieldResult r = evaluateYield(path, cum, robot, s, 0.5, {o}, cfg);
  EXPECT_EQ(r.state, YieldState::kCommitted);
  EXPECT_FALSE(std::isfinite(r.speed_limit));
}

TEST(CrossingYield, CorridorUsesTheSmoothedVelocityNotTheRawOne)
{
  // 통로 축은 평활 속도로 세운다: 추적기 진행각이 직선 보행자에 대해서도 주기간 최대 144.9°
  // 흔들려(08 실측), 원시 속도를 쓰면 통로가 그만큼 돌고 교차 구간이 미터 단위로 이동한다.
  // VO·TTC 는 이 함수 밖에서 원시 속도를 그대로 쓴다.
  YieldConfig cfg;
  cfg.lookahead = 8.0;
  const auto path = straightPath(0.0, 0.0, 12.0);
  const auto cum = cumulativeLength(path);
  const Pose2D robot{0.0, 0.0, 0.0};
  DynamicObstacle o{5.0, -3.0, 1.0, 0.0, 0.25};      // 원시: 경로와 나란히 (교차 없음)
  ASSERT_EQ(evaluateYield(path, cum, robot, 0.0, 0.8, {o}, cfg).state, YieldState::kClear);
  o.vx_pred = 0.0;                                   // 평활: 경로를 가로지른다
  o.vy_pred = 1.0;
  o.has_pred = true;
  const YieldResult r = evaluateYield(path, cum, robot, 0.0, 0.8, {o}, cfg);
  EXPECT_EQ(r.state, YieldState::kYield);            // 평활 속도를 따랐다
  EXPECT_TRUE(std::isfinite(r.stop_distance));
}

TEST(CrossingYield, CommittedBeatsAnotherObstaclesStopLineInEitherOrder)
{
  YieldConfig cfg;
  cfg.lookahead = 8.0;
  const double th = -std::atan2(4.0, 8.0);
  const auto path = straightPath(0.0, 0.0, 12.0, th);
  const auto cum = cumulativeLength(path);
  const double s = 6.0;
  const Pose2D robot{s * std::cos(th), s * std::sin(th), th};
  const DynamicObstacle inside{3.0, -3.0, 1.0, 0.0, 0.25};   // 이 통로 안에 이미 들어섰다
  const DynamicObstacle ahead{8.0, -6.0, 0.0, 1.0, 0.25};    // 앞을 가로지른다 (혼자면 정지선)
  ASSERT_EQ(
    evaluateYield(path, cum, robot, s, 0.5, {inside}, cfg).state, YieldState::kCommitted);
  ASSERT_EQ(evaluateYield(path, cum, robot, s, 0.5, {ahead}, cfg).state, YieldState::kYield);
  // 둘을 같이 주면 순서와 무관하게 "빠져나간다" 가 이긴다 — 정지선을 지키면 남의 차선 안에 선다
  for (const std::vector<DynamicObstacle> & obs :
    {std::vector<DynamicObstacle>{inside, ahead}, std::vector<DynamicObstacle>{ahead, inside}})
  {
    const YieldResult r = evaluateYield(path, cum, robot, s, 0.5, obs, cfg);
    EXPECT_EQ(r.state, YieldState::kCommitted);
    EXPECT_FALSE(std::isfinite(r.speed_limit));
    EXPECT_FALSE(std::isfinite(r.stop_distance));
  }
}

TEST(CrossingYield, SameLaneLeaderIsNotAStopLineCase)
{
  YieldConfig cfg;
  // 같은 차선을 같은 방향으로 느리게 가는 장애물: 통로와 경로가 나란해 교차 구간이 없다
  // (max_zone 초과) → 정지선 없음. 추종·정지는 VO/TTC 와 코스트맵이 맡는다.
  const YieldResult r = run({3.0, 0.0, 0.3, 0.0, 0.25}, 0.0, 1.0, cfg);
  EXPECT_EQ(r.state, YieldState::kClear);
}

TEST(CrossingYield, ObliqueCrossingZoneIsLongerThanTheLaneWidth)
{
  YieldConfig cfg;
  cfg.lookahead = 8.0;
  // 경로 방향 −26.57°, 차선은 +x (y = −3) — 비스듬한 교차라 구간이 차선 폭보다 길다
  const double th = -std::atan2(4.0, 8.0);
  const auto path = straightPath(0.0, 0.0, 12.0, th);
  const auto cum = cumulativeLength(path);
  const DynamicObstacle o{-0.7, -3.0, 1.0, 0.0, 0.25};
  const double R = cfg.robot_radius + o.radius + cfg.corridor_margin;
  const YieldResult r = evaluateYield(path, cum, {0.0, 0.0, th}, 0.0, 1.0, {o}, cfg);
  EXPECT_EQ(r.state, YieldState::kYield);
  const double expect_zone = 2.0 * R / std::abs(std::sin(th));
  EXPECT_NEAR(r.zone_exit - r.zone_entry, expect_zone, 0.10);
  EXPECT_GT(expect_zone, 2.0 * R);
  // 정지선은 통로 밖: 로봇이 거기 섰을 때 차선 축까지의 수직 거리 > R
  const double y_stop = r.stop_distance * std::sin(th);
  EXPECT_GT(std::abs(y_stop - (-3.0)), R);
}

TEST(CrossingYield, DisabledOrShortPathGivesNoLimit)
{
  YieldConfig cfg;
  cfg.enable = false;
  EXPECT_EQ(run({5.0, 5.0, 0.0, -1.0, 0.25}, 0.0, 1.0, cfg).state, YieldState::kClear);
  cfg.enable = true;
  const std::vector<Pose2D> one{{0.0, 0.0, 0.0}};
  EXPECT_EQ(
    evaluateYield(
      one, cumulativeLength(one), {0.0, 0.0, 0.0}, 0.0, 1.0,
      {{5.0, 5.0, 0.0, -1.0, 0.25}}, cfg).state, YieldState::kClear);
}

TEST(CrossingYield, CommittedReleasesWhenTheBodyHasPassedNotTheMargin)
{
  // 통로 반폭 R_c 에는 접근 판단용 여유(corridor_margin)가 들어 있다. 그 여유로 해제까지
  // 재면 장애물이 지나간 뒤에도 corridor_margin/|u| 만큼 더 붙잡힌다. 08 실측 f8a 1번 시행:
  // 위협이 사라지고도(ttc inf, VO 기각 0) 0.70 s 동안 v ≈ 0.01 로 서 있었고 — 그 값이
  // corridor_margin 0.7 m ÷ 작업자 1.0 m/s 와 같다 — 복귀가 5.02 s 로 명세 4.7 상한을 넘겼다.
  YieldConfig cfg;   // R_c = 0.361 + 0.25 + 0.50 = 1.111 m, 몸 기준 반경 합 = 0.611 m
  const double body = cfg.robot_radius + 0.25;
  const double rc = body + cfg.corridor_margin;
  // 로봇은 통로 안(x = 3.0), 보행자는 경로를 가로질러 +y 로 지나간다.
  const YieldResult mid = run({3.0, 0.4, 0.0, 1.0, 0.25}, 3.0, 0.5, cfg);
  EXPECT_EQ(mid.state, YieldState::kCommitted);     // 아직 몸이 교차 구간에 걸쳐 있다 (0.4 < 0.611)
  const YieldResult past = run({3.0, 0.8, 0.0, 1.0, 0.25}, 3.0, 0.5, cfg);
  EXPECT_EQ(past.state, YieldState::kClear);        // 몸은 지났다 (0.611 < 0.8 < 1.111)
  // 여유로 쟀다면 1.111 m 를 지날 때까지 붙잡혔다 — 1.0 m/s 에서 0.5 s 의 헛기다림이다.
  EXPECT_LT(body, 0.8);
  EXPECT_LT(0.8, rc);
}

// --- kCommitted 진입 결정 go / hold / retreat (08 접촉 4건의 공통 기제를 끊는 층) -----------
namespace
{
// 08 기하: 경로 −26.57°, 차선 y = −3 (+x 방향 û, 좌수직 n̂ = +y). 배포 설정(통로 여유 0.7).
YieldConfig deployedCfg()
{
  YieldConfig cfg;
  cfg.corridor_margin = 0.7;
  cfg.stop_margin = 0.8;
  cfg.clear_margin = 2.0;
  cfg.max_zone = 7.0;
  cfg.lookahead = 4.0;
  return cfg;
}
const double kTh = -std::atan2(4.0, 8.0);
std::vector<Pose2D> obliquePath() {return straightPath(0.0, 0.0, 14.0, kTh);}
// 차선 y = −3 에서 |β| 만큼 북쪽에 있는 경로 점의 호길이
double sAtBeta(double beta) {return (3.0 - beta) / -std::sin(kTh);}
Pose2D robotAt(double s) {return {s * std::cos(kTh), s * std::sin(kTh), kTh};}
}  // namespace

TEST(CrossingYield, CommittedDecisionGoWhenTheRobotClearsTheBodyZoneFirst)
{
  // InsideTheZoneIsCommittedNotStopped 의 배치: 로봇은 위험 구간 안(β 0.32)에서 0.5 m/s, 작업자는
  // 2.4 m 뒤에서 같은 쪽으로 1.0 m/s. 출구까지 로봇 2.5 s, 작업자 앞은 3.9 s → 여유 1.4 s → go.
  YieldConfig cfg;
  cfg.lookahead = 8.0;
  const auto path = straightPath(0.0, 0.0, 12.0, kTh);
  const auto cum = cumulativeLength(path);
  const double s = 6.0;
  const DynamicObstacle behind{3.0, -3.0, 1.0, 0.0, 0.25};
  const YieldResult r = evaluateYield(path, cum, robotAt(s), s, 0.5, {behind}, cfg);
  EXPECT_EQ(r.state, YieldState::kCommitted);
  EXPECT_EQ(r.decision, YieldDecision::kGo);
  EXPECT_GT(r.margin, 1.0);                          // 시간 여유 ≈ 1.4 s
  EXPECT_GE(r.clearance, cfg.go_clearance);          // 26.6°: 0.15 + 0.236·(1.4 + 0.611) ≈ 0.62 m
  EXPECT_FALSE(std::isfinite(r.speed_limit));
  ASSERT_EQ(r.decided.size(), 1u);
  EXPECT_TRUE(r.decided[0]);
  // 결정을 끄면 예전 그대로 (상한 없음, 결정 없음)
  cfg.decision_enable = false;
  const YieldResult off = evaluateYield(path, cum, robotAt(s), s, 0.5, {behind}, cfg);
  EXPECT_EQ(off.state, YieldState::kCommitted);
  EXPECT_EQ(off.decision, YieldDecision::kNone);
}

TEST(CrossingYield, CommittedDecisionHoldStopsBeforeTheBodyZoneAndSticks)
{
  // reg1 시행 13 형: 통로 진입점(β 1.30)에서 1.0 m/s, 작업자가 4.5 m 앞에서 마주 온다.
  // 위험 구간 입구(β = 0.611 + 0.15)까지 경로로 (1.30 − 0.761)/sin 26.6° ≈ 1.2 m ≥ d_stop(1.0) 1.04
  // → hold, 상한 = d_stop⁻¹(입구까지). 더 다가가 못 설 거리가 돼도 이미 hold 였으면 계속 선다.
  const YieldConfig cfg = deployedCfg();
  const auto path = obliquePath();
  const auto cum = cumulativeLength(path);
  const double s0 = sAtBeta(1.30);
  const Pose2D robot = robotAt(s0);
  const DynamicObstacle o{robot.x + 4.5, -3.0, -1.0, 0.0, 0.25};
  const YieldResult r = evaluateYield(path, cum, robot, s0, 1.0, {o}, cfg);
  ASSERT_EQ(r.state, YieldState::kCommitted);
  EXPECT_EQ(r.decision, YieldDecision::kHold);
  EXPECT_LT(r.clearance, cfg.go_clearance);
  EXPECT_NEAR(r.body_entry, (1.30 - 0.761) / -std::sin(kTh), 0.08);
  EXPECT_TRUE(std::isfinite(r.speed_limit));
  EXPECT_NEAR(
    r.speed_limit, SpeedProfile::maxSpeedForStop(r.body_entry, cfg.decel, cfg.jerk, cfg.latency),
    1e-9);
  // 0.9 m 더 간 자리(입구 0.3 m 앞)에서 아직 0.9 m/s: 새로 판정하면 못 서므로 retreat 이지만,
  // 직전 결정이 hold 면 계속 hold (상한 = 남은 거리의 역함수 → 최대 감속)
  const double s1 = s0 + 0.9;
  const YieldResult fresh = evaluateYield(path, cum, robotAt(s1), s1, 0.9, {o}, cfg);
  EXPECT_EQ(fresh.decision, YieldDecision::kRetreat);
  const YieldResult held =
    evaluateYield(path, cum, robotAt(s1), s1, 0.9, {o}, cfg, YieldDecision::kHold);
  EXPECT_EQ(held.decision, YieldDecision::kHold);
  EXPECT_LT(held.speed_limit, 0.9);
}

TEST(CrossingYield, CommittedDecisionRetreatInsideTheBodyZone)
{
  // 몸 원통 안(β 0.4)에서 0.8 m/s, 작업자 4 m 앞에서 마주 온다: 출구까지 2.6 m ≈ 2.7 s 인데 작업자
  // 앞은 1.1 s 뒤 → go 도 hold 도 아니다 → retreat (상한 없음, 입구 0).
  const YieldConfig cfg = deployedCfg();
  const auto path = obliquePath();
  const auto cum = cumulativeLength(path);
  const double s = sAtBeta(0.4);
  const Pose2D robot = robotAt(s);
  const DynamicObstacle ahead{robot.x + 4.0, -3.0, -1.0, 0.0, 0.25};
  const YieldResult r = evaluateYield(path, cum, robot, s, 0.8, {ahead}, cfg);
  ASSERT_EQ(r.state, YieldState::kCommitted);
  EXPECT_EQ(r.decision, YieldDecision::kRetreat);
  EXPECT_FALSE(std::isfinite(r.speed_limit));
  EXPECT_NEAR(r.body_entry, 0.0, 1e-9);
  // 부호: 서향(û = −x) 작업자의 좌수직 n̂ 은 −y 라 차선 북쪽(+y)은 β < 0
  EXPECT_NEAR(r.body_beta, -0.4, 0.05);
  ASSERT_EQ(r.decided.size(), 1u);
  EXPECT_TRUE(r.decided[0]);                // retreat 도 결정 층이 맡는다 (TTC₀ 벌점 제외)
}

TEST(CrossingYield, CommittedDecisionNoneOnceTheWalkerHasPassed)
{
  // 작업자 중심이 로봇을 지났으나 몸은 아직 걸쳐 있다 (α −0.4): kCommitted 지만 결정은 없다
  YieldConfig cfg;
  const YieldResult r = run({3.0, 0.4, 0.0, 1.0, 0.25}, 3.0, 0.5, cfg);
  EXPECT_EQ(r.state, YieldState::kCommitted);
  EXPECT_EQ(r.decision, YieldDecision::kNone);
}

TEST(CrossingYield, GoHasHysteresis)
{
  // 마주 오는 배치(08 형): 몸 원통 안(β 0.4)·0.8 m/s, 출구까지 2.6 m ≈ 2.6 s, 작업자는 5.75 m 앞 →
  // 시간 여유 ≈ 0.2 s → 예측 여유 ≈ 0.15 + 0.236·(0.2 + 0.611) ≈ 0.34 m — 새로 들어가기(0.40)
  // 에는 모자라지만 이미 go 였다면(유지 0.30) go 를 지킨다. 추적기 속도 추정이 한 주기 튀어도
  // 뒤집히지 않는다. (같은 방향으로 뒤에서 오는 배치는 닫힘 속도가 0.1 m/s 라 여유가 m 단위로 커
  // 이 띠를 만들 수 없다.)
  const YieldConfig cfg = deployedCfg();
  const auto path = obliquePath();
  const auto cum = cumulativeLength(path);
  const double s = sAtBeta(0.4);
  const Pose2D robot = robotAt(s);
  const DynamicObstacle o{robot.x + 5.75, -3.0, -1.0, 0.0, 0.25};
  const YieldResult fresh = evaluateYield(path, cum, robot, s, 0.8, {o}, cfg);
  const YieldResult kept = evaluateYield(path, cum, robot, s, 0.8, {o}, cfg, YieldDecision::kGo);
  ASSERT_EQ(fresh.state, YieldState::kCommitted);
  std::printf(
    "[ info ] go hysteresis: time margin %.2f s, predicted clearance %.3f m\n", fresh.margin,
    fresh.clearance);
  EXPECT_GT(fresh.margin, 0.0);
  EXPECT_GE(fresh.clearance, cfg.go_hold_clearance);
  EXPECT_LT(fresh.clearance, cfg.go_clearance);
  EXPECT_NE(fresh.decision, YieldDecision::kGo);
  EXPECT_EQ(kept.decision, YieldDecision::kGo);
}

TEST(CrossingYield, SplitTracksRecordTheNearestAxisAndTheMostConservativeDecision)
{
  // 근거리에서 사람 하나가 두 트랙으로 갈라진다 (08 접촉 4건 모두 0.4~1.0 s 전, 0.35~0.39 m 비낌).
  // 예전 규칙(출구가 가장 먼 통로)은 비낀 쪽을 골라 β 를 과대평가했다. 이제 축이 가장 가까운
  // 트랙을 남기고, 결정은 둘 중 보수적인 쪽이다.
  const YieldConfig cfg = deployedCfg();
  const auto path = obliquePath();
  const auto cum = cumulativeLength(path);
  const double s = sAtBeta(0.9);
  const Pose2D robot = robotAt(s);
  const DynamicObstacle body{robot.x + 2.0, -3.0, -1.0, 0.0, 0.25};      // 진짜 몸: β 0.9
  const DynamicObstacle split{robot.x + 2.0, -2.65, -1.0, 0.0, 0.25};    // 비낀 중복: β 0.55
  for (const std::vector<DynamicObstacle> & obs :
    {std::vector<DynamicObstacle>{body, split}, std::vector<DynamicObstacle>{split, body}})
  {
    const YieldResult r = evaluateYield(path, cum, robot, s, 0.6, obs, cfg);
    ASSERT_EQ(r.state, YieldState::kCommitted);
    EXPECT_NEAR(std::abs(r.body_beta), 0.55, 0.05);            // 가장 가까운 축
    EXPECT_NEAR(obs[static_cast<std::size_t>(r.obstacle)].y, -2.65, 1e-9);
    EXPECT_EQ(r.decision, YieldDecision::kRetreat);   // 중복 축 기준으로는 원통 안 → retreat
  }
}
