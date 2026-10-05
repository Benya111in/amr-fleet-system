// 횡단 게이트(T안) 코어 계약: 노출 구간, 정지점, 도달가능성(양방향·되돌아옴), 커밋 후 불간섭,
// 정지 중 v = 0 / w = 0, 경로 후진. 상수는 출발 위상 610개 시뮬레이션에서 접촉 0/610 을 낸 값이다.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include "amr_navigation/core/crossing_gate.hpp"
#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/speed_profile.hpp"

using amr_navigation::core::DwaConfig;
using amr_navigation::core::DwaInput;
using amr_navigation::core::DwaPlanner;
using amr_navigation::core::DwaResult;
using amr_navigation::core::DynamicObstacle;
using amr_navigation::core::GateConfig;
using amr_navigation::core::GatePhase;
using amr_navigation::core::GateReason;
using amr_navigation::core::GateResult;
using amr_navigation::core::GateState;
using amr_navigation::core::GateTrack;
using amr_navigation::core::Point2D;
using amr_navigation::core::Pose2D;
using amr_navigation::core::SpeedProfile;
using amr_navigation::core::YieldState;
using amr_navigation::core::canReach;
using amr_navigation::core::cumulativeLength;
using amr_navigation::core::evaluateGate;
using amr_navigation::core::integrateArc;
using amr_navigation::core::reversalReturnTime;
using amr_navigation::core::travelTime;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

std::vector<Pose2D> straightPath(double len = 12.0)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    p.push_back({s, 0.0, 0.0});
  }
  return p;
}

GateConfig gateCfg()
{
  GateConfig c;
  c.enable = true;
  c.lookahead = 12.0;   // 시험 경로(12 m) 전체를 본다 — 배포값 8 m 이면 구간 끝이 창에서 잘린다
  return c;
}

GateTrack walker(int id, double x, double y, double ux, double uy, double speed = 1.0)
{
  GateTrack t;
  t.id = id;
  t.x = x;
  t.y = y;
  t.radius = 0.30;
  t.heading_known = true;
  t.ux = ux;
  t.uy = uy;
  t.speed = speed;
  return t;
}

GateTrack stander(int id, double x, double y, double stationary_s = 0.0)
{
  GateTrack t;
  t.id = id;
  t.x = x;
  t.y = y;
  t.radius = 0.30;
  t.heading_known = false;
  t.stationary_s = stationary_s;
  return t;
}

// 경로 y = 0 (x = 0..12) 위 로봇 (s_robot = x)
GateResult gate(
  const std::vector<GateTrack> & tracks, double s_robot, double v, const GateConfig & cfg,
  const GateState & prev = GateState{})
{
  const auto path = straightPath();
  const auto cum = cumulativeLength(path);
  const Pose2D robot{s_robot, 0.0, 0.0};
  return evaluateGate(path, cum, robot, s_robot, v, tracks, cfg, prev);
}

// 반경 합 = 0.361 + 0.30 + 0.25
constexpr double kR = 0.911;
// x = 8 에서 +y 로 건너는 보행자 (y = −4): 통로 반폭은 R + 4·sin(0.1) ≈ 1.31 → 구간 x ≈ 6.7 ~ 9.3
GateTrack crosser()
{
  return walker(1, 8.0, -4.0, 0.0, 1.0);
}
// 같은 차선을 1.5 m 앞에서 건너기 시작한 보행자: 경로에 0.59 s 면 닿으므로 노출 구간이 로봇의
// 도착 시각과 무관하게 x ≈ 6.94 ~ 9.06 (반폭 0.911 + 1.5·sin 0.1) 으로 고정된다
GateTrack nearCrosser()
{
  return walker(1, 8.0, -1.5, 0.0, 1.0);
}
}  // namespace

TEST(CrossingGate, DefaultsPinTheSimulatedConstants)
{
  // 시뮬레이션(접촉 0/610)에서 쓴 상수 그대로 — 바꾸면 시뮬레이션과 다른 정책이다
  GateConfig c;
  EXPECT_FALSE(c.enable);              // 기본 꺼짐: 기존 거동 유지
  EXPECT_NEAR(c.hold_back, 1.2, 1e-12);
  EXPECT_NEAR(c.delay, 0.3, 1e-12);
  EXPECT_NEAR(c.pos_err, 0.25, 1e-12);
  EXPECT_NEAR(c.vknown, 0.3, 1e-12);
  EXPECT_NEAR(c.turn_time, 1.57, 1e-12);
  EXPECT_NEAR(c.walk_accel, 0.6, 1e-12);
  EXPECT_NEAR(c.v_max, 1.0, 1e-12);
  EXPECT_FALSE(DwaConfig().gate.enable);
}

TEST(CrossingGate, ReversalReturnTimeStopsTurnsAndReaccelerates)
{
  const GateConfig c = gateCfg();
  // 1.0 m/s 로 멀어지는 사람이 지금 자리로 돌아오기: 정지 1.667 s(0.833 m) + 회전 1.57 s +
  // 0.833 m 재가속 1.667 s = 4.90 s
  EXPECT_NEAR(
    reversalReturnTime(0.0, 1.0, c), 1.0 / 0.6 + 1.57 + std::sqrt(2.0 * 0.833333 / 0.6), 1e-3);
  // 2.0 m 뒤: 되돌아갈 거리 2.833 > 가속 거리 0.833 → 1.667 + 2.0/1.0 = 3.667 → 합 6.90 s
  EXPECT_NEAR(reversalReturnTime(2.0, 1.0, c), 1.0 / 0.6 + 1.57 + 1.0 / 0.6 + 2.0, 1e-3);
  // 이미 서 있으면 회전 + 재가속뿐
  EXPECT_NEAR(reversalReturnTime(0.0, 0.0, c), 1.57, 1e-9);
  // 속력은 v_max 로 포화 (크기를 믿지 않는다)
  EXPECT_NEAR(reversalReturnTime(0.0, 5.0, c), reversalReturnTime(0.0, 1.0, c), 1e-9);
}

TEST(CrossingGate, ReachabilityUsesOnlyTheHeadingSign)
{
  const GateConfig c = gateCfg();
  const GateTrack t = walker(1, 0.0, 0.0, 1.0, 0.0, 0.3);   // 느리게(0.3) +x 로
  GateReason why = GateReason::kNone;
  // 앞 3 m: 속력 0.3 이어도 v_max 1.0 으로 본다 → 2.089 s 면 닿는다
  EXPECT_TRUE(canReach(t, {3.0, 0.0}, 2.1, c, &why));
  EXPECT_EQ(why, GateReason::kForward);
  EXPECT_FALSE(canReach(t, {3.0, 0.0}, 2.0, c));
  // 뒤 3 m: 되돌아옴 — 정지 0.5 s + 회전 1.57 + 재가속 (2.089 + 0.075 m → 3.0 s) ≈ 5.07 s
  EXPECT_FALSE(canReach(t, {-3.0, 0.0}, 4.5, c));
  EXPECT_TRUE(canReach(t, {-3.0, 0.0}, 5.5, c, &why));
  EXPECT_EQ(why, GateReason::kReversal);
  // 진행선 통로 밖(가로 2.0 m > 0.911 + 3·sin 0.1)은 아무리 기다려도 못 닿는다
  EXPECT_FALSE(canReach(t, {3.0, 2.0}, 100.0, c));
  // 옆으로 비껴 앞(|α| ≤ R, 통로 안)은 바로 닿는다 — 속력 0 이어도
  EXPECT_TRUE(canReach(t, {0.5, 0.8}, 0.0, c, &why));
  EXPECT_EQ(why, GateReason::kForward);
  // 반경 합 안이면 이미 닿아 있다
  EXPECT_TRUE(canReach(t, {0.3, 0.5}, 0.0, c, &why));
  EXPECT_EQ(why, GateReason::kTouching);
}

TEST(CrossingGate, UnknownHeadingIsADiskEitherWay)
{
  const GateConfig c = gateCfg();
  const GateTrack t = stander(1, 0.0, 0.0);
  GateReason why = GateReason::kNone;
  for (const double x : {3.0, -3.0}) {
    EXPECT_TRUE(canReach(t, {x, 0.0}, 2.1, c, &why));
    EXPECT_EQ(why, GateReason::kUnknownHeading);
    EXPECT_FALSE(canReach(t, {x, 0.0}, 2.0, c));
  }
  EXPECT_TRUE(canReach(t, {0.0, 2.5}, 1.6, c));   // 옆으로도
}

TEST(CrossingGate, ExposureFollowsTheCrossingLineAndHoldsBeforeIt)
{
  const GateConfig c = gateCfg();
  const GateResult r = gate({crosser()}, 0.0, 1.0, c);
  EXPECT_EQ(r.phase, GatePhase::kApproach);
  EXPECT_EQ(r.reason, GateReason::kForward);
  EXPECT_EQ(r.n_threats, 1);
  ASSERT_EQ(r.exempt_ids.size(), 1u);   // 접근 중에도 닿을 수 있는 트랙은 게이트 몫 (VO·TTC₀ 면제)
  EXPECT_EQ(r.exempt_ids.front(), 1);
  EXPECT_NEAR(r.s_in, 8.0 - (kR + 4.0 * std::sin(0.1)), 0.10);
  EXPECT_NEAR(r.s_out, 8.0 + (kR + 4.0 * std::sin(0.1)), 0.10);
  EXPECT_NEAR(r.s_hold, r.s_in - c.hold_back, 1e-9);
  EXPECT_GT(r.speed_limit, c.robot_v_max);   // 정지점이 멀어 아직 감속 없음
  EXPECT_TRUE(r.next.has_hold);
  EXPECT_NEAR(r.next.hold_point.x, r.s_hold, 1e-9);
  EXPECT_TRUE(r.next.has_exit);
  EXPECT_NEAR(r.next.exit_point.x, r.s_out, 1e-9);
  // 창 길이 = 지금 상태에서 구간 출구까지의 주행 시간
  EXPECT_NEAR(r.window_s, travelTime(r.s_out, 1.0, 1.0, 1.0), 1e-9);
  // 보행자가 없으면 열림
  const GateResult none = gate({}, 0.0, 1.0, c);
  EXPECT_EQ(none.phase, GatePhase::kOpen);
  EXPECT_EQ(none.n_threats, 0);
}

TEST(CrossingGate, HoldPointAvoidsSpotsAnotherTrackCanReachWhileWaiting)
{
  const GateConfig c = gateCfg();
  const double s_hold_alone = gate({crosser()}, 0.0, 1.0, c).s_hold;
  // 두 번째 보행자: x = 4.3 에서 −y 로 8 m 밖 — 경로에는 7.1 s 뒤 닿아 지금 지나가는 로봇과는
  // 충돌하지 않지만, 정지점(5.5, 0)에서 **기다리는** 로봇은 닿는다 (도착 5.8 s + 대기 지평 3 s)
  const GateResult r = gate({crosser(), walker(2, 4.3, 8.0, 0.0, -1.0)}, 0.0, 1.0, c);
  EXPECT_EQ(r.n_threats, 1);                 // 두 번째는 지금 지나가면 못 닿는다
  EXPECT_NEAR(s_hold_alone, 8.0 - (kR + 4.0 * std::sin(0.1)) - 1.2, 0.10);
  // 정지점은 두 번째 보행자의 진행선 통로(|x − 4.3| ≤ 0.911 + 8·sin 0.1 = 1.71) **밖**으로 물러난다
  // —
  // 시간이 남는다고 통로 안에 서면 기다리는 동안 걸어 들어온다 (포팅 시뮬레이션 B→A 62/610 접촉)
  EXPECT_LT(r.s_hold, 4.3 - (kR + 8.0 * std::sin(0.1)) + 0.06);
  EXPECT_GT(r.s_hold, 4.3 - (kR + 8.0 * std::sin(0.1)) - 0.20);
  EXPECT_LT(r.s_hold, s_hold_alone);
  // 진행선이 정지점을 지나가는 것은 시간과 무관: 같은 배치의 보행자를 14 m 밖에 두어도 배제된다
  // (통로 반폭이 거리에 비례해 0.911 + 14·sin 0.1 = 2.31 로 자라므로 정지점은 그만큼 더 뒤)
  const GateResult far2 = gate({crosser(), walker(2, 4.3, 14.0, 0.0, -1.0)}, 0.0, 1.0, c);
  EXPECT_LT(far2.s_hold, 4.3 - (kR + 14.0 * std::sin(0.1)) + 0.06);
  EXPECT_GT(far2.s_hold, 4.3 - (kR + 14.0 * std::sin(0.1)) - 0.20);
  // 같은 방향으로 뒤따라오는 보행자도 같다: 그 통로(y = 0.5, 앞 전부)가 경로를 덮으면 설 자리가
  // 창 시작까지 밀린다 — 경로가 그 진행선으로 모여드는 곳에 서면 따라잡힌다. 서 있으면 그 자리,
  // 달리는 중이면 지금 최대 감속으로 설 수 있는 자리(제동 거리)가 정지점이 된다
  const std::vector<GateTrack> follow{crosser(), walker(3, -5.0, 0.5, 1.0, 0.0)};
  EXPECT_NEAR(gate(follow, 0.0, 0.0, c).s_hold, 0.0, 1e-9);
  const GateResult behind = gate(follow, 0.0, 1.0, c);
  EXPECT_NEAR(
    behind.s_hold, SpeedProfile::stoppingDistance(1.0, c.decel, c.jerk, c.latency), 1e-9);
  EXPECT_LT(behind.s_hold, s_hold_alone);
  EXPECT_NEAR(behind.speed_limit, 0.0, 1e-9);
  // 경로를 이미 지나 멀어지는 보행자의 뒤도 되돌아올 수 있는 만큼은 배제된다 (x = 5.5, y = −1.0, −y
  // 로)
  const GateResult left = gate({crosser(), walker(4, 5.5, -1.0, 0.0, -1.0)}, 0.0, 1.0, c);
  EXPECT_LT(left.s_hold, 5.5 - kR + 0.06);
  EXPECT_TRUE(
    amr_navigation::core::waitingSpotThreatened(walker(4, 5.5, -1.0, 0.0, -1.0), {5.5, 0.0}, c));
  EXPECT_FALSE(
    amr_navigation::core::waitingSpotThreatened(walker(4, 5.5, -9.0, 0.0, -1.0), {5.5, 0.0}, c));
}

TEST(CrossingGate, ApproachLimitReachesZeroAtTheHoldPoint)
{
  const GateConfig c = gateCfg();
  const double s_hold = gate({nearCrosser()}, 0.0, 1.0, c).s_hold;
  EXPECT_NEAR(s_hold, 8.0 - (kR + 1.5 * std::sin(0.1)) - 1.2, 0.10);
  // 정지점 0.8 m 앞, 0.8 m/s: 상한 = 정지거리 역함수 (0.5 < 상한 < 1.0)
  const GateResult far = gate({nearCrosser()}, s_hold - 0.8, 0.8, c);
  EXPECT_EQ(far.phase, GatePhase::kApproach);
  EXPECT_NEAR(far.s_hold, s_hold, 1e-9);
  EXPECT_NEAR(
    far.speed_limit, SpeedProfile::maxSpeedForStop(0.8, c.decel, c.jerk, c.latency), 1e-9);
  EXPECT_LT(far.speed_limit, c.robot_v_max);
  EXPECT_GT(far.speed_limit, 0.5);
  // 정지점 5 cm 앞, 0.3 m/s: 설 수 있다 → 상한이 거의 0
  const GateResult near = gate({nearCrosser()}, s_hold - 0.05, 0.3, c);
  EXPECT_EQ(near.phase, GatePhase::kApproach);
  EXPECT_LT(near.speed_limit, 0.15);
  // 정지점에 서 있으면 hold: v = 0, w = 0
  const GateResult hold = gate({nearCrosser()}, s_hold, 0.0, c);
  EXPECT_EQ(hold.phase, GatePhase::kHold);
  EXPECT_TRUE(hold.hard_stop);
  EXPECT_NEAR(hold.speed_limit, 0.0, 1e-12);
  EXPECT_EQ(hold.next.phase, GatePhase::kHold);
}

TEST(CrossingGate, CannotStopBeforeTheHoldPointBrakesNowAndCommitsOnlyInsideTheExposure)
{
  const GateConfig c = gateCfg();
  const GateResult ref = gate({nearCrosser()}, 0.0, 1.0, c);
  const double s_hold = ref.s_hold;
  const double d_stop = SpeedProfile::stoppingDistance(1.0, c.decel, c.jerk, c.latency);
  // 정지점 + 허용 오차 안에 못 서지만 노출 구간 시작 앞에는 선다 → 지금 최대 감속 (상한 0), 접근
  // 유지
  const GateResult brake = gate({nearCrosser()}, s_hold + c.hold_tol - d_stop + 0.05, 1.0, c);
  EXPECT_EQ(brake.phase, GatePhase::kApproach);
  EXPECT_NEAR(brake.speed_limit, 0.0, 1e-12);
  EXPECT_LT(brake.s_hold, ref.s_in);
  EXPECT_FALSE(brake.next.forced);
  // 지금 서는 자리가 노출 구간 안이라도 **몸으로는** 진행선 밖(x = 8 에서 1.0 m > 0.661)이면 선다
  const GateResult clear = gate({nearCrosser()}, ref.s_in - d_stop + 0.05, 1.0, c);
  EXPECT_EQ(clear.phase, GatePhase::kApproach);
  EXPECT_NEAR(clear.speed_limit, 0.0, 1e-12);
  // 지금 서는 자리가 노출 구간 안이고 몸으로도 진행선 위(x = 8 에서 0.61 m), 출구까지 1.7 m → 강제
  // 커밋. 면제 없음 (VO/TTC 는 예전대로), 상한 없음
  const GateResult r = gate({nearCrosser()}, 8.0 - 0.611 - d_stop, 1.0, c);
  EXPECT_EQ(r.phase, GatePhase::kCommitted);
  // 출구가 멀면(나란한 구간) 강제 커밋 대신 지금 선다 — 달려도 못 벗어난다
  GateConfig short_run = c;
  short_run.commit_max_run = 1.0;
  const GateResult stop = gate({nearCrosser()}, 8.0 - 0.611 - d_stop, 1.0, short_run);
  EXPECT_EQ(stop.phase, GatePhase::kApproach);
  EXPECT_NEAR(stop.speed_limit, 0.0, 1e-12);
  EXPECT_EQ(r.reason, GateReason::kForcedCommit);
  EXPECT_TRUE(r.exempt_ids.empty());
  EXPECT_TRUE(r.next.forced);
  EXPECT_FALSE(r.hard_stop);
  EXPECT_FALSE(std::isfinite(r.speed_limit));
  EXPECT_TRUE(r.next.has_exit);
  // 정지점 앞에 설 수 있으면 접근 (상한 = 정지거리 역함수)
  const GateResult ok = gate({nearCrosser()}, s_hold + c.hold_tol - d_stop - 0.05, 1.0, c);
  EXPECT_EQ(ok.phase, GatePhase::kApproach);
  EXPECT_GT(ok.speed_limit, 0.5);
  // 서 있는 로봇은 절대 강제 커밋되지 않는다 (재판정만 한다)
  EXPECT_EQ(gate({nearCrosser()}, s_hold + 0.5, 0.0, c).phase, GatePhase::kHold);
}

TEST(CrossingGate, DecidedCommitExemptsTheTracksItJudgedUntilTheExit)
{
  const GateConfig c = gateCfg();
  const GateResult hold = gate({nearCrosser()}, 5.5, 0.0, c);
  ASSERT_EQ(hold.phase, GatePhase::kHold);
  // 보행자가 경로를 지나 멀어진다 (y = −2, −y 로): 되돌아와도 로봇이 먼저 지난다 → 커밋
  GateTrack passed = walker(1, 8.0, -2.0, 0.0, -1.0);
  // 12 m 밖에 선 사람: 창 끝(x 12, 7.8 s)까지도 원판이 못 닿는다
  GateTrack far_stander = stander(9, 8.0, 12.0);
  const GateResult go = gate({passed, far_stander}, 5.5, 0.0, c, hold.next);
  EXPECT_EQ(go.phase, GatePhase::kCommitted);
  EXPECT_EQ(go.reason, GateReason::kDecided);
  EXPECT_FALSE(go.next.forced);
  ASSERT_EQ(go.exempt_ids.size(), 2u);            // 판정에 들어간 트랙 전부
  EXPECT_NE(std::find(go.exempt_ids.begin(), go.exempt_ids.end(), 1), go.exempt_ids.end());
  EXPECT_NE(std::find(go.exempt_ids.begin(), go.exempt_ids.end(), 9), go.exempt_ids.end());
  EXPECT_TRUE(go.next.has_exit);
  EXPECT_NEAR(go.next.exit_point.x, hold.s_out, 1e-9);
  // 커밋 중: 판정한 트랙(id 1)이 다시 위협으로 보여도 개입하지 않고 면제를 유지한다
  const GateResult mid = gate({crosser()}, 7.0, 1.0, c, go.next);
  EXPECT_EQ(mid.phase, GatePhase::kCommitted);
  EXPECT_EQ(mid.exempt_ids, go.exempt_ids);
  EXPECT_FALSE(mid.hard_stop);
  EXPECT_FALSE(std::isfinite(mid.speed_limit));
  // 커밋 뒤에 **나타난** 트랙(id 5, 판정에 없던)은 다시 판정한다 — 면제는 유지한 채 정지점 논리로
  GateTrack newcomer = walker(5, 10.5, -4.0, 0.0, 1.0);   // x = 10.5 를 건너온다: 6.0 m 앞
  const GateResult seen = gate({crosser(), newcomer}, 5.8, 0.0, c, go.next);
  EXPECT_NE(seen.phase, GatePhase::kCommitted);
  EXPECT_TRUE(std::isfinite(seen.s_hold));
  EXPECT_NE(std::find(seen.exempt_ids.begin(), seen.exempt_ids.end(), 1), seen.exempt_ids.end());
  EXPECT_NE(
    std::find(seen.next.exempt_ids.begin(), seen.next.exempt_ids.end(), 1),
    seen.next.exempt_ids.end());
  // 출구를 지나면 조우 종료 → 열림, 상태 초기화
  const GateResult done = gate({}, hold.s_out + c.exit_tol + 0.05, 1.0, c, go.next);
  EXPECT_EQ(done.phase, GatePhase::kOpen);
  EXPECT_TRUE(done.exempt_ids.empty());
  EXPECT_EQ(done.next.phase, GatePhase::kOpen);
  EXPECT_FALSE(done.next.has_exit);
}

TEST(CrossingGate, ReversalThreatKeepsTheRobotWaitingOnlyWhenItCanReturnInTime)
{
  GateConfig c = gateCfg();
  // 경로를 0.59 m 지나 멀어지는 보행자 (x = 8, y = −1.5, −y): 돌아오는 데 ≈ 5.4 s
  const GateTrack leaving = walker(1, 8.0, -1.5, 0.0, -1.0);
  // 8 m 앞 정지 로봇: x = 8 도착 8.8 s → 되돌아오면 닿는다 → 위협
  const GateResult far = gate({leaving}, 0.0, 0.0, c);
  EXPECT_EQ(far.phase, GatePhase::kApproach);
  EXPECT_EQ(far.reason, GateReason::kReversal);
  // 2.5 m 앞: 도착 3.3 s < 5.4 → 위협 아님 → 열림
  EXPECT_EQ(gate({leaving}, 5.5, 0.0, c).phase, GatePhase::kOpen);
  // 되돌아옴 모델을 끄면(회전 무한) 8 m 앞에서도 열림 — 시뮬레이션에서 이 모델이 없으면 39/610 접촉
  c.turn_time = 1e9;
  EXPECT_EQ(gate({leaving}, 0.0, 0.0, c).phase, GatePhase::kOpen);
}

TEST(CrossingGate, StaticTimeoutIgnoresLongStationaryTracks)
{
  const GateConfig c = gateCfg();
  // 경로 옆 0.5 m 에 방향 없이 선 트랙: 6 s 넘게 서 있으면 정적 (코스트맵 몫) → 게이트는 열림
  EXPECT_EQ(gate({stander(1, 6.0, 0.5, 6.0)}, 0.0, 1.0, c).phase, GatePhase::kOpen);
  // 방금 선 트랙(반환점의 보행자)은 원판 위협: 어느 쪽으로든 걸을 수 있어 기다리는 동안(10 s)
  // 닿는 자리에는 서지 않는다 → 창 안에 설 자리가 없어 지금 최대 감속 (상한 0)
  const GateResult r = gate({stander(1, 6.0, 0.5, 1.0)}, 0.0, 1.0, c);
  EXPECT_EQ(r.phase, GatePhase::kApproach);
  EXPECT_EQ(r.reason, GateReason::kUnknownHeading);
  EXPECT_NEAR(r.speed_limit, 0.0, 1e-12);
  EXPECT_TRUE(
    amr_navigation::core::waitingSpotThreatened(stander(1, 6.0, 0.5, 1.0), {0.5, 0.0}, c));
  EXPECT_FALSE(
    amr_navigation::core::waitingSpotThreatened(stander(1, 6.0, 12.0, 1.0), {0.5, 0.0}, c));
  // 걷기 시작해 방향이 잡히면 진행선 통로만 배제된다 (+x 로 걷는 사람 옆 0.5 m 는 통로 안)
  EXPECT_TRUE(
    amr_navigation::core::waitingSpotThreatened(walker(1, 6.0, 0.5, 1.0, 0.0), {7.0, 0.0}, c));
  EXPECT_FALSE(
    amr_navigation::core::waitingSpotThreatened(walker(1, 6.0, 0.5, 1.0, 0.0), {6.0, 3.0}, c));
}

TEST(CrossingGate, RetreatsAlongThePathOnlyWhenThatGainsDistance)
{
  const GateConfig c = gateCfg();
  const GateResult hold = gate({nearCrosser()}, 5.5, 0.0, c);
  ASSERT_EQ(hold.phase, GatePhase::kHold);
  // 정지 자리 위(x = 5.5)로 −y 방향 보행자가 2 m 앞에서 온다: 3 s 안에 닿는다. 경로를 따라
  // 물러나면(−x) 그 진행선(x = 5.5)에서 멀어진다 → 후진
  const GateResult r =
    gate({nearCrosser(), walker(2, 5.5, 2.0, 0.0, -1.0)}, 5.5, 0.0, c, hold.next);
  EXPECT_EQ(r.phase, GatePhase::kRetreat);
  EXPECT_TRUE(r.retreat);
  EXPECT_FALSE(r.hard_stop);
  EXPECT_NEAR(r.retreat_speed, c.retreat_speed, 1e-12);
  EXPECT_EQ(r.reason, GateReason::kHoldThreatened);
  EXPECT_TRUE(r.next.has_retreat_origin);
  EXPECT_NEAR(r.next.retreat_origin.x, 5.5, 1e-9);
  // 경로를 따라 뒤에서 오는 보행자(y = 0, +x): 물러나도 진행선까지 거리가 안 변한다 → 서 있는다
  const GateResult stay =
    gate({nearCrosser(), walker(3, 3.5, 0.0, 1.0, 0.0)}, 5.5, 0.0, c, hold.next);
  EXPECT_EQ(stay.phase, GatePhase::kHold);
  EXPECT_TRUE(stay.hard_stop);
  EXPECT_EQ(stay.reason, GateReason::kHoldThreatened);
  // 정지 자리가 **나중에 나타난** 보행자의 진행선 통로 안이면(6 m 밖이라 3 s 안에는 못 닿아도)
  // 물러난다 — 고를 때 피했을 자리에 서 있지 않는다 (B 출발 때 좁은 랙 뒤에서 나타나는 보행자)
  const GateResult early =
    gate({nearCrosser(), walker(6, 5.5, 6.0, 0.0, -1.0)}, 5.5, 0.0, c, hold.next);
  EXPECT_EQ(early.phase, GatePhase::kRetreat);
  EXPECT_FALSE(canReach(walker(6, 5.5, 6.0, 0.0, -1.0), {5.5, 0.0}, c.hold_threat_horizon, c));
  // 지나가서 멀어지는 보행자 뒤에서는 물러나지 않는다 (되돌아옴은 정지점 배제에서만) — 창 시작에
  // 붙어 있어도 마찬가지로 선다: 첫 계획의 시작점 뒤는 이탈이다
  const GateResult passed_by =
    gate({nearCrosser(), walker(8, 5.5, -1.5, 0.0, -1.0)}, 5.5, 0.0, c, hold.next);
  EXPECT_EQ(passed_by.phase, GatePhase::kHold);
  const GateResult at_start = gate({walker(9, 0.0, 2.0, 0.0, -1.0)}, 0.0, 0.0, c, hold.next);
  EXPECT_EQ(at_start.phase, GatePhase::kHold);
  // 후진 상한: 이미 retreat_max 만큼 물러났으면 선다
  GateState worn = hold.next;
  worn.phase = GatePhase::kRetreat;
  worn.has_retreat_origin = true;
  worn.retreat_origin = {5.5 + c.retreat_max, 0.0};
  const GateResult capped =
    gate({nearCrosser(), walker(2, 5.5, 2.0, 0.0, -1.0)}, 5.5, 0.0, c, worn);
  EXPECT_EQ(capped.phase, GatePhase::kHold);
  // 후진 기능을 끄면 서 있는다
  GateConfig off = c;
  off.retreat_enable = false;
  EXPECT_EQ(
    gate({nearCrosser(), walker(2, 5.5, 2.0, 0.0, -1.0)}, 5.5, 0.0, off, hold.next).phase,
    GatePhase::kHold);
}

// ---- DwaPlanner::compute 통합 계약 ----------------------------------------------------------
namespace
{
DwaConfig gateDwaConfig()
{
  DwaConfig c;
  c.vth_samples = 31;
  c.weights.oscillation = 0.5;
  c.robot_radius = 0.361;
  c.gate.enable = true;
  return c;
}

DwaInput inputAt(const std::vector<Pose2D> * path, double x, double v)
{
  DwaInput in;
  in.pose = {x, 0.0, 0.0};
  in.v_meas = v;
  in.w_meas = 0.0;
  in.has_last = true;
  in.v_last = v;
  in.w_last = 0.0;
  in.path = path;
  return in;
}

DwaResult run(const DwaPlanner & dwa, const DwaInput & in)
{
  return dwa.compute(in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
}

// 진짜 보행자 상태 → 게이트 트랙 (제어기의 변위 규칙과 같이 |v| ≥ 0.3 이면 방향을 안다)
GateTrack toGateTrack(const DynamicObstacle & o)
{
  GateTrack t;
  t.id = o.id;
  t.x = o.x;
  t.y = o.y;
  t.radius = o.radius;
  const double su = o.speed();
  t.heading_known = su >= 0.3;
  if (t.heading_known) {
    t.ux = o.vx / su;
    t.uy = o.vy / su;
  }
  t.speed = su;
  return t;
}

double rectClearance(const Pose2D & p, const DynamicObstacle & o)
{
  const double c = std::cos(p.theta);
  const double s = std::sin(p.theta);
  const double dx = o.x - p.x;
  const double dy = o.y - p.y;
  const double lx = c * dx + s * dy;
  const double ly = -s * dx + c * dy;
  return std::hypot(std::max(0.0, std::abs(lx) - 0.30), std::max(0.0, std::abs(ly) - 0.20)) -
         o.radius;
}
}  // namespace

TEST(CrossingGate, HoldOutputsZeroVelocityEvenWhenVoSaturates)
{
  const DwaPlanner dwa(gateDwaConfig());
  const auto path = straightPath();
  DwaInput in = inputAt(&path, 5.5, 0.0);
  // 정면 1.2 m 에서 마주 오는 장애물 → VO 포화·탈출 이득이 있어도
  DynamicObstacle head_on{6.7, 0.0, -1.0, 0.0, 0.30};
  head_on.id = 5;
  in.obstacles = {head_on};
  in.gate_tracks = {crosser(), toGateTrack(head_on)};
  const DwaResult r = run(dwa, in);
  EXPECT_EQ(r.gate.phase, GatePhase::kHold);
  EXPECT_TRUE(r.found);
  EXPECT_NEAR(r.v, 0.0, 1e-12);
  EXPECT_NEAR(r.w, 0.0, 1e-12);
  EXPECT_EQ(r.yield.state, YieldState::kYield);   // 기록용 투영: 정지선
  EXPECT_NEAR(r.yield.stop_distance, 0.0, 0.31);
}

TEST(CrossingGate, CommittedDisablesYieldEscapeAndVoForJudgedTracks)
{
  const DwaPlanner dwa(gateDwaConfig());
  const auto path = straightPath();
  // 판정 커밋 상태: 출구 9.3 m, 면제 id 7
  GateState st;
  st.phase = GatePhase::kCommitted;
  st.has_exit = true;
  st.exit_point = {9.3, 0.0};
  st.exempt_ids = {7};
  DynamicObstacle o{8.0, -1.0, 0.0, 1.0, 0.30};    // 바로 앞을 가로지르는 장애물 (VO 가 막는다)
  o.id = 7;
  DwaInput in = inputAt(&path, 7.0, 0.8);
  in.obstacles = {o};
  in.gate_tracks = {toGateTrack(o)};
  in.gate_state = st;
  const DwaResult r = run(dwa, in);
  EXPECT_EQ(r.gate.phase, GatePhase::kCommitted);
  EXPECT_EQ(r.gate.reason, GateReason::kDecided);
  EXPECT_EQ(r.yield.decision, amr_navigation::core::YieldDecision::kNone);
  EXPECT_FALSE(std::isfinite(r.yield.speed_limit));      // 양보 상한 없음
  EXPECT_NEAR(r.v_cap, 1.0, 1e-12);
  EXPECT_GE(r.window.v_lo, 0.0);                         // 후진 탈출 창이 열리지 않는다
  EXPECT_EQ(r.n_vo_rejected, 0u);                        // 판정한 트랙은 VO 면제
  EXPECT_FALSE(r.vo_saturated);
  EXPECT_GE(r.v, 0.8 - 1e-9);                            // 감속하지 않는다
  EXPECT_NEAR(r.best.terms.escape, 0.0, 1e-12);
  EXPECT_NEAR(r.best.terms.dynamic, 0.0, 1e-12);         // TTC₀ 벌점도 면제
  // 강제 커밋(면제 없음)이면 같은 장애물을 VO 가 그대로 본다
  st.forced = true;
  st.exempt_ids.clear();
  in.gate_state = st;
  const DwaResult forced = run(dwa, in);
  EXPECT_EQ(forced.gate.phase, GatePhase::kCommitted);
  EXPECT_EQ(forced.gate.reason, GateReason::kForcedCommit);
  EXPECT_GT(forced.n_vo_rejected, 0u);
  EXPECT_GE(forced.window.v_lo, 0.0);                    // 그래도 탈출 창은 열리지 않는다
}

TEST(CrossingGate, DisabledGateLeavesTheYieldPathUntouched)
{
  DwaConfig c = gateDwaConfig();
  c.gate.enable = false;
  const DwaPlanner dwa(c);
  const auto path = straightPath();
  DynamicObstacle o{5.0, 5.0, 0.0, -1.0, 0.25};   // x = 5 차선을 −y 로 (crossing_yield 와 같다)
  DwaInput in = inputAt(&path, 0.0, 1.0);
  in.obstacles = {o};
  in.gate_tracks = {toGateTrack(o)};   // 넣어도 무시된다
  const DwaResult r = run(dwa, in);
  EXPECT_EQ(r.gate.phase, GatePhase::kOpen);
  EXPECT_EQ(r.yield.state, YieldState::kYield);   // 예전 통로 정지선이 그대로 선다
  EXPECT_TRUE(std::isfinite(r.yield.stop_distance));
  // 게이트가 켜지면 통로 정지선 대신 게이트 정지점이다 (같은 배치에서 교차 구간이 같은 자리)
  const DwaPlanner gated(gateDwaConfig());
  const DwaResult g = run(gated, in);
  EXPECT_EQ(g.gate.phase, GatePhase::kApproach);
  EXPECT_EQ(g.yield.state, YieldState::kYield);
  EXPECT_NEAR(g.yield.stop_distance, g.gate.s_hold, 1e-9);
}

TEST(CrossingGate, ClosedLoopWaitsOutsideThenCrossesWithoutLeavingThePath)
{
  // 이상 플랜트(명령 즉시 반영, 20 Hz): 로봇은 (0,0) 에서 +x 12 m, 보행자는 x = 8 을 +y → −y 로
  // 0.6 m/s 로 건넌다 (y = 4.5 에서 출발: 경로에 6 s 뒤 도달 — 게이트는 속력을 믿지 않고 1.0 m/s 로
  // 보므로 출발 순간부터 위협이다). 게이트: 정지점(≈5.5)에서 기다리다 보행자가 지나 되돌아와도
  // 늦을 때 건넌다 — 접촉 없음, 정지 중 (0, 0), 경로 이탈 0, 끝까지 간다.
  const DwaPlanner dwa(gateDwaConfig());
  const auto path = straightPath();
  const auto cum = cumulativeLength(path);
  DynamicObstacle o{8.0, 4.5, 0.0, -0.6, 0.30};
  o.id = 1;
  DwaInput in = inputAt(&path, 0.0, 0.0);
  double min_gap = 1e9, max_cte = 0.0, hold_s = 0.0, s_end = 0.0;
  bool contact = false, moved_while_holding = false, committed_seen = false;
  double s_at_commit = -1.0;
  for (int k = 0; k < 400; ++k) {
    in.obstacles = {o};
    in.gate_tracks = {toGateTrack(o)};
    const DwaResult r = run(dwa, in);
    in.gate_state = r.gate.next;
    if (std::getenv("GATE_DEBUG") != nullptr && k % 4 == 0) {
      std::printf(
        "  k %3d x %.2f y %.3f th %.3f | v %.2f w %.3f cap %.2f | phase %d reason %d s_hold %.2f "
        "s_in %.2f | walker (%.2f, %.2f) vo %zu\n", k, in.pose.x, in.pose.y, in.pose.theta, r.v,
        r.w, r.v_cap,
        static_cast<int>(r.gate.phase), static_cast<int>(r.gate.reason), r.gate.s_hold, r.gate.s_in,
        o.x, o.y, r.n_vo_rejected);
    }
    if (r.gate.phase == GatePhase::kHold) {
      hold_s += 0.05;
      if (std::abs(r.v) > 1e-9 || std::abs(r.w) > 1e-9) {
        moved_while_holding = true;
      }
    }
    if (r.gate.phase == GatePhase::kCommitted && !committed_seen) {
      committed_seen = true;
      s_at_commit = in.pose.x;
    }
    in.pose = integrateArc(in.pose, r.v, r.w, 0.05);
    in.v_meas = in.v_last = r.v;
    in.w_meas = in.w_last = r.w;
    o.x += o.vx * 0.05;
    o.y += o.vy * 0.05;
    const double gap = rectClearance(in.pose, o);
    min_gap = std::min(min_gap, gap);
    contact = contact || gap < 0.0;
    const auto pr = amr_navigation::core::projectOntoPath(path, {in.pose.x, in.pose.y}, 0, 0, &cum);
    max_cte = std::max(max_cte, std::abs(pr.cte));
    s_end = pr.s;
    if (s_end > 11.5) {
      break;
    }
  }
  std::printf(
    "[ info ] gate closed loop: min clearance %.3f m, hold %.2f s, commit at x %.2f, max cte %.3f, "
    "s_end %.2f\n", min_gap, hold_s, s_at_commit, max_cte, s_end);
  EXPECT_FALSE(contact);
  EXPECT_GT(min_gap, 0.30);                 // e-stop 거리 이상
  EXPECT_GT(hold_s, 0.5);                   // 실제로 기다렸다
  EXPECT_FALSE(moved_while_holding);        // 정지 중 v = 0, w = 0
  EXPECT_TRUE(committed_seen);
  EXPECT_LT(s_at_commit, 8.0 - 0.911);      // 노출 구간 밖에서 결정했다
  EXPECT_LT(max_cte, 0.02);                 // 경로를 떠나지 않는다 (이탈 0)
  EXPECT_GT(s_end, 11.5);                   // 끝까지 간다 (교착 없음)
}
