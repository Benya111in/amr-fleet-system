// DWA 코어 단위 테스트: 동적 창, 샘플링, 궤적 시뮬레이션 정확성, 정지거리, 비용 순위, VO 샘플 제외.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/pid.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"
#include "aisle_scene.hpp"

using amr_navigation::core::DwaConfig;
using amr_navigation::core::DwaInput;
using amr_navigation::core::DwaPlanner;
using amr_navigation::core::DwaResult;
using amr_navigation::core::DwaWindow;
using amr_navigation::core::DynamicObstacle;
using amr_navigation::core::FootprintChecker;
using amr_navigation::core::OwnedGrid;
using amr_navigation::core::Pose2D;
using amr_navigation::core::inflate;
using amr_navigation::core::inflationCost;
using amr_navigation::core::integrateArc;
using amr_navigation::core::kLethalObstacle;
using amr_navigation::core::kPi;

namespace
{
std::vector<Pose2D> straightPath(double x0, double y, double len, double theta = 0.0)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    p.push_back({x0 + s * std::cos(theta), y + s * std::sin(theta), theta});
  }
  return p;
}

// 12 x 12 m 로컬 코스트맵 (원점 −6,−6), 선택적 장애물 사각형
struct Scene
{
  OwnedGrid grid{240, 240, 0.05, 0, -6.0, -6.0};
  std::vector<amr_navigation::core::Point2D> fp = FootprintChecker::rectangle(0.60, 0.40);

  void finalize() {inflate(grid, 0.2, 0.8, 3.0);}
  DwaResult run(const DwaPlanner & dwa, const DwaInput & in) const
  {
    const FootprintChecker chk(
      grid.view(), fp, inflationCost(FootprintChecker::circumscribedRadius(fp), 0.2, 3.0));
    const auto view = grid.view();
    return dwa.compute(
      in, [&chk](const Pose2D & p) {return chk.cost(p);},
      [view](double x, double y) {return static_cast<double>(view.costAtWorld(x, y));});
  }
};

DwaInput movingInput(const std::vector<Pose2D> * path, double v, double w = 0.0)
{
  DwaInput in;
  in.pose = {0.0, 0.0, 0.0};
  in.v_meas = v;
  in.w_meas = w;
  in.has_last = true;
  in.v_last = v;
  in.w_last = w;
  in.path = path;
  return in;
}
}  // namespace

TEST(Dwa, DynamicWindowFromLimits)
{
  DwaPlanner dwa;   // a = 1.0, α = 2.0, Δt = 0.05
  DwaWindow w = dwa.dynamicWindow(0.5, 0.2, 10.0);
  EXPECT_NEAR(w.v_lo, 0.45, 1e-12);
  EXPECT_NEAR(w.v_hi, 0.55, 1e-12);
  EXPECT_NEAR(w.w_lo, 0.10, 1e-12);
  EXPECT_NEAR(w.w_hi, 0.30, 1e-12);
  // 속도 한계에서 잘림
  w = dwa.dynamicWindow(0.98, 1.45, 10.0);
  EXPECT_NEAR(w.v_hi, 1.0, 1e-12);
  EXPECT_NEAR(w.w_hi, 1.5, 1e-12);
  // 정지 상태: 후진 불가(min_vel_x 0) → 하한 0
  w = dwa.dynamicWindow(0.0, 0.0, 10.0);
  EXPECT_NEAR(w.v_lo, 0.0, 1e-12);
  EXPECT_NEAR(w.v_hi, 0.05, 1e-12);
  // 외부 속도 제한이 창보다 낮음 → 최대 감속만
  w = dwa.dynamicWindow(0.5, 0.0, 0.2);
  EXPECT_NEAR(w.v_lo, 0.45, 1e-12);
  EXPECT_NEAR(w.v_hi, 0.45, 1e-12);
  // 후진 한계 밖(외부 요인) → 한계 쪽으로만
  // 정지 후 측정 잡음으로 아주 조금 음수: 창은 [0, v_c + a·Δt] (0 에 갇히지 않는다)
  w = dwa.dynamicWindow(-1e-4, 0.0, 10.0);
  EXPECT_NEAR(w.v_lo, 0.0, 1e-12);
  EXPECT_NEAR(w.v_hi, 0.05 - 1e-4, 1e-12);
  // 허용 범위 한참 아래 (후진 복구 직후): 한 주기에 닿는 만큼만 범위 쪽으로 한 점
  w = dwa.dynamicWindow(-0.3, 0.0, 10.0);
  EXPECT_NEAR(w.v_lo, -0.25, 1e-12);
  EXPECT_NEAR(w.v_hi, -0.25, 1e-12);
  // 각속도 중심이 한계 밖 → 한계 값 한 점
  w = dwa.dynamicWindow(0.5, 3.0, 10.0);
  EXPECT_NEAR(w.w_lo, 1.5, 1e-12);
  EXPECT_NEAR(w.w_hi, 1.5, 1e-12);
}

TEST(Dwa, WindowCenterRule)
{
  DwaPlanner dwa;
  DwaInput in;
  in.v_meas = 0.4;
  in.w_meas = 0.0;
  in.has_last = true;
  in.v_last = 0.6;
  in.w_last = 0.1;
  auto c = dwa.windowCenter(in);   // 차 0.2 ≤ 0.3 → 직전 명령
  EXPECT_DOUBLE_EQ(c.first, 0.6);
  EXPECT_DOUBLE_EQ(c.second, 0.1);
  in.v_last = 0.8;                 // 차 0.4 > 0.3 → 측정값
  c = dwa.windowCenter(in);
  EXPECT_DOUBLE_EQ(c.first, 0.4);
  in.has_last = false;
  c = dwa.windowCenter(in);
  EXPECT_DOUBLE_EQ(c.first, 0.4);
}

TEST(Dwa, SamplingGridZeroColumnAndBrake)
{
  DwaPlanner dwa;
  const DwaWindow win = dwa.dynamicWindow(0.5, 0.0, 10.0);
  const auto s = dwa.sampleVelocities(win, 0.5, 0.0);
  const auto & cfg = dwa.config();
  ASSERT_EQ(s.size(), static_cast<std::size_t>(cfg.vx_samples * cfg.vth_samples + 1));
  // ω = 0 열 포함, 모든 샘플이 창 안
  EXPECT_TRUE(std::any_of(s.begin(), s.end(), [](auto p) {return std::abs(p.second) < 1e-9;}));
  for (const auto & p : s) {
    EXPECT_GE(p.first, win.v_lo - 1e-12);
    EXPECT_LE(p.first, win.v_hi + 1e-12);
    EXPECT_GE(p.second, win.w_lo - 1e-12);
    EXPECT_LE(p.second, win.w_hi + 1e-12);
  }
  // 제동 후보 = 마지막: 최대 감속
  EXPECT_NEAR(s.back().first, 0.45, 1e-12);
  EXPECT_NEAR(s.back().second, 0.0, 1e-12);
  // ω 창이 0 을 포함하지 않으면 열을 추가하지 않는다; 제동 후보는 ω 를 0 쪽으로
  const DwaWindow turn = dwa.dynamicWindow(0.5, 0.6, 10.0);
  const auto st = dwa.sampleVelocities(turn, 0.5, 0.6);
  EXPECT_EQ(st.size(), static_cast<std::size_t>(cfg.vx_samples * cfg.vth_samples + 1));
  EXPECT_NEAR(st.back().second, 0.5, 1e-12);
  // 짝수 개 ω 샘플이면 0 이 격자에 없으므로 따로 추가된다
  DwaConfig c2;
  c2.vth_samples = 4;
  DwaPlanner even(c2);
  const auto se = even.sampleVelocities(win, 0.5, 0.0);
  EXPECT_EQ(se.size(), static_cast<std::size_t>(c2.vx_samples * 5 + 1));
  // 창 폭이 0 이면 샘플 1 개씩
  const DwaWindow point{0.3, 0.3, 0.0, 0.0};
  EXPECT_EQ(dwa.sampleVelocities(point, 0.3, 0.0).size(), 2U);
}

TEST(Dwa, RolloutIsExactArc)
{
  DwaPlanner dwa;
  const Pose2D start{1.0, 2.0, 0.3};
  const auto poses = dwa.rollout(start, 0.8, 0.5, 2.0);
  ASSERT_EQ(poses.size(), 21U);   // 2.0 / 0.1 + 1
  for (std::size_t k = 0; k < poses.size(); ++k) {
    const Pose2D e = integrateArc(start, 0.8, 0.5, 0.1 * static_cast<double>(k));
    EXPECT_NEAR(poses[k].x, e.x, 1e-12);
    EXPECT_NEAR(poses[k].y, e.y, 1e-12);
    // 모든 점이 반지름 v/ω = 1.6 원 위
    const double cx = start.x - 1.6 * std::sin(start.theta);
    const double cy = start.y + 1.6 * std::cos(start.theta);
    EXPECT_NEAR(std::hypot(poses[k].x - cx, poses[k].y - cy), 1.6, 1e-9);
  }
  EXPECT_NEAR(dwa.simTime(0.0), 1.5, 1e-12);
  EXPECT_NEAR(dwa.simTime(1.5), 2.0, 1e-12);
  EXPECT_NEAR(dwa.simTime(-3.0), 2.5, 1e-12);
}

TEST(Dwa, JerkLimitedStoppingDistance)
{
  // 연구 브리프 local-planning §3.2 표 (t_r = 0.15, a = 1, j = 2): v=1.0 → 0.889 m, v=2.0 → 2.789 m
  EXPECT_NEAR(DwaPlanner::stoppingDistance(1.0, 1.0, 2.0, 0.15), 0.889, 1e-3);
  EXPECT_NEAR(DwaPlanner::stoppingDistance(2.0, 1.0, 2.0, 0.15), 2.789, 1e-3);
  EXPECT_NEAR(DwaPlanner::stoppingDistance(0.5, 1.0, 2.0, 0.15), 0.315, 1e-3);
  // 램프 도중 정지 (v ≤ a²/2j = 0.25)
  EXPECT_NEAR(DwaPlanner::stoppingDistance(0.2, 1.0, 2.0, 0.15), 0.0896, 1e-4);
  // 저크 무시(사다리꼴) = v t + v²/2a
  EXPECT_NEAR(DwaPlanner::stoppingDistance(1.0, 1.0, 0.0, 0.15), 0.65, 1e-12);
  EXPECT_DOUBLE_EQ(DwaPlanner::stoppingDistance(0.0, 1.0, 2.0, 0.15), 0.0);
  EXPECT_TRUE(std::isinf(DwaPlanner::stoppingDistance(1.0, 0.0, 2.0, 0.15)));
}

TEST(Dwa, FreeSpaceGoesStraightAndAccelerates)
{
  Scene sc;
  sc.finalize();
  const auto path = straightPath(0.0, 0.0, 5.0);
  DwaPlanner dwa;
  const DwaResult r = sc.run(dwa, movingInput(&path, 0.5));
  ASSERT_TRUE(r.found);
  EXPECT_NEAR(r.v, 0.55, 1e-9);    // 창 상한
  EXPECT_NEAR(r.w, 0.0, 1e-9);
  EXPECT_EQ(r.n_collision, 0U);
  EXPECT_EQ(r.n_valid, r.n_samples);
  EXPECT_FALSE(r.best.poses.empty());
}

TEST(Dwa, SteersBackTowardPath)
{
  Scene sc;
  sc.finalize();
  const auto path = straightPath(0.0, -0.3, 5.0);   // 경로가 로봇 오른쪽 0.3 m
  DwaPlanner dwa;
  const DwaResult r = sc.run(dwa, movingInput(&path, 0.5));
  ASSERT_TRUE(r.found);
  EXPECT_LT(r.w, 0.0);    // 우회전
  // 비용 순위: 직진 샘플보다 선택된 샘플의 경로 항이 작다
  EXPECT_GT(r.best.terms.path, 0.0);
}

TEST(Dwa, AvoidsObstacleOnPath)
{
  Scene sc;
  sc.grid.fillRect(0.8, -0.1, 1.1, 0.4, kLethalObstacle);   // 경로 위 상자 (왼쪽으로 치우침)
  sc.finalize();
  const auto path = straightPath(0.0, 0.0, 5.0);
  DwaConfig cfg;
  cfg.vth_samples = 31;
  DwaPlanner dwa(cfg);
  // 넓은 각속도 창: 직전 명령 ω = −0.6 (우회전 중)
  const DwaResult r = sc.run(dwa, movingInput(&path, 0.6, -0.6));
  ASSERT_TRUE(r.found);
  EXPECT_GT(r.n_collision, 0U);
  EXPECT_FALSE(r.best.collision);
  EXPECT_LT(r.w, 0.0);   // 장애물의 오른쪽(열린 쪽)으로 계속 돈다
}

TEST(Dwa, AllCollidingReturnsBrake)
{
  Scene sc;
  sc.grid.fillRect(0.45, -3.0, 0.8, 3.0, kLethalObstacle);   // 정면 벽
  sc.finalize();
  const auto path = straightPath(0.0, 0.0, 5.0);
  DwaPlanner dwa;
  const DwaResult r = sc.run(dwa, movingInput(&path, 0.5));
  EXPECT_FALSE(r.found);
  EXPECT_EQ(r.n_collision, r.n_samples);
  EXPECT_NEAR(r.v, 0.45, 1e-12);   // 제동 후보
  EXPECT_TRUE(r.best.is_brake);
}

TEST(Dwa, GoalApproachAndAlignment)
{
  Scene sc;
  sc.finalize();
  // 목표 0.6 m 앞 → v_des = √(2·1·0.6) ≈ 1.1 > 창 → 가속 유지, 목표 0.1 m 앞 → 정렬 모드(v_des = 0)
  auto path = straightPath(0.0, 0.0, 0.1);
  path.back().theta = kPi / 2.0;   // 목표 방향 +90°
  DwaPlanner dwa;
  DwaInput in = movingInput(&path, 0.0);
  const DwaResult r = sc.run(dwa, in);
  ASSERT_TRUE(r.found);
  EXPECT_NEAR(r.d_goal, 0.1, 1e-9);
  EXPECT_NEAR(r.v, 0.0, 1e-12);    // 정렬 중에는 전진 안 함
  EXPECT_GT(r.w, 0.0);             // 좌회전으로 목표 방향 정렬
  // 목표 너머 충돌은 무시: 목표(0.3 m) 뒤 벽(x ≥ 0.75)이 있어도 중심이 목표+여유(0.4 m)까지만 검사
  Scene wall;
  wall.grid.fillRect(0.75, -3.0, 1.0, 3.0, kLethalObstacle);
  wall.finalize();
  const auto path2 = straightPath(0.0, 0.0, 0.3);
  const DwaResult r2 = wall.run(dwa, movingInput(&path2, 0.3));
  EXPECT_TRUE(r2.found);
  EXPECT_EQ(r2.n_collision, 0U);
  // 경로 끝이 목표가 아니면(부분 경로) 전체 롤아웃을 검사 → 벽과 충돌하는 샘플이 생긴다
  DwaInput partial = movingInput(&path2, 0.3);
  partial.path_end_is_goal = false;
  EXPECT_GT(wall.run(dwa, partial).n_collision, 0U);
}

TEST(Dwa, VelocityObstacleRejectsFastCrossing)
{
  Scene sc;
  sc.finalize();
  const auto path = straightPath(0.0, 0.0, 6.0);
  DwaConfig cfg;
  cfg.robot_radius = 0.35;
  cfg.vo_margin = 0.4;     // R = 0.35 + 0.25 + 0.4 = 1.0
  DwaPlanner dwa(cfg);
  DwaInput in = movingInput(&path, 0.5);
  // 왼쪽 앞 (2, 2) 에서 −y 로 1 m/s 횡단: 상대 운동으로 τ = 2 s 뒤 (2 − 2v, 0) → v > 0.5 면 VO 안
  in.obstacles.push_back(DynamicObstacle{2.0, 2.0, 0.0, -1.0, 0.25});
  const DwaResult r = sc.run(dwa, in);
  ASSERT_TRUE(r.found);
  EXPECT_GT(r.n_vo_rejected, 0U);
  EXPECT_FALSE(r.vo_saturated);
  EXPECT_FALSE(r.best.vo_rejected);
  EXPECT_LE(r.v, 0.5 + 1e-9);
  EXPECT_TRUE(std::isfinite(r.best.ttc) || r.best.terms.dynamic == 0.0);
  // VO 끄면 제외 없음 → 더 빠른 샘플도 선택 가능
  cfg.use_velocity_obstacles = false;
  DwaPlanner no_vo(cfg);
  const DwaResult r2 = sc.run(no_vo, in);
  EXPECT_EQ(r2.n_vo_rejected, 0U);
  // 느린 트랙(정적 취급)은 무시
  in.obstacles[0].vy = -0.1;
  const DwaResult r3 = sc.run(dwa, in);
  EXPECT_EQ(r3.n_vo_rejected, 0U);
}

TEST(Dwa, VelocityObstacleSaturationKeepsVoAndBrakes)
{
  Scene sc;
  sc.finalize();
  const auto path = straightPath(0.0, 0.0, 6.0);
  DwaPlanner dwa;
  DwaInput in = movingInput(&path, 0.8);
  // 정면 3 m 에서 −1 m/s 로 마주 옴: 좁은 창(±0.05 m/s)의 모든 샘플이 VO 안 (포화)
  in.obstacles.push_back(DynamicObstacle{3.0, 0.0, -1.0, 0.0, 0.25});
  const DwaResult r = sc.run(dwa, in);
  EXPECT_TRUE(r.found);
  EXPECT_TRUE(r.vo_saturated);
  EXPECT_EQ(r.n_vo_rejected, r.n_valid);
  // VO 를 끄지 않는다: VO 진입이 가장 늦은 샘플 = 창의 최저 속도 (가속하지 않는다)
  EXPECT_NEAR(r.v, 0.75, 1e-9);
  EXPECT_TRUE(std::isfinite(r.best.vo_time));
  for (const auto & o : {0.80, 0.85}) {
    DwaInput faster = in;
    faster.v_meas = faster.v_last = o;
    EXPECT_LT(sc.run(dwa, faster).v, o);
  }
  // TTC₀ 비용: 예측 접촉 전에 설 수 없는 속도 초과분이 걸린다
  EXPECT_GT(r.best.terms.dynamic, 0.0);
  EXPECT_TRUE(std::isfinite(r.best.ttc));
}

TEST(Dwa, TtcCostSlowsBeforeVoSaturates)
{
  // 옆 4 m 앞에서 가로지를 장애물이 VO(R, τ 2 s) 밖이지만 TTC₀(R_d, 5 s) 안: 과속분 비용이
  // 속도 항을 이겨 창의 최고 속도가 아니라 감속 쪽을 고른다
  // (리뷰: 기존 1 − TTC/T 항은 이웃 샘플 간 차가 속도 항보다 작아 가속)
  Scene sc;
  sc.finalize();
  const auto path = straightPath(0.0, 0.0, 8.0);
  DwaPlanner dwa;
  DwaInput in = movingInput(&path, 1.0);
  in.obstacles.push_back(DynamicObstacle{3.6, 2.4, 0.0, -1.0, 0.25});
  const DwaResult r = sc.run(dwa, in);
  ASSERT_TRUE(r.found);
  EXPECT_FALSE(r.vo_saturated);
  EXPECT_LT(r.v, 1.0);
  DwaConfig off;
  off.weights.dynamic = 0.0;
  off.use_velocity_obstacles = false;
  EXPECT_NEAR(sc.run(DwaPlanner(off), in).v, 1.0, 1e-9);   // 동적 항이 없으면 최고 속도 유지
}

namespace
{
struct LoopResult
{
  double min_center{1e9};       // 중심 거리 최소 [m]
  double min_clearance{1e9};    // 로봇 사각형(0.60 x 0.40) ↔ 장애물 원판 최소 여유 [m] (< 0 접촉)
  double v_at_closest{0.0};
  double v_max{0.0};              // 최근접 전까지의 최고 속도 (지나간 뒤 재가속은 허용)
  bool saturated_seen{false};
};

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

// 이상 플랜트(명령 즉시 반영, 20 Hz) 폐루프: 트랙만 보고(코스트맵에는 없음) 등속 장애물을 만난다.
// nav2_params.yaml 의 DWA 값 (기본 DwaConfig 와 같음 + 진동 0.5, 원 반경 0.361)
LoopResult closedLoop(DynamicObstacle o, double v0, double seconds)
{
  DwaConfig c;
  c.vth_samples = 31;
  c.weights.oscillation = 0.5;
  c.goal_align_distance = 0.08;
  c.robot_radius = 0.361;
  const DwaPlanner dwa(c);
  const auto path = straightPath(0.0, 0.0, 12.0);
  DwaInput in = movingInput(&path, v0);
  LoopResult out;
  for (int k = 0; k < static_cast<int>(seconds / 0.05); ++k) {
    in.obstacles = {o};
    const DwaResult r = dwa.compute(
      in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
    out.saturated_seen |= r.vo_saturated;
    in.pose = integrateArc(in.pose, r.v, r.w, 0.05);
    in.v_meas = in.v_last = r.v;
    in.w_meas = in.w_last = r.w;
    o.x += o.vx * 0.05;
    o.y += o.vy * 0.05;
    const double d = std::hypot(o.x - in.pose.x, o.y - in.pose.y);
    const bool approaching = (o.x - in.pose.x) * (o.vx - r.v * std::cos(in.pose.theta)) +
      (o.y - in.pose.y) * (o.vy - r.v * std::sin(in.pose.theta)) < 0.0;
    if (d < out.min_center) {
      out.min_center = d;
      out.v_at_closest = r.v;
    }
    out.min_clearance = std::min(out.min_clearance, rectClearance(in.pose, o));
    if (approaching) {
      out.v_max = std::max(out.v_max, r.v);
    }
  }
  return out;
}
}  // namespace

TEST(Dwa, ClosedLoopCrossingKeepsVoRadius)
{
  // 1.0 m/s 횡단 장애물 (명세 4.7). 리뷰 하네스: 기존 VO 폴백은 가속해 중심 거리 0.35 m (접촉).
  // VO 반경 R = 0.361 + 0.25 + 0.3 = 0.911 을 지킨다 (실측: TTC₀ 반경 R_d 1.111 까지 유지).
  struct Case {DynamicObstacle o; double v0;};
  for (const Case & k :
    {Case{{2.0, 1.5, 0.0, -1.0, 0.25}, 1.0}, Case{{4.0, 3.2, 0.0, -1.0, 0.25}, 1.0},
      Case{{4.0, 3.2, 0.0, -0.5, 0.25}, 1.0}, Case{{3.0, -2.5, 0.0, 1.0, 0.25}, 0.8}})
  {
    const LoopResult r = closedLoop(k.o, k.v0, 6.0);
    std::printf(
      "[ info ] crossing from (%.1f, %.1f) v_obs %.1f: min centre %.3f m, clearance %.3f m, "
      "v at closest %.2f, saturated %d\n", k.o.x, k.o.y, k.o.vy, r.min_center, r.min_clearance,
      r.v_at_closest, r.saturated_seen);
    EXPECT_GT(r.min_center, 0.911 - 0.03);
    EXPECT_GT(r.min_clearance, 0.30);   // 사각형 기준으로도 e-stop 거리 이상
  }
}

TEST(Dwa, ClosedLoopHeadOnYields)
{
  // 정면 −1 m/s 로 마주 오는 장애물은 후진 없이(min_vel_x 0) 피할 수 없다
  // (장애물이 비키지 않으면 접촉).
  // 요구: 가속하지 않고, 장애물이 가까워지기 전에 멈춰 선다 (접근 속도 = 장애물 속도뿐).
  for (const double d0 : {3.0, 2.0}) {
    const LoopResult r = closedLoop({d0, 0.0, -1.0, 0.0, 0.25}, 0.8, 3.0);
    std::printf(
      "[ info ] head-on from %.1f m: min centre %.3f m, v at closest %.2f, v max %.2f\n", d0,
      r.min_center, r.v_at_closest, r.v_max);
    EXPECT_TRUE(r.saturated_seen);
    EXPECT_LE(r.v_max, 0.8 + 1e-9);
    EXPECT_LT(r.v_at_closest, 0.05);
  }
  // 옆으로 0.7 m 비껴 오는 정면 장애물: 감속·회피로 접촉 없이 지나간다
  const LoopResult side = closedLoop({3.0, 0.7, -1.0, 0.0, 0.25}, 0.8, 4.0);
  std::printf(
    "[ info ] head-on offset 0.7 m: min centre %.3f m, clearance %.3f m\n", side.min_center,
    side.min_clearance);
  EXPECT_GT(side.min_clearance, 0.0);
}

namespace
{
struct AisleRun
{
  bool traversed{false};
  double min_clearance{1e9};     // 참값 로봇 사각형 ↔ 랙
  double min_speed_inside{1e9};  // x ∈ [3.2, 6.0] 의 명령 속도 최소
  int no_valid{0};               // 유효 샘플 없음 주기 (통로 안)
  int cycles_inside{0};
};

// σ 0.03 LiDAR 로 매 스캔 새로 만든 코스트맵으로 0.60 m 통로를 1.0 m/s 관통 (이상 플랜트, 20 Hz).
// path_offset: 위치추정 오차로 경로가 통로 중심에서 옆으로 비낀 양.
// dynamics = true: 명령 → VelocityProfiler(50 Hz, 저크 2) → 서보 (지연 0.04 + 1차 0.08 s)
//   → 운동학 (tracking_sim 과 같은 체인)
AisleRun dwaThroughAisle(
  double resolution, bool denoise, double path_offset, unsigned seed,
  const Pose2D & start = {0.3, 0.0, 0.0}, double v0 = 0.0, bool recenter = true,
  bool dynamics = false)
{
  aisle::Scene sc;
  sc.resolution = resolution;
  sc.denoise = denoise;
  sc.rng.seed(seed);
  DwaConfig c;
  c.vth_samples = 31;
  c.weights.oscillation = 0.5;
  c.goal_align_distance = 0.08;
  c.robot_radius = 0.361;
  c.recenter_narrow = recenter;
  c.recenter_step = resolution;
  const DwaPlanner dwa(c);
  const auto path = straightPath(0.0, path_offset, 8.3);
  DwaInput in;
  in.pose = start;
  in.v_meas = in.v_last = v0;
  in.has_last = v0 > 0.0;
  in.path = &path;
  amr_navigation::core::VelocityProfiler prof;
  prof.reset(v0, 0.0);
  amr_navigation::core::FirstOrderPlant sv(1.0, 0.08, 0.04, 0.01, v0);
  amr_navigation::core::FirstOrderPlant sw(1.0, 0.08, 0.04, 0.01, 0.0);
  double pv = v0;
  double pw = 0.0;
  AisleRun out;
  for (int k = 0; k < 900; ++k) {
    if (k % 2 == 0) {
      sc.observe(in.pose);   // 10 Hz 스캔
    }
    const auto chk = sc.checker();
    const auto view = sc.grid.view();
    const DwaResult r = dwa.compute(
      in, [&chk](const Pose2D & p) {return chk.cost(p);},
      [view](double x, double y) {return static_cast<double>(view.costAtWorld(x, y));});
    const bool inside = in.pose.x > 3.0 && in.pose.x < 7.0;
    if (inside) {
      ++out.cycles_inside;
      out.no_valid += r.found ? 0 : 1;
    }
    if (in.pose.x > 3.2 && in.pose.x < 6.0) {
      out.min_speed_inside = std::min(out.min_speed_inside, r.v);
    }
    if (dynamics) {
      for (int sub = 0; sub < 5; ++sub) {   // 100 Hz 적분, 50 Hz 프로파일러
        if (sub % 2 == 0) {
          const auto o = prof.update(r.v, r.w, true, sv.output(), sw.output(), 0.02);
          pv = o.v;
          pw = o.w;
        }
        const double vv = sv.step(pv, 0.01);
        const double ww = sw.step(pw, 0.01);
        in.pose = integrateArc(in.pose, vv, ww, 0.01);
        out.min_clearance = std::min(out.min_clearance, aisle::trueClearance(in.pose));
      }
      in.v_meas = sv.output();
      in.w_meas = sw.output();
    } else {
      in.pose = integrateArc(in.pose, r.v, r.w, 0.05);
      in.v_meas = r.v;
      in.w_meas = r.w;
    }
    in.v_last = r.v;
    in.w_last = r.w;
    in.has_last = true;
    out.min_clearance = std::min(out.min_clearance, aisle::trueClearance(in.pose));
    if (in.pose.x > 8.0 && std::abs(r.v) < 1e-3 && std::abs(in.v_meas) < 0.01) {
      out.traversed = true;
      break;
    }
  }
  return out;
}
}  // namespace

TEST(Dwa, NarrowAisleWithLidarNoise)
{
  // 명세 4.4 "로봇 폭 + 20 cm (0.60 m) 통로 충돌 없이".
  // 리뷰 실측: σ 0.03 끝점이 벽 안쪽 0.05 m 이상에
  // 매 스캔 찍혀 통로 안 치명 셀 26–51 개, DWA 샘플 88–92 % 충돌, 속도 0.35 m/s.
  // 채택 설정 (0.025 m 격자 + 게이트 중앙값): 멈춤 없이 1.0 m/s 로 지나가고 참값 여유를 지킨다.
  for (const double offset : {0.0, 0.02}) {
    for (const unsigned seed : {1U, 2U, 3U}) {
      const AisleRun r = dwaThroughAisle(0.025, true, offset, seed);
      std::printf(
        "[ info ] DWA aisle 0.025 m + denoise, path offset %.2f seed %u: traversed %d, "
        "min clearance %.3f m, min v inside %.2f, no-valid %d/%d\n",
        offset, seed, r.traversed, r.min_clearance,
        r.min_speed_inside, r.no_valid,
        r.cycles_inside);
      EXPECT_TRUE(r.traversed);
      EXPECT_GT(r.min_clearance, 0.04);
      EXPECT_GT(r.min_speed_inside, 0.8);
      EXPECT_EQ(r.no_valid, 0);
    }
  }
  // Gazebo 실측 실패 재현 (aisle_fix_nosafety_r1 lap 13: 남쪽 회전점이 입구 1.2 m 앞, 회전 뒤
  // 9 cm 옆·8° 틀어진 채 0.3 m/s, 전역 경로는 위치추정 편향으로 통로 중심에서 3.5 cm): 재중심이
  // 없으면 입구에서 유효 샘플 0 으로 끼여 멈췄다. 재중심은 지역 코스트맵의 통로 골 바닥을 따라
  // 들어간다.
  for (const Pose2D & bad_start : {Pose2D{1.8, -0.09, 0.14}, Pose2D{1.8, -0.09, -0.14}}) {
    for (const unsigned seed : {1U, 2U, 3U}) {
      const AisleRun on = dwaThroughAisle(0.025, true, -0.035, seed, bad_start, 0.3, true, true);
      const AisleRun off = dwaThroughAisle(0.025, true, -0.035, seed, bad_start, 0.3, false, true);
      std::printf(
        "[ info ] DWA aisle misaligned entry (heading %+.2f) seed %u: recenter on traversed %d "
        "clearance %.3f no-valid %d | off traversed %d clearance %.3f no-valid %d\n",
        bad_start.theta, seed,
        on.traversed,
        on.min_clearance,
        on.no_valid, off.traversed, off.min_clearance,
        off.no_valid);
      EXPECT_TRUE(on.traversed);
      EXPECT_GT(on.min_clearance, 0.04);
      EXPECT_EQ(on.no_valid, 0);
    }
  }
  // 이전 설정 (0.05 m 격자, 원 끝점): 같은 장면에서 멈추거나 크게 느려진다 (문제 재현)
  int degraded = 0;
  for (const unsigned seed : {1U, 2U, 3U}) {
    const AisleRun r = dwaThroughAisle(0.05, false, 0.0, seed);
    std::printf(
      "[ info ] DWA aisle 0.05 m raw marks seed %u: traversed %d, min clearance %.3f m, "
      "min v inside %.2f, no-valid %d/%d\n",
      seed, r.traversed, r.min_clearance, r.min_speed_inside, r.no_valid,
      r.cycles_inside);
    degraded += !r.traversed || r.no_valid > 0 || r.min_speed_inside < 0.8;
  }
  EXPECT_EQ(degraded, 3);
}

TEST(Dwa, WindowRecoversFromTinyNegativeSpeed)
{
  // Gazebo 실측 (좁은 통로 왕복 lap 8): 제자리 회전 뒤 측정·직전 명령이 −1e-4 수준
  // → 이전 창은 [v_c, 0] 이라
  // 18 s 동안 v = 0 만 골랐다. 이제 매 주기 가속한다.
  Scene sc;
  sc.finalize();
  const auto path = straightPath(0.0, 0.0, 6.0);
  DwaPlanner dwa;
  DwaInput in = movingInput(&path, -1e-4);
  double v = in.v_last;
  for (int k = 0; k < 5; ++k) {   // 창 재설정(직전 명령 − 측정 > 0.3) 전까지
    const DwaResult r = sc.run(dwa, in);
    ASSERT_TRUE(r.found);
    EXPECT_GT(r.v, v);
    v = r.v;
    in.v_last = r.v;
    in.v_meas = -1e-4;   // 측정은 아직 음수 잡음 (창 중심 = 직전 명령, 차 < window_reset_v)
  }
  EXPECT_GT(v, 0.2);
}

TEST(Dwa, OscillationPenalty)
{
  Scene sc;
  sc.finalize();
  const auto path = straightPath(0.0, 0.0, 5.0, kPi);   // 경로가 뒤쪽 → 제자리 회전
  DwaPlanner dwa;
  DwaInput in = movingInput(&path, 0.0, 0.3);   // 직전에 좌회전 중
  const DwaResult r = sc.run(dwa, in);
  ASSERT_TRUE(r.found);
  EXPECT_GT(r.w, 0.0);   // 반전(우회전)하지 않는다
}

// S 자 경로(R 3 m, ±30°·60° 물결) 이상 추종 폐루프 (명령 즉시 반영, 20 Hz): 추종 항을 롤아웃 앞
// path_eval_time 만 보면 긴 등곡률 원호의 곡률 평균화(안쪽 가로지르기)가 줄어든다 (dwa.md §1.5).
double sCurveMeanCte(double path_eval_time)
{
  std::vector<Pose2D> path{{0.0, 0.0, 0.0}};
  for (const double a : {kPi / 6.0, -kPi / 3.0, kPi / 6.0, kPi / 6.0, -kPi / 3.0, kPi / 6.0}) {
    const Pose2D o = path.back();
    const int n = static_cast<int>(std::lround(std::abs(a) * 3.0 / 0.05));
    for (int i = 1; i <= n; ++i) {
      path.push_back(integrateArc(o, 1.0, (a > 0.0 ? 1.0 : -1.0) / 3.0, i * 0.05));
    }
  }
  DwaConfig c;
  c.path_eval_time = path_eval_time;
  const DwaPlanner dwa(c);
  const auto cum = amr_navigation::core::cumulativeLength(path);
  Pose2D pose = path.front();
  DwaInput in;
  in.path = &path;
  double sum = 0.0;
  int n = 0;
  for (int k = 0; k < 600; ++k) {
    const DwaResult r = dwa.compute(
      in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
    pose = integrateArc(pose, r.v, r.w, 0.05);
    in.pose = pose;
    in.v_meas = in.v_last = r.v;
    in.w_meas = in.w_last = r.w;
    in.has_last = true;
    const auto pr = amr_navigation::core::projectOntoPath(path, {pose.x, pose.y}, 0, 0, &cum);
    if (pr.s > cum.back() - 0.3) {
      break;
    }
    sum += std::abs(pr.cte);
    ++n;
  }
  EXPECT_GT(n, 100);
  return sum / std::max(1, n);
}

TEST(Dwa, ShortTrackingHorizonFollowsSCurve)
{
  const double full = sCurveMeanCte(0.0);
  const double prefix = sCurveMeanCte(0.8);
  std::printf("[ info ] S-curve mean CTE: full rollout %.3f m, first 0.8 s %.3f m\n", full, prefix);
  EXPECT_LT(prefix, 0.03);
  EXPECT_LT(prefix, full);
}

TEST(Dwa, CycleTimeBudget)
{
  Scene sc;
  sc.grid.fillRect(2.0, 0.6, 5.0, 1.2, kLethalObstacle);
  sc.grid.fillRect(2.0, -1.2, 5.0, -0.6, kLethalObstacle);
  sc.finalize();
  const auto path = straightPath(0.0, 0.0, 6.0);
  DwaConfig cfg;
  cfg.vx_samples = 11;
  cfg.vth_samples = 31;
  DwaPlanner dwa(cfg);
  DwaInput in = movingInput(&path, 0.8, 0.1);
  in.obstacles.push_back(DynamicObstacle{4.0, 3.0, 0.0, -1.0, 0.25});
  const int n = 50;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    const DwaResult r = sc.run(dwa, in);
    ASSERT_GT(r.n_samples, 300U);
  }
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count() / n;
  std::printf("[ info ] DWA 11x31 samples, 12x12 m costmap: %.2f ms/cycle\n", ms);
  EXPECT_LT(ms, 50.0);   // 20 Hz 주기(50 ms) 안
}
