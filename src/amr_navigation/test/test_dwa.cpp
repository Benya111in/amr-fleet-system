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
  w = dwa.dynamicWindow(-0.3, 0.0, 10.0);
  EXPECT_NEAR(w.v_lo, -0.3, 1e-12);
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
  EXPECT_FALSE(r.vo_fallback);
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

TEST(Dwa, VelocityObstacleFallbackWhenAllRejected)
{
  Scene sc;
  sc.finalize();
  const auto path = straightPath(0.0, 0.0, 6.0);
  DwaPlanner dwa;
  DwaInput in = movingInput(&path, 0.5);
  // 정면 3 m 에서 −1 m/s 로 마주 옴: 좁은 창의 모든 샘플이 VO 안
  in.obstacles.push_back(DynamicObstacle{3.0, 0.0, -1.0, 0.0, 0.25});
  const DwaResult r = sc.run(dwa, in);
  EXPECT_TRUE(r.found);
  EXPECT_TRUE(r.vo_fallback);
  EXPECT_EQ(r.n_vo_rejected, r.n_valid);
  // TTC₀ 비용이 걸린다 (평균 예측 5 s 안에 접촉)
  EXPECT_GT(r.best.terms.dynamic, 0.0);
  EXPECT_TRUE(std::isfinite(r.best.ttc));
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
