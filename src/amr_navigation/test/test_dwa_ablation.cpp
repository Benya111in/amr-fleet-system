// DWA 절제 스위치 시험 (docs/research/dwa-ablation/prereg.md).
//
// 1) 비트 동일 (golden): 절제 스위치를 더하기 **전** 코어(origin/main 2345e27)로 돌린
//    폐루프 명령열의 해시를 상수로 박아 두고, 기본값 설정이 지금도 같은 해시를 내는지
//    본다. 장면은 기본 동작의
//    주요 분기를 모두 지나게 골랐다: 체인(프로파일러 + 서보) S 자·U 턴 추종과 목표 접근·정렬,
//    LiDAR 잡음 코스트맵의 0.60 m 통로(재중심·여유 항·충돌 검사), 횡단 장애물(VO·TTC·양보),
//    배포 nav2_params 값.
//    DWA_GOLDEN_PRINT=1 이면 해시를 출력한다 (기준 코어에서 상수를 다시 뽑을 때만 쓴다).
#include <gtest/gtest.h>

#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/pid.hpp"
#include "amr_navigation/core/speed_profile.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"
#include "aisle_scene.hpp"

using amr_navigation::core::DwaConfig;
using amr_navigation::core::DwaInput;
using amr_navigation::core::DwaPlanner;
using amr_navigation::core::DwaResult;
using amr_navigation::core::DynamicObstacle;
using amr_navigation::core::Pose2D;
using amr_navigation::core::integrateArc;
using amr_navigation::core::kPi;

namespace
{
struct Hash
{
  std::uint64_t h{1469598103934665603ULL};
  void add(double x)
  {
    unsigned char b[sizeof(double)];
    std::memcpy(b, &x, sizeof(double));
    for (unsigned char c : b) {
      h ^= c;
      h *= 1099511628211ULL;
    }
  }
};

void appendStraight(std::vector<Pose2D> & p, double len)
{
  const Pose2D o = p.back();
  const int n = static_cast<int>(std::lround(len / 0.05));
  for (int i = 1; i <= n; ++i) {
    p.push_back(integrateArc(o, 1.0, 0.0, i * 0.05));
  }
}

void appendArc(std::vector<Pose2D> & p, double radius, double angle)
{
  const Pose2D o = p.back();
  const int n = static_cast<int>(std::lround(std::abs(angle) * radius / 0.05));
  const double sgn = angle >= 0.0 ? 1.0 : -1.0;
  for (int i = 1; i <= n; ++i) {
    p.push_back(integrateArc(o, 1.0, sgn / radius, i * 0.05));
  }
}

std::vector<Pose2D> sCurve()
{
  std::vector<Pose2D> c{{0.0, 0.0, 0.0}};
  for (int k = 0; k < 2; ++k) {
    appendArc(c, 3.0, kPi / 6.0);
    appendArc(c, 3.0, -kPi / 3.0);
    appendArc(c, 3.0, kPi / 6.0);
  }
  appendStraight(c, 1.0);
  return c;
}

std::vector<Pose2D> uTurn()
{
  std::vector<Pose2D> u{{0.0, 0.0, kPi / 2.0}};
  appendStraight(u, 3.0);
  appendArc(u, 2.0, -kPi);
  appendStraight(u, 3.0);
  return u;
}

// nav2_params.yaml DWA 블록과 같은 값 (플러그인이 읽는 것)
DwaConfig deployedConfig()
{
  DwaConfig c;
  c.vth_samples = 31;
  c.weights.oscillation = 0.5;
  c.goal_align_distance = 0.08;
  c.robot_radius = 0.361;
  c.off_path_band = 1.0;
  c.yield_corridor_margin = 0.7;
  c.yield_stop_margin = 0.8;
  c.yield_clear_margin = 2.0;
  c.yield_escape_speed = 0.5;
  c.yield_decision = false;
  return c;
}

// 체인 폐루프 (tracking_sim 과 같은 구조): 20 Hz 계획 → 50 Hz 프로파일러 → 100 Hz 서보
// (지연 0.04 + 1차 0.08 s) → 정확 원호. 목표 판정 xy 0.10 / yaw 0.05 뒤 명령 0.
std::uint64_t chainHash(const DwaConfig & cfg, const std::vector<Pose2D> & path, int cycles)
{
  const DwaPlanner dwa(cfg);
  amr_navigation::core::VelocityProfiler prof;
  amr_navigation::core::FirstOrderPlant sv(1.0, 0.08, 0.04, 0.01, 0.0);
  amr_navigation::core::FirstOrderPlant sw(1.0, 0.08, 0.04, 0.01, 0.0);
  DwaInput in;
  in.pose = path.front();
  in.path = &path;
  in.path_end_is_goal = true;
  Hash h;
  double pv = 0.0;
  double pw = 0.0;
  double vc = 0.0;
  double wc = 0.0;
  bool done = false;
  for (int k = 0; k < cycles; ++k) {
    if (!done) {
      const double dg = std::hypot(in.pose.x - path.back().x, in.pose.y - path.back().y);
      if (dg <= 0.10 &&
        std::abs(amr_navigation::core::wrapAngle(path.back().theta - in.pose.theta)) <= 0.05)
      {
        done = true;
        vc = 0.0;
        wc = 0.0;
      } else {
        const DwaResult r = dwa.compute(
          in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
        vc = r.v;
        wc = r.w;
        h.add(r.v);
        h.add(r.w);
        h.add(r.best.cost);
        in.v_last = r.v;
        in.w_last = r.w;
        in.has_last = true;
      }
    }
    for (int sub = 0; sub < 5; ++sub) {
      if (sub % 2 == 0) {
        const auto o = prof.update(vc, wc, true, sv.output(), sw.output(), 0.02);
        pv = o.v;
        pw = o.w;
      }
      in.pose = integrateArc(in.pose, sv.step(pv), sw.step(pw), 0.01);
    }
    in.v_meas = sv.output();
    in.w_meas = sw.output();
  }
  h.add(in.pose.x);
  h.add(in.pose.y);
  h.add(in.pose.theta);
  return h.h;
}

// σ 0.03 LiDAR 코스트맵 0.60 m 통로 (test_dwa.cpp dwaThroughAisle 의 체인판), 경로 편향 −0.035,
// 틀어진 진입 — 재중심·상대 여유·풋프린트 충돌 검사를 모두 지난다.
std::uint64_t aisleHash(const DwaConfig & base, unsigned seed)
{
  aisle::Scene sc;
  sc.resolution = 0.025;
  sc.denoise = true;
  sc.rng.seed(seed);
  DwaConfig c = base;
  c.recenter_step = 0.025;
  const DwaPlanner dwa(c);
  std::vector<Pose2D> path;
  for (double s = 0.0; s <= 8.3 + 1e-9; s += 0.05) {
    path.push_back({s, -0.035, 0.0});
  }
  DwaInput in;
  in.pose = {1.8, -0.09, 0.14};
  in.v_meas = in.v_last = 0.3;
  in.has_last = true;
  in.path = &path;
  amr_navigation::core::VelocityProfiler prof;
  prof.reset(0.3, 0.0);
  amr_navigation::core::FirstOrderPlant sv(1.0, 0.08, 0.04, 0.01, 0.3);
  amr_navigation::core::FirstOrderPlant sw(1.0, 0.08, 0.04, 0.01, 0.0);
  double pv = 0.3;
  double pw = 0.0;
  Hash h;
  for (int k = 0; k < 300; ++k) {
    if (k % 2 == 0) {
      sc.observe(in.pose);
    }
    const auto chk = sc.checker();
    const auto view = sc.grid.view();
    const DwaResult r = dwa.compute(
      in, [&chk](const Pose2D & p) {return chk.cost(p);},
      [view](double x, double y) {return static_cast<double>(view.costAtWorld(x, y));});
    h.add(r.v);
    h.add(r.w);
    h.add(static_cast<double>(r.n_valid));
    h.add(static_cast<double>(r.n_recentered));
    for (int sub = 0; sub < 5; ++sub) {
      if (sub % 2 == 0) {
        const auto o = prof.update(r.v, r.w, true, sv.output(), sw.output(), 0.02);
        pv = o.v;
        pw = o.w;
      }
      in.pose = integrateArc(in.pose, sv.step(pv), sw.step(pw), 0.01);
    }
    in.v_meas = sv.output();
    in.w_meas = sw.output();
    in.v_last = r.v;
    in.w_last = r.w;
  }
  h.add(in.pose.x);
  h.add(in.pose.y);
  return h.h;
}

// 이상 플랜트 횡단: 직선 경로 위 1.0 m/s 로봇 앞을 1.0 m/s 장애물이 가로지른다 (VO·TTC·양보).
std::uint64_t crossingHash(const DwaConfig & cfg, double ox, double oy, double ovy)
{
  const DwaPlanner dwa(cfg);
  std::vector<Pose2D> path;
  for (double s = 0.0; s <= 14.0 + 1e-9; s += 0.05) {
    path.push_back({s, 0.0, 0.0});
  }
  DwaInput in;
  in.pose = path.front();
  in.v_meas = in.v_last = 0.8;
  in.has_last = true;
  in.path = &path;
  DynamicObstacle o;
  o.x = ox;
  o.y = oy;
  o.vx = 0.0;
  o.vy = ovy;
  o.radius = 0.25;
  o.id = 3;
  Hash h;
  for (int k = 0; k < 200; ++k) {
    in.obstacles = {o};
    const DwaResult r = dwa.compute(
      in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
    h.add(r.v);
    h.add(r.w);
    h.add(r.vo_saturated ? 1.0 : 0.0);
    h.add(static_cast<double>(static_cast<int>(r.yield.state)));
    in.pose = integrateArc(in.pose, r.v, r.w, 0.05);
    in.v_meas = in.v_last = r.v;
    in.w_meas = in.w_last = r.w;
    in.yield_limit_last = r.yield.speed_limit;
    in.yield_decision_last = static_cast<int>(r.yield.decision);
    o.y += o.vy * 0.05;
  }
  h.add(in.pose.x);
  h.add(in.pose.y);
  return h.h;
}

bool printing()
{
  const char * e = std::getenv("DWA_GOLDEN_PRINT");
  return e != nullptr && e[0] == '1';
}

void check(const char * name, std::uint64_t got, std::uint64_t expected)
{
  if (printing()) {
    std::printf("GOLDEN %s 0x%016" PRIx64 "ULL\n", name, got);
  }
  EXPECT_EQ(got, expected) << name << " — 기본값 동작이 기준 코어(2345e27)와 달라졌다";
}
}  // namespace

// 기준 코어(origin/main 2345e27, 절제 스위치 추가 전)에서 뽑은 해시. 기본값이면 지금도 같아야 한다.
TEST(DwaAblation, DefaultsAreBitIdenticalToBaselineCore)
{
  DwaConfig core_default;
  core_default.vth_samples = 31;
  core_default.weights.oscillation = 0.5;
  core_default.goal_align_distance = 0.08;
  check(
    "scurve_chain_core", chainHash(core_default, sCurve(), 600), 0x1d3e87f7180083b8ULL);
  check(
    "uturn_chain_core", chainHash(core_default, uTurn(), 500), 0x4137d80ff58b300dULL);
  check(
    "scurve_chain_deployed", chainHash(deployedConfig(), sCurve(), 600), 0x1d3e87f7180083b8ULL);
  check(
    "uturn_chain_deployed", chainHash(deployedConfig(), uTurn(), 500), 0x4137d80ff58b300dULL);
  check("aisle_deployed_s1", aisleHash(deployedConfig(), 1U), 0x695bdc1efd370c9dULL);
  check("aisle_deployed_s2", aisleHash(deployedConfig(), 2U), 0x29e6e63078414420ULL);
  check("crossing_core", crossingHash(core_default, 4.0, 3.2, -1.0), 0x2e5db9e09019019dULL);
  check(
    "crossing_deployed", crossingHash(deployedConfig(), 3.0, 2.0, -1.0), 0x7c15561996e60a67ULL);
}

// ---------------------------------------------------------------------------------------------
// 2) 스위치마다 "끈 쪽" 이 정의대로 동작하는가

namespace
{
std::vector<Pose2D> straight(double len, double y = 0.0)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    p.push_back({s, y, 0.0});
  }
  return p;
}

DwaInput cruising(const std::vector<Pose2D> * path, double v)
{
  DwaInput in;
  in.pose = path->front();
  in.v_meas = v;
  in.w_meas = 0.0;
  in.has_last = true;
  in.v_last = v;
  in.w_last = 0.0;
  in.path = path;
  return in;
}

DwaResult emptyRun(const DwaPlanner & dwa, const DwaInput & in)
{
  return dwa.compute(in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
}
}  // namespace

TEST(DwaAblation, MeasuredWindowCenterIgnoresTheLastCommand)
{
  DwaConfig c;
  DwaInput in;
  in.has_last = true;
  in.v_last = 0.40;
  in.w_last = 0.10;
  in.v_meas = 0.30;
  in.w_meas = 0.05;
  const auto on = DwaPlanner(c).windowCenter(in);
  EXPECT_DOUBLE_EQ(on.first, 0.40);   // 배포: 직전 명령 (차 0.1 ≤ 재설정 0.3)
  c.window_center_measured = true;
  const auto off = DwaPlanner(c).windowCenter(in);
  EXPECT_DOUBLE_EQ(off.first, 0.30);
  EXPECT_DOUBLE_EQ(off.second, 0.05);
}

TEST(DwaAblation, NoBrakeCandidateMeansGridOnlyAndStopWhenBlocked)
{
  DwaConfig c;
  c.vth_samples = 31;
  const DwaPlanner on(c);
  c.brake_candidate = false;
  const DwaPlanner off(c);
  const auto win = on.dynamicWindow(0.5, 0.0, 1.0);
  EXPECT_EQ(
    on.sampleVelocities(win, 0.5, 0.0).size(), off.sampleVelocities(
      win, 0.5,
      0.0).size() + 1);
  // 모든 후보가 충돌: 배포는 제동 후보(0.45), 교과서는 정지 명령 (0, 0)
  const auto path = straight(6.0);
  const DwaInput in = cruising(&path, 0.5);
  auto blocked = [](const Pose2D &) {return -1.0;};
  auto zero = [](double, double) {return 0.0;};
  const DwaResult r_on = on.compute(in, blocked, zero);
  const DwaResult r_off = off.compute(in, blocked, zero);
  EXPECT_FALSE(r_on.found);
  EXPECT_NEAR(r_on.v, 0.45, 1e-9);
  EXPECT_FALSE(r_off.found);
  EXPECT_DOUBLE_EQ(r_off.v, 0.0);
  EXPECT_DOUBLE_EQ(r_off.w, 0.0);
}

TEST(DwaAblation, GoalApproachLawsSetTheDesiredSpeed)
{
  // 목표 0.30 m 앞, 0.5 m/s, 직선(헤딩 감속 cos = 1). 고른 후보의 속도 항 |v_des − v| / v_span 에서
  // v_des 를 되읽는다: 배포 = 저크·지연 역함수, 사다리꼴 = √(2·a·d), 없음 = v_cap.
  const auto path = straight(0.30);
  DwaConfig c;
  c.vth_samples = 31;
  c.goal_align_distance = 0.08;
  const DwaInput in = cruising(&path, 0.5);
  auto vdesOk = [&](int law, double v_des) {
      DwaConfig k = c;
      k.goal_approach = law;
      const DwaResult r = emptyRun(DwaPlanner(k), in);
      return std::abs(std::abs(v_des - r.best.v) - r.best.terms.velocity) < 1e-12;
    };
  const double v_jerk = amr_navigation::core::SpeedProfile::maxSpeedForStop(
    0.30, c.limits.decel_lim_x, c.limits.jerk_lim_x, c.approach_latency);
  EXPECT_LT(v_jerk, std::sqrt(2.0 * 0.30));
  EXPECT_TRUE(vdesOk(0, v_jerk));
  EXPECT_TRUE(vdesOk(2, std::sqrt(2.0 * c.limits.decel_lim_x * 0.30)));
  EXPECT_TRUE(vdesOk(1, c.limits.max_vel_x));
  EXPECT_FALSE(vdesOk(1, v_jerk));
}

TEST(DwaAblation, AbsoluteClearanceChargesTheCostOnThePathItself)
{
  // 경로가 비용 150 띠 안을 지난다 (좁은 통로 중앙과 같은 상황).
  // 상대 여유는 0, 절대 여유는 150/252.
  const auto path = straight(6.0);
  DwaConfig c;
  c.vth_samples = 31;
  auto band = [](double, double y) {return std::abs(y) < 0.5 ? 150.0 : 0.0;};
  auto fp = [](const Pose2D &) {return 0.0;};
  const DwaResult rel = DwaPlanner(c).compute(cruising(&path, 0.5), fp, band);
  c.relative_clearance = false;
  const DwaResult abs = DwaPlanner(c).compute(cruising(&path, 0.5), fp, band);
  EXPECT_DOUBLE_EQ(rel.best.terms.clearance, 0.0);
  EXPECT_NEAR(abs.best.terms.clearance, 150.0 / 252.0, 1e-12);
}

TEST(DwaAblation, TextbookCostHasOnlyHeadingClearanceAndSpeed)
{
  // 경로 0.3 m 옆에서 경로와 나란히: 교과서 비용은 경로·이탈·진동 항이 0 이고, 목표점(1.2 m 앞
  // 경로 점) 쪽으로 도는 후보의 방향 항이 작다.
  const auto path = straight(8.0);
  DwaConfig c;
  c.vth_samples = 31;
  c.textbook_cost = true;
  DwaInput in = cruising(&path, 0.6);
  in.pose = {0.0, 0.3, 0.0};
  const DwaResult r = emptyRun(DwaPlanner(c), in);
  EXPECT_DOUBLE_EQ(r.best.terms.path, 0.0);
  EXPECT_DOUBLE_EQ(r.best.terms.off_path, 0.0);
  EXPECT_DOUBLE_EQ(r.best.terms.oscillation, 0.0);
  EXPECT_LT(r.w, 0.0);   // 경로(−y) 쪽으로 돈다
  EXPECT_GT(r.best.terms.heading, 0.0);
  // 속도 항 = (v_cap − v)/v_span: 빈 공간에서는 창 최고 속도가 이긴다 (헤딩 차보다 크다)
  EXPECT_NEAR(r.v, 0.65, 1e-9);
}

TEST(DwaAblation, SpeedDependentHorizonIsConstantAtDeployedLimits)
{
  // M4-a: T_sim(v) = clip(|v|/a + 0.5, 1.5, 2.5). a = 1.0, |v| ≤ 1.0 이면 늘 1.5 s 하한 —
  // 지평을 1.5 s 로 고정한 절제 팔은 배포와 같은 명령열을 낸다 (효과 0 이 정의상 맞다).
  DwaConfig c;
  for (double v = 0.0; v <= 1.0 + 1e-9; v += 0.05) {
    EXPECT_DOUBLE_EQ(DwaPlanner(c).simTime(v), 1.5) << "v=" << v;
  }
  EXPECT_GT(DwaPlanner(c).simTime(1.5), 1.5);    // 1.0 m/s 초과에서만 변한다
  DwaConfig loaded = c;
  loaded.limits.acc_lim_x = 0.5;                  // 적재(가속 한계 축소)에서도 변한다
  EXPECT_GT(DwaPlanner(loaded).simTime(0.8), 1.5);
  DwaConfig fixed = c;
  fixed.vth_samples = 31;
  fixed.weights.oscillation = 0.5;
  fixed.goal_align_distance = 0.08;
  DwaConfig deployed = fixed;
  fixed.sim_time_min = fixed.sim_time_max = 1.5;
  EXPECT_EQ(chainHash(fixed, sCurve(), 600), chainHash(deployed, sCurve(), 600));
  EXPECT_EQ(chainHash(fixed, uTurn(), 500), chainHash(deployed, uTurn(), 500));
}

TEST(DwaAblation, EachSwitchChangesTheClosedLoop)
{
  // 스위치가 실제로 연결돼 있는가: 하나씩 끄면 체인 S 자 명령열이 달라진다 (M4-d·M1-c 는 빈
  // 코스트맵에서 효과가 없어야 정상이라 같다).
  DwaConfig base;
  base.vth_samples = 31;
  base.weights.oscillation = 0.5;
  base.goal_align_distance = 0.08;
  const std::uint64_t ref = chainHash(base, sCurve(), 600);
  DwaConfig m1 = base;
  m1.textbook_cost = true;
  DwaConfig m2 = base;
  m2.goal_approach = 1;
  DwaConfig m3 = base;
  m3.window_center_measured = true;
  DwaConfig m1e = base;
  m1e.path_eval_time = 0.0;
  EXPECT_NE(chainHash(m1, sCurve(), 600), ref);
  EXPECT_NE(chainHash(m2, sCurve(), 600), ref);
  EXPECT_NE(chainHash(m3, sCurve(), 600), ref);
  EXPECT_NE(chainHash(m1e, sCurve(), 600), ref);
  DwaConfig m4 = base;
  m4.brake_candidate = false;
  DwaConfig m1c = base;
  m1c.relative_clearance = false;
  EXPECT_EQ(chainHash(m4, sCurve(), 600), ref);
  EXPECT_EQ(chainHash(m1c, sCurve(), 600), ref);
}
