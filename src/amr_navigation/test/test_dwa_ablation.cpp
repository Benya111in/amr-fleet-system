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
