// DwaPlannerX 기본값 = 원본 DwaPlanner (비트 단위) 시험 (prereg.md §3.3, contract.md §1).
//
// 1) golden: 원본 test_dwa_ablation 의 폐루프 해시 8 개 (기준 코어 2345e27 에서 뽑은 상수)를
//    DwaPlannerX 로 그대로 다시 낸다. 같은 루프를 원본 DwaPlanner 로도 돌려 둘이 같은지도 본다.
// 2) 무작위 한 주기: 설정 변형(배포·절제 스위치·sustained·게이트·후진 허용) × 무작위
//    경로·자세·속도·직전 명령·코스트맵·동적 장애물에서 결과 전체(명령, 후보, 비용항, 창,
//    양보·게이트, 계측)를 비트 단위로 비교한다.
// 3) 무작위 폐루프: 같은 사슬(프로파일러·서보)을 두 계획기로 나란히 돌려 매 주기 결과를 비교한다.
#include <gtest/gtest.h>

#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "amr_nav_research/dwa_x.hpp"
#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/pid.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"
#include "aisle_scene.hpp"

namespace core = amr_navigation::core;
using amr_nav_research::DwaInputX;
using amr_nav_research::DwaPlannerX;
using amr_nav_research::DwaResultX;
using amr_nav_research::ResearchState;
using core::DwaConfig;
using core::DwaInput;
using core::DwaPlanner;
using core::DwaResult;
using core::DynamicObstacle;
using core::Pose2D;
using core::integrateArc;
using core::kPi;

namespace
{
// ---------------------------------------------------------------------------------------------
// 비트 비교

bool sameBits(double a, double b)
{
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

class Diff
{
public:
  void d(const char * name, double a, double b)
  {
    if (msg_.empty() && !sameBits(a, b)) {
      std::ostringstream o;
      o.precision(17);
      o << name << ": " << a << " != " << b;
      msg_ = o.str();
    }
  }
  void i(const char * name, std::int64_t a, std::int64_t b)
  {
    if (msg_.empty() && a != b) {
      msg_ = std::string(name) + ": " + std::to_string(a) + " != " + std::to_string(b);
    }
  }
  void poses(const char * name, const std::vector<Pose2D> & a, const std::vector<Pose2D> & b)
  {
    i(name, static_cast<std::int64_t>(a.size()), static_cast<std::int64_t>(b.size()));
    for (std::size_t k = 0; k < a.size() && k < b.size() && msg_.empty(); ++k) {
      d(name, a[k].x, b[k].x);
      d(name, a[k].y, b[k].y);
      d(name, a[k].theta, b[k].theta);
    }
  }
  void vec(const char * name, const std::vector<double> & a, const std::vector<double> & b)
  {
    i(name, static_cast<std::int64_t>(a.size()), static_cast<std::int64_t>(b.size()));
    for (std::size_t k = 0; k < a.size() && k < b.size(); ++k) {
      d(name, a[k], b[k]);
    }
  }
  const std::string & msg() const {return msg_;}

private:
  std::string msg_;
};

// 결과 전체 비교. 빈 문자열이면 같다.
std::string compareResults(const DwaResult & a, const DwaResult & b)
{
  Diff f;
  f.i("found", a.found, b.found);
  f.d("v", a.v, b.v);
  f.d("w", a.w, b.w);
  f.i("n_samples", a.n_samples, b.n_samples);
  f.i("n_valid", a.n_valid, b.n_valid);
  f.i("n_collision", a.n_collision, b.n_collision);
  f.i("n_vo_rejected", a.n_vo_rejected, b.n_vo_rejected);
  f.i("vo_saturated", a.vo_saturated, b.vo_saturated);
  f.i("n_recentered", a.n_recentered, b.n_recentered);
  f.d("d_goal", a.d_goal, b.d_goal);
  f.d("v_cap", a.v_cap, b.v_cap);
  f.d("window.v_lo", a.window.v_lo, b.window.v_lo);
  f.d("window.v_hi", a.window.v_hi, b.window.v_hi);
  f.d("window.w_lo", a.window.w_lo, b.window.w_lo);
  f.d("window.w_hi", a.window.w_hi, b.window.w_hi);
  f.d("window.v_cap", a.window.v_cap, b.window.v_cap);
  // 선택 후보
  const auto & p = a.best;
  const auto & q = b.best;
  f.d("best.v", p.v, q.v);
  f.d("best.w", p.w, q.w);
  f.poses("best.poses", p.poses, q.poses);
  f.d("best.heading", p.terms.heading, q.terms.heading);
  f.d("best.clearance", p.terms.clearance, q.terms.clearance);
  f.d("best.velocity", p.terms.velocity, q.terms.velocity);
  f.d("best.path", p.terms.path, q.terms.path);
  f.d("best.oscillation", p.terms.oscillation, q.terms.oscillation);
  f.d("best.dynamic", p.terms.dynamic, q.terms.dynamic);
  f.d("best.off_path", p.terms.off_path, q.terms.off_path);
  f.d("best.escape", p.terms.escape, q.terms.escape);
  f.d("best.cost", p.cost, q.cost);
  f.i("best.collision", p.collision, q.collision);
  f.i("best.vo_rejected", p.vo_rejected, q.vo_rejected);
  f.i("best.is_brake", p.is_brake, q.is_brake);
  f.d("best.ttc", p.ttc, q.ttc);
  f.d("best.vo_time", p.vo_time, q.vo_time);
  f.d("best.max_cte", p.max_cte, q.max_cte);
  f.d("best.v_emit", p.v_emit, q.v_emit);
  f.d("best.w_emit", p.w_emit, q.w_emit);
  f.vec("best.arc", p.arc, q.arc);
  f.d("best.v_peak", p.v_peak, q.v_peak);
  // 양보·게이트
  f.i("yield.state", static_cast<int>(a.yield.state), static_cast<int>(b.yield.state));
  f.d("yield.speed_limit", a.yield.speed_limit, b.yield.speed_limit);
  f.d("yield.stop_distance", a.yield.stop_distance, b.yield.stop_distance);
  f.d("yield.zone_entry", a.yield.zone_entry, b.yield.zone_entry);
  f.d("yield.zone_exit", a.yield.zone_exit, b.yield.zone_exit);
  f.i("yield.obstacle", a.yield.obstacle, b.yield.obstacle);
  f.i("yield.decision", static_cast<int>(a.yield.decision), static_cast<int>(b.yield.decision));
  f.d("yield.margin", a.yield.margin, b.yield.margin);
  f.i("yield.decided", a.yield.decided.size(), b.yield.decided.size());
  f.i("gate.phase", static_cast<int>(a.gate.phase), static_cast<int>(b.gate.phase));
  f.i("gate.reason", static_cast<int>(a.gate.reason), static_cast<int>(b.gate.reason));
  f.d("gate.speed_limit", a.gate.speed_limit, b.gate.speed_limit);
  f.i("gate.hard_stop", a.gate.hard_stop, b.gate.hard_stop);
  f.i("gate.retreat", a.gate.retreat, b.gate.retreat);
  f.d("gate.window_s", a.gate.window_s, b.gate.window_s);
  f.i("gate.n_threats", a.gate.n_threats, b.gate.n_threats);
  f.i("gate.exempt", a.gate.exempt_ids.size(), b.gate.exempt_ids.size());
  f.i("gate.next.phase", static_cast<int>(a.gate.next.phase), static_cast<int>(b.gate.next.phase));
  // R0 계측
  f.d("diag.cost_min", a.diag.cost_min, b.diag.cost_min);
  f.d("diag.cost_max", a.diag.cost_max, b.diag.cost_max);
  f.i("diag.n_selectable", a.diag.n_selectable, b.diag.n_selectable);
  f.i("diag.n_collision_free", a.diag.n_collision_free, b.diag.n_collision_free);
  f.d("diag.v_lo", a.diag.v_lo, b.diag.v_lo);
  f.d("diag.v_hi", a.diag.v_hi, b.diag.v_hi);
  f.d("diag.w_lo", a.diag.w_lo, b.diag.w_lo);
  f.d("diag.w_hi", a.diag.w_hi, b.diag.w_hi);
  f.d("diag.disp_span", a.diag.disp_span, b.diag.disp_span);
  f.d("diag.ttc_min", a.diag.ttc_min, b.diag.ttc_min);
  f.d("diag.nearest_obs", a.diag.nearest_obs, b.diag.nearest_obs);
  f.i("diag.escaping", a.diag.escaping, b.diag.escaping);
  f.i("diag.leave_lane", a.diag.leave_lane, b.diag.leave_lane);
  for (std::size_t k = 0; k < a.diag.term_span.size(); ++k) {
    f.d("diag.term_span", a.diag.term_span[k], b.diag.term_span[k]);
  }
  return f.msg();
}

// ---------------------------------------------------------------------------------------------
// 두 계획기를 같은 방식으로 부르는 어댑터. X 쪽은 플러그인·chain_sim_x 처럼 연구 상태를 되먹인다.

struct OrigRunner
{
  explicit OrigRunner(const DwaConfig & c)
  : p(c) {}
  template<class FP, class PC>
  DwaResult step(const DwaInput & in, const FP & fp, const PC & pc)
  {
    return p.compute(in, fp, pc);
  }
  DwaPlanner p;
};

struct XRunner
{
  explicit XRunner(const DwaConfig & c)
  : p(c) {}
  template<class FP, class PC>
  DwaResult step(const DwaInput & in, const FP & fp, const PC & pc)
  {
    DwaInputX x(in);
    x.research = st;
    const DwaResultX r = p.compute(x, fp, pc);
    st = r.next;
    return r;
  }
  DwaPlannerX p;
  ResearchState st;
};

// ---------------------------------------------------------------------------------------------
// 원본 test_dwa_ablation.cpp 의 golden 루프 (어댑터만 일반화)

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

DwaConfig coreDefault()
{
  DwaConfig c;
  c.vth_samples = 31;
  c.weights.oscillation = 0.5;
  c.goal_align_distance = 0.08;
  return c;
}

template<class R>
std::uint64_t chainHash(R & dwa, const std::vector<Pose2D> & path, int cycles)
{
  core::VelocityProfiler prof;
  core::FirstOrderPlant sv(1.0, 0.08, 0.04, 0.01, 0.0);
  core::FirstOrderPlant sw(1.0, 0.08, 0.04, 0.01, 0.0);
  DwaInput in;
  in.pose = path.front();
  in.path = &path;
  in.path_end_is_goal = true;
  Hash h;
  double pv = 0.0, pw = 0.0, vc = 0.0, wc = 0.0;
  bool done = false;
  for (int k = 0; k < cycles; ++k) {
    if (!done) {
      const double dg = std::hypot(in.pose.x - path.back().x, in.pose.y - path.back().y);
      if (dg <= 0.10 && std::abs(core::wrapAngle(path.back().theta - in.pose.theta)) <= 0.05) {
        done = true;
        vc = 0.0;
        wc = 0.0;
      } else {
        const DwaResult r = dwa.step(
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

template<class R>
std::uint64_t aisleHash(const DwaConfig & base, unsigned seed)
{
  aisle::Scene sc;
  sc.resolution = 0.025;
  sc.denoise = true;
  sc.rng.seed(seed);
  DwaConfig c = base;
  c.recenter_step = 0.025;
  R dwa(c);
  std::vector<Pose2D> path;
  for (double s = 0.0; s <= 8.3 + 1e-9; s += 0.05) {
    path.push_back({s, -0.035, 0.0});
  }
  DwaInput in;
  in.pose = {1.8, -0.09, 0.14};
  in.v_meas = in.v_last = 0.3;
  in.has_last = true;
  in.path = &path;
  core::VelocityProfiler prof;
  prof.reset(0.3, 0.0);
  core::FirstOrderPlant sv(1.0, 0.08, 0.04, 0.01, 0.3);
  core::FirstOrderPlant sw(1.0, 0.08, 0.04, 0.01, 0.0);
  double pv = 0.3, pw = 0.0;
  Hash h;
  for (int k = 0; k < 300; ++k) {
    if (k % 2 == 0) {
      sc.observe(in.pose);
    }
    const auto chk = sc.checker();
    const auto view = sc.grid.view();
    const DwaResult r = dwa.step(
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

template<class R>
std::uint64_t crossingHash(const DwaConfig & cfg, double ox, double oy, double ovy)
{
  R dwa(cfg);
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
    const DwaResult r = dwa.step(
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

template<class R>
std::uint64_t chainHashCfg(const DwaConfig & c, const std::vector<Pose2D> & path, int cycles)
{
  R r(c);
  return chainHash(r, path, cycles);
}
}  // namespace

// 원본 test_dwa_ablation DefaultsAreBitIdenticalToBaselineCore 와 같은 상수.
TEST(DwaXEquivalence, GoldenHashesOfTheBaselineCore)
{
  struct G
  {
    const char * name;
    std::uint64_t x;
    std::uint64_t orig;
    std::uint64_t golden;
  };
  const std::vector<G> gs{
    {"scurve_chain_core", chainHashCfg<XRunner>(coreDefault(), sCurve(), 600),
      chainHashCfg<OrigRunner>(coreDefault(), sCurve(), 600), 0x1d3e87f7180083b8ULL},
    {"uturn_chain_core", chainHashCfg<XRunner>(coreDefault(), uTurn(), 500),
      chainHashCfg<OrigRunner>(coreDefault(), uTurn(), 500), 0x4137d80ff58b300dULL},
    {"scurve_chain_deployed", chainHashCfg<XRunner>(deployedConfig(), sCurve(), 600),
      chainHashCfg<OrigRunner>(deployedConfig(), sCurve(), 600), 0x1d3e87f7180083b8ULL},
    {"uturn_chain_deployed", chainHashCfg<XRunner>(deployedConfig(), uTurn(), 500),
      chainHashCfg<OrigRunner>(deployedConfig(), uTurn(), 500), 0x4137d80ff58b300dULL},
    {"aisle_deployed_s1", aisleHash<XRunner>(deployedConfig(), 1U),
      aisleHash<OrigRunner>(deployedConfig(), 1U), 0x695bdc1efd370c9dULL},
    {"aisle_deployed_s2", aisleHash<XRunner>(deployedConfig(), 2U),
      aisleHash<OrigRunner>(deployedConfig(), 2U), 0x29e6e63078414420ULL},
    {"crossing_core", crossingHash<XRunner>(coreDefault(), 4.0, 3.2, -1.0),
      crossingHash<OrigRunner>(coreDefault(), 4.0, 3.2, -1.0), 0x2e5db9e09019019dULL},
    {"crossing_deployed", crossingHash<XRunner>(deployedConfig(), 3.0, 2.0, -1.0),
      crossingHash<OrigRunner>(deployedConfig(), 3.0, 2.0, -1.0), 0x7c15561996e60a67ULL},
  };
  for (const auto & g : gs) {
    EXPECT_EQ(g.x, g.golden) << g.name << " — DwaPlannerX 기본값이 기준 코어와 다르다";
    EXPECT_EQ(g.orig, g.golden) << g.name << " — 원본 코어 자체가 golden 과 다르다";
  }
}

// 절제 스위치를 켠 설정에서도 (연구 스위치만 끄면) 원본과 같은가 — 폐루프 S 자·U 턴.
TEST(DwaXEquivalence, AblationSwitchesStillMatchTheOriginal)
{
  std::vector<DwaConfig> cs;
  DwaConfig b = coreDefault();
  cs.push_back(b);
  DwaConfig m1 = b;
  m1.textbook_cost = true;
  cs.push_back(m1);
  DwaConfig m2 = b;
  m2.goal_approach = 1;
  cs.push_back(m2);
  DwaConfig m2t = b;
  m2t.goal_approach = 2;
  cs.push_back(m2t);
  DwaConfig m3 = b;
  m3.window_center_measured = true;
  cs.push_back(m3);
  DwaConfig m4 = b;
  m4.brake_candidate = false;
  m4.sim_time_max = 1.5;
  cs.push_back(m4);
  DwaConfig m1e = b;
  m1e.path_eval_time = 0.0;
  cs.push_back(m1e);
  DwaConfig sus = b;
  sus.sustained_sampling = true;
  cs.push_back(sus);
  for (std::size_t i = 0; i < cs.size(); ++i) {
    EXPECT_EQ(
      chainHashCfg<XRunner>(cs[i], sCurve(), 600),
      chainHashCfg<OrigRunner>(cs[i], sCurve(), 600)) << "config " << i;
    EXPECT_EQ(
      chainHashCfg<XRunner>(cs[i], uTurn(), 500),
      chainHashCfg<OrigRunner>(cs[i], uTurn(), 500)) << "config " << i;
  }
}

// ---------------------------------------------------------------------------------------------
// 무작위 입력

namespace
{
struct World
{
  core::OwnedGrid grid;
  std::vector<core::Point2D> fp = core::FootprintChecker::rectangle(0.60, 0.40);
};

World randomWorld(std::mt19937 & g, bool empty)
{
  World w;
  w.grid = core::OwnedGrid(480, 480, 0.05, 0, -12.0, -12.0);
  if (!empty) {
    std::uniform_real_distribution<double> pos(-10.0, 10.0);
    std::uniform_real_distribution<double> size(0.2, 1.5);
    std::uniform_int_distribution<int> n(4, 16);
    const int m = n(g);
    for (int i = 0; i < m; ++i) {
      const double x = pos(g);
      const double y = pos(g);
      w.grid.fillRect(x, y, x + size(g), y + size(g), core::kLethalObstacle);
    }
    // 좁은 통로 (재중심 분기)
    w.grid.fillRect(2.0, 0.30, 6.0, 1.30, core::kLethalObstacle);
    w.grid.fillRect(2.0, -1.30, 6.0, -0.30, core::kLethalObstacle);
  }
  core::inflate(w.grid, 0.2, 0.8, 3.0);
  return w;
}

std::vector<Pose2D> randomPath(std::mt19937 & g, const Pose2D & start)
{
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::vector<Pose2D> p{start};
  const int segs = 1 + static_cast<int>(u(g) * 4.0);
  for (int s = 0; s < segs; ++s) {
    if (u(g) < 0.5) {
      appendStraight(p, 0.5 + 3.0 * u(g));
    } else {
      appendArc(p, 0.8 + 4.0 * u(g), (u(g) - 0.5) * 2.0 * kPi / 2.0);
    }
  }
  return p;
}

std::vector<DwaConfig> configVariants()
{
  std::vector<DwaConfig> cs;
  cs.push_back(coreDefault());
  cs.push_back(deployedConfig());
  DwaConfig dep_dec = deployedConfig();
  dep_dec.yield_decision = true;
  cs.push_back(dep_dec);
  DwaConfig sus = deployedConfig();
  sus.sustained_sampling = true;
  cs.push_back(sus);
  DwaConfig tb = deployedConfig();
  tb.textbook_cost = true;
  tb.relative_clearance = false;
  tb.goal_approach = 1;
  tb.window_center_measured = true;
  tb.brake_candidate = false;
  cs.push_back(tb);
  DwaConfig rev = deployedConfig();
  rev.limits.min_vel_x = -0.3;
  cs.push_back(rev);
  DwaConfig gate = deployedConfig();
  gate.gate.enable = true;
  cs.push_back(gate);
  DwaConfig trap = deployedConfig();
  trap.goal_approach = 2;
  trap.recenter_narrow = false;
  trap.path_eval_time = 0.0;
  cs.push_back(trap);
  return cs;
}

DwaInput randomInput(
  std::mt19937 & g, const std::vector<Pose2D> & path, const DwaConfig & c, bool with_obs)
{
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::normal_distribution<double> n(0.0, 1.0);
  DwaInput in;
  // 경로 위 임의 점 근처 (목표 근처도 자주 — 접근·정렬 분기)
  const std::size_t k = u(g) < 0.25 ? path.size() - 1 - std::min<std::size_t>(
    path.size() - 1, static_cast<std::size_t>(u(g) * 10.0)) :
    static_cast<std::size_t>(u(g) * static_cast<double>(path.size() - 1));
  const Pose2D & b = path[k];
  in.pose = {b.x + 0.15 * n(g), b.y + 0.15 * n(g), core::wrapAngle(b.theta + 0.4 * n(g))};
  in.v_meas = std::clamp(c.limits.min_vel_x + u(g) * (1.05 - c.limits.min_vel_x), -0.5, 1.0);
  in.w_meas = std::clamp(0.6 * n(g), -1.5, 1.5);
  in.has_last = u(g) < 0.85;
  // 직전 명령: 측정값 근처 (재설정 임계 0.3 을 넘는 경우도 섞는다)
  in.v_last = in.v_meas + 0.25 * n(g);
  in.w_last = in.w_meas + 0.4 * n(g);
  in.path = &path;
  in.path_end_is_goal = u(g) < 0.8;
  in.speed_limit = u(g) < 0.8 ? std::numeric_limits<double>::infinity() : 0.1 + 0.9 * u(g);
  in.yield_limit_last = u(g) < 0.8 ? std::numeric_limits<double>::infinity() : u(g);
  in.yield_decision_last = static_cast<int>(u(g) * 4.0);
  if (with_obs) {
    const int m = static_cast<int>(u(g) * 3.0);
    for (int i = 0; i < m; ++i) {
      DynamicObstacle o;
      const double a = 2.0 * kPi * u(g);
      const double r = 1.0 + 5.0 * u(g);
      o.x = in.pose.x + r * std::cos(a);
      o.y = in.pose.y + r * std::sin(a);
      const double h = 2.0 * kPi * u(g);
      const double s = 1.3 * u(g);
      o.vx = s * std::cos(h);
      o.vy = s * std::sin(h);
      o.radius = 0.25;
      o.id = i;
      if (u(g) < 0.5) {
        o.vx_pred = 0.9 * o.vx;
        o.vy_pred = 0.9 * o.vy;
        o.has_pred = true;
      }
      in.obstacles.push_back(o);
      core::GateTrack t;
      t.id = i;
      t.x = o.x;
      t.y = o.y;
      t.heading_known = u(g) < 0.7;
      t.ux = std::cos(h);
      t.uy = std::sin(h);
      t.speed = s;
      t.stationary_s = t.heading_known ? 0.0 : 3.0 * u(g);
      in.gate_tracks.push_back(t);
    }
  }
  return in;
}
}  // namespace

TEST(DwaXEquivalence, RandomSingleCyclesAreBitIdentical)
{
  std::mt19937 g(20261017U);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const auto variants = configVariants();
  std::vector<World> worlds;
  for (int i = 0; i < 6; ++i) {
    worlds.push_back(randomWorld(g, i == 0));
  }
  int n = 0;
  int n_moving = 0;
  for (int trial = 0; trial < 1600; ++trial) {
    const DwaConfig & c = variants[static_cast<std::size_t>(trial) % variants.size()];
    const World & w = worlds[static_cast<std::size_t>(u(g) * worlds.size())];
    DwaConfig cc = c;
    cc.recenter_step = 0.05;
    const Pose2D start{-8.0 + 16.0 * u(g), -8.0 + 16.0 * u(g), 2.0 * kPi * u(g)};
    const std::vector<Pose2D> path = u(g) < 0.05 ? std::vector<Pose2D>{} : randomPath(g, start);
    const std::vector<Pose2D> ref = path.empty() ? std::vector<Pose2D>{start} : path;
    const DwaInput in = randomInput(g, ref, cc, u(g) < 0.5);
    DwaInput in2 = in;
    in2.path = path.empty() ? nullptr : &path;
    const core::FootprintChecker chk(
      w.grid.view(), w.fp,
      core::inflationCost(core::FootprintChecker::circumscribedRadius(w.fp), 0.2, 3.0), true);
    const auto view = w.grid.view();
    auto fp = [&chk](const Pose2D & p) {return chk.cost(p);};
    auto pc = [view](double x, double y) {
        const uint8_t v = view.costAtWorld(x, y);
        return v == core::kNoInformation ? 0.0 : static_cast<double>(v);
      };
    const DwaResult a = DwaPlanner(cc).compute(in2, fp, pc);
    DwaInputX x(in2);
    // 연구 상태는 기본 스위치에서 출력에 영향이 없어야 한다 — 일부러 엉뚱한 값을 넣는다
    x.research.persist_count = static_cast<int>(u(g) * 5.0);
    x.research.a_last = 3.0 * (u(g) - 0.5);
    x.research.alpha_last = 5.0 * (u(g) - 0.5);
    const DwaResultX b = DwaPlannerX(cc).compute(x, fp, pc);
    const std::string d = compareResults(a, b);
    ASSERT_TRUE(d.empty()) << "trial " << trial << " config " <<
      (trial % static_cast<int>(variants.size())) << ": " << d;
    // 연구 출력: 기본값이면 sample_window = window, 연구 후보 없음
    ASSERT_FALSE(b.pp_added);
    ASSERT_FALSE(b.jerk_applied);
    ASSERT_FALSE(b.center.suppressed);
    ASSERT_TRUE(sameBits(b.sample_window.v_lo, b.window.v_lo));
    ASSERT_TRUE(sameBits(b.sample_window.v_hi, b.window.v_hi));
    ++n;
    n_moving += std::abs(a.v) > 1e-3;
  }
  EXPECT_EQ(n, 1600);
  EXPECT_GT(n_moving, 400);   // 의미 있는 표본인지 (대부분 정지 명령이 아니어야)
}

TEST(DwaXEquivalence, StageFunctionsMatchTheOriginal)
{
  std::mt19937 g(7U);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (const DwaConfig & c : configVariants()) {
    const DwaPlanner a(c);
    const DwaPlannerX b(c);
    for (int k = 0; k < 200; ++k) {
      DwaInput in;
      in.has_last = u(g) > -0.6;
      in.v_meas = u(g);
      in.w_meas = 1.5 * u(g);
      in.v_last = in.v_meas + 0.4 * u(g);
      in.w_last = in.w_meas + 0.7 * u(g);
      const auto ca = a.windowCenter(in);
      const auto cb = b.windowCenter(in);
      const auto cx = b.windowCenterX(in, ResearchState{});
      ASSERT_TRUE(sameBits(ca.first, cb.first) && sameBits(ca.second, cb.second));
      ASSERT_TRUE(sameBits(ca.first, cx.v_c) && sameBits(ca.second, cx.w_c));
      const double cap = u(g) > 0.0 ? 1.0 : 0.5 + 0.5 * u(g);
      const auto wa = a.dynamicWindow(ca.first, ca.second, cap);
      const auto wb = b.dynamicWindow(ca.first, ca.second, cap);
      ASSERT_TRUE(sameBits(wa.v_lo, wb.v_lo) && sameBits(wa.v_hi, wb.v_hi));
      ASSERT_TRUE(sameBits(wa.w_lo, wb.w_lo) && sameBits(wa.w_hi, wb.w_hi));
      const auto sa = a.sampleVelocities(wa, ca.first, ca.second);
      const auto sb = b.sampleVelocities(wb, ca.first, ca.second);
      ASSERT_EQ(sa.size(), sb.size());
      for (std::size_t i = 0; i < sa.size(); ++i) {
        ASSERT_TRUE(sameBits(sa[i].first, sb[i].first) && sameBits(sa[i].second, sb[i].second));
      }
      ASSERT_TRUE(sameBits(a.simTime(in.v_meas), b.simTime(in.v_meas)));
    }
  }
}

// 무작위 폐루프: 사슬(프로파일러·서보) 위에서 두 계획기를 나란히 돌리며 매 주기 결과를 비교한다.
TEST(DwaXEquivalence, RandomClosedLoopsAreBitIdentical)
{
  std::mt19937 g(4242U);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const auto variants = configVariants();
  const World w = randomWorld(g, false);
  int cycles_total = 0;
  for (int run = 0; run < 24; ++run) {
    DwaConfig c = variants[static_cast<std::size_t>(run) % variants.size()];
    c.recenter_step = 0.05;
    const Pose2D start{-6.0 + 12.0 * u(g), -6.0 + 12.0 * u(g), 2.0 * kPi * u(g)};
    const std::vector<Pose2D> path = randomPath(g, start);
    OrigRunner ra(c);
    XRunner rb(c);
    const core::FootprintChecker chk(
      w.grid.view(), w.fp,
      core::inflationCost(core::FootprintChecker::circumscribedRadius(w.fp), 0.2, 3.0), true);
    const auto view = w.grid.view();
    auto fp = [&chk](const Pose2D & p) {return chk.cost(p);};
    auto pc = [view](double x, double y) {
        const uint8_t v = view.costAtWorld(x, y);
        return v == core::kNoInformation ? 0.0 : static_cast<double>(v);
      };
    core::VelocityProfiler prof;
    core::FirstOrderPlant sv(1.0, 0.08, 0.04, 0.01, 0.0);
    core::FirstOrderPlant sw(1.0, 0.08, 0.04, 0.01, 0.0);
    std::normal_distribution<double> noise(0.0, 0.02);
    DwaInput in;
    in.pose = path.front();
    in.path = &path;
    double pv = 0.0, pw = 0.0;
    for (int k = 0; k < 400; ++k) {
      const DwaResult a = ra.step(in, fp, pc);
      const DwaResult b = rb.step(in, fp, pc);
      const std::string d = compareResults(a, b);
      ASSERT_TRUE(d.empty()) << "run " << run << " cycle " << k << ": " << d;
      ++cycles_total;
      for (int sub = 0; sub < 5; ++sub) {
        if (sub % 2 == 0) {
          const auto o = prof.update(a.v, a.w, true, sv.output(), sw.output(), 0.02);
          pv = o.v;
          pw = o.w;
        }
        in.pose = integrateArc(in.pose, sv.step(pv), sw.step(pw), 0.01);
      }
      // 오도메트리 잡음 (재설정 분기를 지나게)
      in.v_meas = sv.output() + (run % 2 == 0 ? noise(g) : 0.0);
      in.w_meas = sw.output() + (run % 2 == 0 ? noise(g) : 0.0);
      in.v_last = a.v;
      in.w_last = a.w;
      in.has_last = true;
      in.yield_limit_last = a.yield.speed_limit;
    }
  }
  EXPECT_EQ(cycles_total, 24 * 400);
}
