// 2 라운드 스위치 시험 (prereg_r2.md §0):
//   F1 사슬 모형 재설정 — 모형이 chain_sim 사슬(프로파일러 + 서보)을 재현하는가, 가속 중
//      재설정이 없는가, 막힘(측정 0 고정)이면 재설정하는가, hard 규칙이 독립인가.
//   F2 목표 연장 — 0.10 m 첫 진입 순간 yaw 오차가 줄어드는가, d_goal·접근 상한은 그대로인가.
//   F3 막힌 경로 우회 — 원판이 경로를 막으면 우회 경로가 생기고 충돌 없이 지나가는가,
//      양쪽 모두 막히면 원래 경로인가.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "amr_nav_research/chain_model.hpp"
#include "amr_nav_research/dwa_x.hpp"
#include "amr_nav_research/research_io.hpp"
#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"

namespace core = amr_navigation::core;
using amr_nav_research::ChainModelConfig;
using amr_nav_research::ChainModelState;
using amr_nav_research::DetourInfo;
using amr_nav_research::DwaInputX;
using amr_nav_research::DwaPlannerX;
using amr_nav_research::DwaResultX;
using amr_nav_research::GoalCheckEmu;
using amr_nav_research::ResearchConfig;
using amr_nav_research::ResearchState;
using amr_nav_research::ResetReason;
using core::DwaConfig;
using core::DwaInput;
using core::Pose2D;
using core::integrateArc;
using core::kPi;

namespace
{
DwaConfig deployed()
{
  DwaConfig c;
  c.vth_samples = 31;
  c.weights.oscillation = 0.5;
  c.goal_align_distance = 0.08;
  c.robot_radius = 0.361;
  c.off_path_band = 1.0;
  c.yield_decision = false;
  c.recenter_step = 0.025;
  return c;
}

ResearchConfig x03()   // 1 라운드 선택 DWA-R (R1 K = 6) — 2 라운드 격자의 바탕
{
  ResearchConfig r;
  r.reset_persist_cycles = 6;
  return r;
}

std::vector<Pose2D> straight(double len, double y = 0.0)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    p.push_back({s, y, 0.0});
  }
  return p;
}

void appendArc(std::vector<Pose2D> & c, double r, double a)
{
  const Pose2D o = c.back();
  const int n = static_cast<int>(std::lround(std::abs(a) * r / 0.05));
  for (int i = 1; i <= n; ++i) {
    c.push_back(integrateArc(o, 1.0, (a >= 0.0 ? 1.0 : -1.0) / r, i * 0.05));
  }
}

void appendStraight(std::vector<Pose2D> & c, double len)
{
  const Pose2D o = c.back();
  const int n = static_cast<int>(std::lround(len / 0.05));
  for (int i = 1; i <= n; ++i) {
    c.push_back(integrateArc(o, 1.0, 0.0, i * 0.05));
  }
}

// chain_sim Servo (지연 → 1차 → 가속 제한)
struct Servo
{
  Servo(double tau, double delay, double a_max)
  : tau_(tau), a_max_(a_max), buf_(static_cast<std::size_t>(std::lround(delay / 0.01)), 0.0) {}
  double step(double cmd)
  {
    if (!buf_.empty()) {
      buf_.push_back(cmd);
      cmd = buf_.front();
      buf_.erase(buf_.begin());
    }
    const double alpha = 1.0 - std::exp(-0.01 / tau_);
    const double lim = a_max_ * 0.01;
    value_ += std::clamp(alpha * (cmd - value_), -lim, lim);
    return value_;
  }
  double tau_;
  double a_max_;
  std::vector<double> buf_;
  double value_{0.0};
};

struct Disc
{
  double x;
  double y;
  double r;
};

// 원판 장애물 코스트맵 (0.025 m, Nav2 식 팽창 0.2 / 0.8 / 3.0 — chain_sim 과 같은 값)
struct World
{
  core::OwnedGrid grid;
  std::vector<core::Point2D> fp = core::FootprintChecker::rectangle(0.60, 0.40);
  std::vector<Disc> discs;
  explicit World(const std::vector<Disc> & d, double wall_y = 0.0)
  : grid(640, 240, 0.025, 0, -2.0, -3.0), discs(d)
  {
    for (int my = 0; my < grid.height; ++my) {
      for (int mx = 0; mx < grid.width; ++mx) {
        const double x = grid.origin_x + (mx + 0.5) * 0.025;
        const double y = grid.origin_y + (my + 0.5) * 0.025;
        bool hit = wall_y > 0.0 && std::abs(y) >= wall_y && x > 3.0 && x < 7.0;
        for (const auto & o : discs) {
          hit = hit || std::hypot(x - o.x, y - o.y) <= o.r;
        }
        if (hit) {
          grid.at(mx, my) = core::kLethalObstacle;
        }
      }
    }
    core::inflate(grid, 0.2, 0.8, 3.0);
  }
  core::FootprintChecker checker() const
  {
    return core::FootprintChecker(
      grid.view(), fp,
      core::inflationCost(core::FootprintChecker::circumscribedRadius(fp), 0.2, 3.0), true);
  }
  // 참값: 로봇 사각형과 원판이 겹치는가
  bool collides(const Pose2D & p) const
  {
    for (const auto & o : discs) {
      const double dx = o.x - p.x;
      const double dy = o.y - p.y;
      const double lx = std::cos(p.theta) * dx + std::sin(p.theta) * dy;
      const double ly = -std::sin(p.theta) * dx + std::cos(p.theta) * dy;
      const double qx = std::clamp(lx, -0.3, 0.3);
      const double qy = std::clamp(ly, -0.2, 0.2);
      if (std::hypot(lx - qx, ly - qy) < o.r) {
        return true;
      }
    }
    return false;
  }
};

struct LoopRun
{
  bool reached{false};
  int resets{0};
  int suppressed{0};
  int detour{0};
  int side_flips{0};
  bool collided{false};
  double t_v90{-1.0};
  double yaw_entry{-1.0};
  double max_model_err{0.0};    // 재설정 없는 주기의 |v_meas − v̂| 최대
  double x_end{0.0};
};

// chain_sim 과 같은 시각표의 폐루프 (제어 20 Hz, 프로파일러 50 Hz, 서보 100 Hz)
LoopRun chainRun(
  const DwaConfig & c, const ResearchConfig & r, const std::vector<Pose2D> & path, int cycles,
  const World * world = nullptr, Pose2D start = Pose2D{1e9, 0.0, 0.0})
{
  const DwaPlannerX dwa(c, r);
  core::VelocityProfiler prof;
  Servo sv(0.08, 0.04, 1.0);
  Servo sw(0.08, 0.04, 2.0);
  DwaInputX in;
  in.pose = start.x > 1e8 ? path.front() : start;
  in.path = &path;
  in.path_end_is_goal = true;
  LoopRun run;
  double pv = 0.0, pw = 0.0, vc = 0.0, wc = 0.0;
  GoalCheckEmu goal(0.10, 0.05, false);
  const Pose2D g = path.back();
  int last_side = 0;
  for (int k = 0; k < cycles; ++k) {
    const double dg = std::hypot(in.pose.x - g.x, in.pose.y - g.y);
    if (goal.reached(dg, core::wrapAngle(g.theta - in.pose.theta))) {
      run.reached = true;
      break;
    }
    DwaResultX res;
    if (world != nullptr) {
      const auto chk = world->checker();
      const auto view = world->grid.view();
      res = dwa.compute(
        in, [&chk](const Pose2D & p) {return chk.cost(p);},
        [view](double x, double y) {
          const uint8_t v = view.costAtWorld(x, y);
          return v == core::kNoInformation ? 0.0 : static_cast<double>(v);
        });
    } else {
      res = dwa.compute(
        in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
    }
    run.resets += res.center.reason != ResetReason::kNone;
    run.suppressed += res.center.suppressed;
    if (std::isfinite(res.center.v_hat) && res.center.reason == ResetReason::kNone) {
      run.max_model_err = std::max(run.max_model_err, std::abs(in.v_meas - res.center.v_hat));
    }
    if (res.detour.active) {
      ++run.detour;
      run.side_flips += last_side != 0 && res.detour.side != last_side;
      last_side = res.detour.side;
    }
    vc = res.v;
    wc = res.w;
    in.v_last = res.v;
    in.w_last = res.w;
    in.has_last = true;
    in.research = res.next;
    for (int sub = 0; sub < 5; ++sub) {
      if ((k * 5 + sub) % 2 == 0) {
        const auto o = prof.update(vc, wc, true, sv.value_, sw.value_, 0.02);
        pv = o.v;
        pw = o.w;
      }
      in.pose = integrateArc(in.pose, sv.step(pv), sw.step(pw), 0.01);
      if (run.t_v90 < 0.0 && sv.value_ >= 0.9) {
        run.t_v90 = k * 0.05 + (sub + 1) * 0.01;
      }
      if (run.yaw_entry < 0.0 && std::hypot(in.pose.x - g.x, in.pose.y - g.y) <= 0.10) {
        run.yaw_entry = std::abs(core::wrapAngle(g.theta - in.pose.theta));
      }
      if (world != nullptr && world->collides(in.pose)) {
        run.collided = true;
      }
    }
    in.v_meas = sv.value_;
    in.w_meas = sw.value_;
  }
  run.x_end = in.pose.x;
  return run;
}
}  // namespace

// ---------------------------------------------------------------------------------------------
// F1

TEST(RoundTwoF1, ChainModelReproducesTheChainSimChain)
{
  // 같은 명령열을 chain_sim 사슬(VelocityProfiler + Servo)과 모형에 넣어 예상 측정을 비교한다
  const DwaPlannerX p(deployed(), ResearchConfig{});
  const ChainModelConfig mc = p.modelConfig();
  ChainModelState m;
  amr_nav_research::chainModelInit(&m, mc, 0.0, 0.0);
  core::VelocityProfiler prof;
  Servo sv(0.08, 0.04, 1.0);
  Servo sw(0.08, 0.04, 2.0);
  double pv = 0.0, pw = 0.0, err_v = 0.0, err_w = 0.0;
  for (int k = 0; k < 200; ++k) {
    // 가속 → 순항 + 곡선 → 감속 → 제자리 회전
    const double t = k * 0.05;
    const double vc = t < 1.0 ? t : (t < 5.0 ? 1.0 : std::max(0.0, 1.0 - (t - 5.0)));
    const double wc = t > 2.0 && t < 4.0 ? 0.5 : (t > 7.0 ? 1.0 : 0.0);
    for (int sub = 0; sub < 5; ++sub) {
      if ((k * 5 + sub) % 2 == 0) {
        const auto o = prof.update(vc, wc, true, sv.value_, sw.value_, 0.02);
        pv = o.v;
        pw = o.w;
      }
      sv.step(pv);
      sw.step(pw);
    }
    amr_nav_research::chainModelAdvance(&m, mc, vc, wc, 0.05);
    err_v = std::max(err_v, std::abs(m.sv - sv.value_));
    err_w = std::max(err_w, std::abs(m.sw - sw.value_));
  }
  std::printf("[F1 model] max |v̂ − v| %.4f m/s, |ŵ − w| %.4f rad/s\n", err_v, err_w);
  EXPECT_LT(err_v, 0.03);   // 차이는 프로파일러 PI 보정뿐이다 (모형에 넣지 않았다)
  EXPECT_LT(err_w, 0.05);
}

TEST(RoundTwoF1, NoResetWhileAcceleratingWhenTheModelMatches)
{
  const auto path = straight(20.0);
  ResearchConfig r = x03();
  const LoopRun base = chainRun(deployed(), r, path, 300);
  r.reset_model = true;
  r.reset_model_theta = 0.15;
  const LoopRun f1 = chainRun(deployed(), r, path, 300);
  std::printf(
    "[F1 chain] resets X03 %d F1 %d, suppressed F1 %d, t_v90 X03 %.2f F1 %.2f s, "
    "max |v_meas − v̂| %.4f\n", base.resets, f1.resets, f1.suppressed, base.t_v90, f1.t_v90,
    f1.max_model_err);
  EXPECT_GT(base.resets, 0);       // 원래 규칙은 가속 중 재설정한다 (K = 6 으로도 1 회)
  EXPECT_EQ(f1.resets, 0);         // 모형이 맞으면 재설정 후보 자체가 없다
  EXPECT_EQ(f1.suppressed, 0);
  EXPECT_LT(f1.max_model_err, 0.15);
  EXPECT_GT(f1.t_v90, 0.0);
}

TEST(RoundTwoF1, BlockedRobotIsReset)
{
  // 측정 0 고정 (막힘): 명령은 창 위로 오르는데 모형 예상은 따라 오른다 → |0 − v̂| > θ
  const auto path = straight(8.0);
  for (int K : {0, 6}) {
    ResearchConfig r;
    r.reset_persist_cycles = K;
    r.reset_model = true;
    r.reset_model_theta = 0.15;
    const DwaPlannerX p(deployed(), r);
    DwaInputX in;
    in.pose = path.front();
    in.path = &path;
    int first = -1;
    ResetReason why = ResetReason::kNone;
    for (int k = 0; k < 60 && first < 0; ++k) {
      in.v_meas = 0.0;
      in.w_meas = 0.0;
      const DwaResultX res = p.compute(
        in, [](const Pose2D &) {return 0.0;},
        [](double, double) {return 0.0;});
      if (res.center.reason != ResetReason::kNone) {
        first = k;
        why = res.center.reason;
      }
      in.v_last = res.v;
      in.w_last = res.w;
      in.has_last = true;
      in.research = res.next;
    }
    std::printf(
      "[F1 blocked] K %d: 첫 재설정 주기 %d (이유 %s)\n", K, first,
      amr_nav_research::resetReasonName(why));
    ASSERT_GT(first, 0) << "K " << K;
    EXPECT_LT(first, 30) << "K " << K;
    // K = 0 이면 모형 후보 즉시. K = 6 이면 K 주기 지속(persist) 이나, 그 전에 명령–측정 차가
    // reset_hard_v 를 넘으면 hard (원래 차로 판정하는 와인드업 방지 — 이 장면은 hard 가 먼저)
    if (K == 0) {
      EXPECT_EQ(why, ResetReason::kThreshold);
    } else {
      EXPECT_TRUE(why == ResetReason::kPersist || why == ResetReason::kHard);
    }
  }
}

TEST(RoundTwoF1, HardRuleStaysOnTheCommandGap)
{
  ResearchConfig r = x03();
  r.reset_model = true;
  const DwaPlannerX p(deployed(), r);
  DwaInput in;
  in.has_last = true;
  in.v_last = 0.9;
  in.v_meas = 0.25;   // |v_last − v_meas| 0.65 > hard 0.6
  ResearchState st;
  st.model.init = true;
  st.model.sv = 0.25;   // 모형은 측정과 같다 → 모형 후보 아님
  st.model.sw = 0.0;
  const auto c = p.windowCenterX(in, st);
  EXPECT_EQ(c.reason, ResetReason::kHard);
  EXPECT_DOUBLE_EQ(c.v_c, 0.25);
  // 원래 차가 hard 아래이고 모형도 맞으면 직전 명령 중심 (원래 규칙이면 재설정 후보였던 0.45)
  in.v_last = 0.70;
  const auto d = p.windowCenterX(in, st);
  EXPECT_EQ(d.reason, ResetReason::kNone);
  EXPECT_FALSE(d.suppressed);
  EXPECT_DOUBLE_EQ(d.v_c, 0.70);
  EXPECT_DOUBLE_EQ(d.v_hat, 0.25);
  // ω 임계: 기본 −1 이면 θ·window_reset_w/window_reset_v = 0.15·0.5/0.3 = 0.25 rad/s
  in.w_last = 0.0;
  in.w_meas = 0.24;
  EXPECT_EQ(p.windowCenterX(in, st).persist_count, 0);
  in.w_meas = 0.26;
  EXPECT_EQ(p.windowCenterX(in, st).persist_count, 1);
}

// ---------------------------------------------------------------------------------------------
// F2

TEST(RoundTwoF2, GoalExtensionReducesEntryYawError)
{
  // 목표 직전에 옆으로 비껴 들어오는 경로들: S 자 끝, 원호 끝, 옆으로 비낀 출발점
  std::vector<std::vector<Pose2D>> paths;
  std::vector<Pose2D> starts;
  for (double rad : {1.5, 2.5, 4.0}) {
    for (double ang : {0.4, -0.7, 1.2}) {
      std::vector<Pose2D> p{{0.0, 0.0, 0.0}};
      appendStraight(p, 2.0);
      appendArc(p, rad, ang);
      appendStraight(p, 0.4);
      paths.push_back(p);
      starts.push_back({1e9, 0.0, 0.0});
    }
  }
  for (double off : {0.12, -0.2, 0.3}) {
    paths.push_back(straight(3.0));
    starts.push_back({0.0, off, 0.0});
  }
  double sum_off = 0.0, sum_on = 0.0;
  int big_off = 0, big_on = 0, n = 0;
  for (std::size_t i = 0; i < paths.size(); ++i) {
    ResearchConfig r = x03();
    const LoopRun a = chainRun(deployed(), r, paths[i], 600, nullptr, starts[i]);
    r.goal_extension_dist = 1.0;
    const LoopRun b = chainRun(deployed(), r, paths[i], 600, nullptr, starts[i]);
    ASSERT_TRUE(a.reached) << i;
    ASSERT_TRUE(b.reached) << i;
    ASSERT_GE(a.yaw_entry, 0.0);
    ASSERT_GE(b.yaw_entry, 0.0);
    sum_off += a.yaw_entry;
    sum_on += b.yaw_entry;
    big_off += a.yaw_entry > 0.05;
    big_on += b.yaw_entry > 0.05;
    ++n;
  }
  std::printf(
    "[F2 chain] 진입 yaw 오차 평균 끔 %.4f 켬 %.4f rad, > 0.05 rad 끔 %d/%d 켬 %d/%d\n",
    sum_off / n, sum_on / n, big_off, n, big_on, n);
  EXPECT_LT(sum_on, sum_off);
  EXPECT_LE(big_on, big_off);
}

TEST(RoundTwoF2, GoalDistanceAndApproachCapAreUnchanged)
{
  const auto path = straight(3.0);
  DwaInputX in;
  in.pose = {2.4, 0.15, 0.3};   // 목표 0.6 m 앞, 옆으로 0.15 m
  in.path = &path;
  in.has_last = true;
  in.v_last = in.v_meas = 0.4;
  ResearchConfig r;
  r.goal_extension_dist = 1.0;
  auto fp = [](const Pose2D &) {return 0.0;};
  auto pc = [](double, double) {return 0.0;};
  const DwaResultX on = DwaPlannerX(deployed(), r).compute(in, fp, pc);
  const DwaResultX off = DwaPlannerX(deployed()).compute(in, fp, pc);
  EXPECT_TRUE(on.goal_ext);
  EXPECT_FALSE(off.goal_ext);
  EXPECT_DOUBLE_EQ(on.d_goal, off.d_goal);
  EXPECT_DOUBLE_EQ(on.v_cap, off.v_cap);
  EXPECT_DOUBLE_EQ(on.window.v_hi, off.window.v_hi);
  // 목표점이 연장선 위라 경로 쪽(−y)으로 덜 꺾고 목표 yaw(0) 쪽을 향한다:
  // 끄면 목표점(3, 0)을 향해 돌고, 켜면 연장선 위 (3 + …, 0) 을 향해 덜 돈다
  EXPECT_LE(
    std::abs(core::wrapAngle(on.best.poses.back().theta)),
    std::abs(core::wrapAngle(off.best.poses.back().theta)) + 1e-9);
  // 목표에서 D 밖이면 쓰지 않는다
  in.pose = {1.5, 0.0, 0.0};
  EXPECT_FALSE(DwaPlannerX(deployed(), r).compute(in, fp, pc).goal_ext);
}

// ---------------------------------------------------------------------------------------------
// F3

TEST(RoundTwoF3, DetourPathAroundADisc)
{
  const World w({{5.0, 0.05, 0.3}});
  const auto chk = w.checker();
  const auto view = w.grid.view();
  auto fp = [&chk](const Pose2D & p) {return chk.cost(p);};
  auto pc = [view](double x, double y) {return static_cast<double>(view.costAtWorld(x, y));};
  const auto path = straight(9.0);
  const std::vector<double> cum = core::cumulativeLength(path);
  ResearchConfig r;
  r.blocked_detour_shift = 1.0;
  const DwaPlannerX p(deployed(), r);
  std::vector<Pose2D> out;
  DetourInfo info;
  // 로봇 x = 3 → 원판(4.7–5.3)이 앞 3 m 안
  ASSERT_TRUE(p.detourPath(path, cum, {3.0, 0.0, 0.0}, true, fp, pc, ResearchState{}, &out, &info));
  EXPECT_TRUE(info.blocked);
  EXPECT_TRUE(info.active);
  EXPECT_GT(std::abs(info.shift), 0.3);
  EXPECT_LE(std::abs(info.shift), 1.0 + 1e-9);
  EXPECT_EQ(info.side, -1);   // 원판이 왼쪽(+y)으로 0.05 비껴 있다 → 오른쪽이 비용이 낮다
  ASSERT_EQ(out.size(), path.size());
  // 원판 옆 점은 d 만큼, 목표·출발 근처는 그대로, 우회 경로 위 발자국은 모두 비충돌
  const std::size_t i5 = 100;
  EXPECT_NEAR(out[i5].y, info.shift, 1e-9);
  EXPECT_DOUBLE_EQ(out.back().x, path.back().x);
  EXPECT_DOUBLE_EQ(out.back().y, path.back().y);
  for (std::size_t i = 60; i < out.size(); ++i) {
    EXPECT_GE(fp(out[i]), 0.0) << i;
  }
  // 막히지 않았으면 만들지 않는다 (원판 뒤로 지나간 뒤, 기억 없음)
  EXPECT_FALSE(
    p.detourPath(path, cum, {7.5, 0.0, 0.0}, true, fp, pc, ResearchState{}, &out, &info));
  EXPECT_FALSE(info.blocked);
}

TEST(RoundTwoF3, BothSidesBlockedKeepsTheOriginalPath)
{
  // 폭 1.2 m 통로(벽 |y| ≥ 0.6) 한가운데 원판 → 어느 쪽으로도 1.0 m 안에 비충돌 우회가 없다
  const World w({{5.0, 0.0, 0.3}}, 0.6);
  const auto chk = w.checker();
  const auto view = w.grid.view();
  auto fp = [&chk](const Pose2D & p) {return chk.cost(p);};
  auto pc = [view](double x, double y) {return static_cast<double>(view.costAtWorld(x, y));};
  const auto path = straight(9.0);
  const std::vector<double> cum = core::cumulativeLength(path);
  ResearchConfig r;
  r.blocked_detour_shift = 1.0;
  std::vector<Pose2D> out;
  DetourInfo info;
  EXPECT_FALSE(
    DwaPlannerX(deployed(), r).detourPath(
      path, cum, {3.0, 0.0, 0.0}, true, fp, pc, ResearchState{}, &out, &info));
  EXPECT_TRUE(info.blocked);
  EXPECT_FALSE(info.active);
}

TEST(RoundTwoF3, ClosedLoopPassesTheDiscWithoutContact)
{
  const auto path = straight(9.0);
  for (const Disc d : {Disc{5.0, 0.05, 0.3}, Disc{5.0, -0.1, 0.4}, Disc{4.5, 0.1, 0.25}}) {
    const World w({d});
    ResearchConfig r = x03();
    const LoopRun off = chainRun(deployed(), r, path, 500, &w);
    r.blocked_detour_shift = 0.6;
    const LoopRun s06 = chainRun(deployed(), r, path, 500, &w);
    r.blocked_detour_shift = 1.0;
    const LoopRun s10 = chainRun(deployed(), r, path, 500, &w);
    std::printf(
      "[F3 chain] 원판 (%.2f, %.2f, r %.2f): 끔 도달 %d x %.2f / S0.6 도달 %d 우회 %d 주기 "
      "쪽 바뀜 %d / S1.0 도달 %d 우회 %d 주기\n", d.x, d.y, d.r, off.reached, off.x_end,
      s06.reached, s06.detour, s06.side_flips, s10.reached, s10.detour);
    EXPECT_FALSE(off.reached);      // 원래 DWA 는 원판 앞에서 선다 (O 장면 1 라운드 결과)
    EXPECT_FALSE(off.collided);
    for (const LoopRun * x : {&s06, &s10}) {
      EXPECT_TRUE(x->reached);
      EXPECT_FALSE(x->collided);
      EXPECT_GT(x->detour, 0);
      EXPECT_EQ(x->side_flips, 0);
    }
  }
}
