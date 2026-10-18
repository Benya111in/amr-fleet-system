// 3 라운드 스위치 시험 (prereg_r3.md §1): G1 accel_hold — 가속 국면 재설정 유예.
//   - 조건 (a) 명령 증가 (b) 명령 > 측정 (c) |차| ≤ reset_hard_v 일 때만 선속도 후보를 세지 않는다.
//   - 가속 중 측정이 지연돼도 (chain 사슬, F1 끔) 재설정이 없다.
//   - 막힘 (측정 0 고정) 에서는 hard 로 재설정한다.
//   - 감속 중에는 개입하지 않는다 (G1 을 끈 것과 같은 판정).
//   - ω 후보는 기존 규칙 그대로 센다.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "amr_nav_research/dwa_x.hpp"
#include "amr_nav_research/research_io.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"

namespace core = amr_navigation::core;
using amr_nav_research::DwaInputX;
using amr_nav_research::DwaPlannerX;
using amr_nav_research::DwaResultX;
using amr_nav_research::GoalCheckEmu;
using amr_nav_research::ResearchConfig;
using amr_nav_research::ResearchState;
using amr_nav_research::ResetReason;
using amr_nav_research::WindowCenterX;
using core::DwaConfig;
using core::DwaInput;
using core::Pose2D;
using core::integrateArc;

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

ResearchConfig g1(int K)
{
  ResearchConfig r;
  r.reset_persist_cycles = K;
  r.accel_hold = true;
  return r;
}

std::vector<Pose2D> straight(double len)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    p.push_back({s, 0.0, 0.0});
  }
  return p;
}

DwaInput gap(double v_last, double v_meas, double w_last = 0.0, double w_meas = 0.0)
{
  DwaInput in;
  in.has_last = true;
  in.v_last = v_last;
  in.v_meas = v_meas;
  in.w_last = w_last;
  in.w_meas = w_meas;
  return in;
}

ResearchState prev(double v_last_prev)
{
  ResearchState s;
  s.v_last_prev = v_last_prev;
  s.has_v_last_prev = true;
  return s;
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

struct Log
{
  int resets{0};
  int resets_while_rising{0};   // 명령이 증가 중((a))이던 주기의 재설정
  int held{0};
  int held_while_falling{0};    // 명령이 줄어드는 주기의 유예 — 0 이어야 한다
  double t_v90{-1.0};
  bool reached{false};
};

// chain_sim 과 같은 시각표 (제어 20 Hz, 프로파일러 50 Hz, 서보 100 Hz). noise: 오도메트리 σ
Log chainRun(
  const DwaConfig & c, const ResearchConfig & r, const std::vector<Pose2D> & path, int cycles,
  double noise = 0.0, unsigned seed = 1)
{
  const DwaPlannerX dwa(c, r);
  core::VelocityProfiler prof;
  Servo sv(0.08, 0.04, 1.0);
  Servo sw(0.08, 0.04, 2.0);
  std::mt19937 gen(seed);
  std::normal_distribution<double> nd(0.0, 1.0);
  DwaInputX in;
  in.pose = path.front();
  in.path = &path;
  in.path_end_is_goal = true;
  Log log;
  double pv = 0.0, pw = 0.0, vc = 0.0, wc = 0.0, v_prev_cmd = 0.0;
  bool have_prev = false;
  GoalCheckEmu goal(0.10, 0.05, false);
  const Pose2D g = path.back();
  for (int k = 0; k < cycles; ++k) {
    const double dg = std::hypot(in.pose.x - g.x, in.pose.y - g.y);
    if (goal.reached(dg, core::wrapAngle(g.theta - in.pose.theta))) {
      log.reached = true;
      break;
    }
    const DwaResultX res = dwa.compute(
      in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
    const bool rising = in.has_last && have_prev && in.v_last > v_prev_cmd;
    const bool falling = in.has_last && have_prev && in.v_last < v_prev_cmd;
    if (res.center.reason != ResetReason::kNone) {
      ++log.resets;
      log.resets_while_rising += rising && in.v_last > in.v_meas;
    }
    log.held += res.center.accel_held;
    log.held_while_falling += res.center.accel_held && falling;
    have_prev = in.has_last;
    v_prev_cmd = in.v_last;
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
      if (log.t_v90 < 0.0 && sv.value_ >= 0.9) {
        log.t_v90 = k * 0.05 + (sub + 1) * 0.01;
      }
    }
    in.v_meas = sv.value_ + noise * nd(gen);
    in.w_meas = sw.value_ + noise * nd(gen);
  }
  return log;
}
}  // namespace

TEST(RoundThreeG1, HoldsOnlyWhenAllThreeConditionsHold)
{
  const DwaPlannerX on(deployed(), g1(0));
  const DwaPlannerX off(deployed());
  // 가속 중: 명령 0.75 → 0.80 증가, 측정 0.45 (차 0.35 > 0.3, ≤ 0.6)
  const DwaInput acc = gap(0.80, 0.45);
  WindowCenterX c = on.windowCenterX(acc, prev(0.75));
  EXPECT_TRUE(c.accel_held);
  EXPECT_EQ(c.reason, ResetReason::kNone);
  EXPECT_FALSE(c.measured);
  EXPECT_DOUBLE_EQ(c.v_c, 0.80);
  EXPECT_EQ(c.persist_count, 0);   // 연속 계수도 늘지 않는다
  EXPECT_EQ(off.windowCenterX(acc, prev(0.75)).reason, ResetReason::kThreshold);
  // (a) 실패: 명령이 그대로 (상한 도달) → 재설정
  c = on.windowCenterX(acc, prev(0.80));
  EXPECT_FALSE(c.accel_held);
  EXPECT_EQ(c.reason, ResetReason::kThreshold);
  // (a) 판단 자료가 없으면 (직전 두 주기 명령 모름) 유예하지 않는다
  EXPECT_EQ(on.windowCenterX(acc, ResearchState{}).reason, ResetReason::kThreshold);
  // (b) 실패: 명령 < 측정 (명령은 증가 중이어도)
  c = on.windowCenterX(gap(0.25, 0.60), prev(0.20));
  EXPECT_FALSE(c.accel_held);
  EXPECT_EQ(c.reason, ResetReason::kThreshold);
  // (c) 실패: 차 0.65 > reset_hard_v 0.6 → G1 이 켜져 있으면 K 와 무관하게 hard
  c = on.windowCenterX(gap(0.90, 0.25), prev(0.85));
  EXPECT_FALSE(c.accel_held);
  EXPECT_EQ(c.reason, ResetReason::kHard);
  EXPECT_DOUBLE_EQ(c.v_c, 0.25);
  // ω 후보는 그대로: 선속도는 유예 조건이어도 ω 차 0.6 > 0.5 면 재설정
  c = on.windowCenterX(gap(0.80, 0.45, 0.8, 0.2), prev(0.75));
  EXPECT_FALSE(c.accel_held);
  EXPECT_EQ(c.reason, ResetReason::kThreshold);
}

TEST(RoundThreeG1, AlsoHoldsTheChainModelCandidate)
{
  ResearchConfig r = g1(6);
  r.reset_model = true;
  r.reset_model_theta = 0.10;
  const DwaPlannerX p(deployed(), r);
  ResearchState st = prev(0.55);
  st.model.init = true;
  st.model.sv = 0.50;   // 측정 0.30 은 모형 예상보다 0.20 낮다 (θ 0.10 초과)
  st.model.sw = 0.0;
  const WindowCenterX c = p.windowCenterX(gap(0.60, 0.30), st);
  EXPECT_TRUE(c.accel_held);
  EXPECT_FALSE(c.suppressed);
  EXPECT_EQ(c.persist_count, 0);
  EXPECT_DOUBLE_EQ(c.v_c, 0.60);
  ResearchConfig r0 = r;
  r0.accel_hold = false;
  EXPECT_TRUE(DwaPlannerX(deployed(), r0).windowCenterX(gap(0.60, 0.30), st).suppressed);
}

TEST(RoundThreeG1, NoResetWhileAcceleratingInTheChain)
{
  // F1 끔. 정지 출발 직선: 원래 규칙은 가속 중 명령–측정 차가 임계를 넘어 재설정한다
  const auto path = straight(20.0);
  ResearchConfig x03;
  x03.reset_persist_cycles = 6;
  const Log base0 = chainRun(deployed(), ResearchConfig{}, path, 300);
  const Log base6 = chainRun(deployed(), x03, path, 300);
  const Log on0 = chainRun(deployed(), g1(0), path, 300);
  const Log on6 = chainRun(deployed(), g1(6), path, 300);
  std::printf(
    "[G1 chain] 재설정 K0 %d→%d (증가 중 %d→%d), K6 %d→%d, 유예 K0 %d K6 %d, "
    "t_v90 K0 %.2f→%.2f K6 %.2f→%.2f s\n", base0.resets, on0.resets, base0.resets_while_rising,
    on0.resets_while_rising, base6.resets, on6.resets, on0.held, on6.held, base0.t_v90, on0.t_v90,
    base6.t_v90, on6.t_v90);
  EXPECT_GT(base0.resets_while_rising, 0);
  EXPECT_EQ(on0.resets_while_rising, 0);   // 가속 중(명령 증가·명령 > 측정)에는 재설정 없음
  EXPECT_GT(base6.resets, 0);
  EXPECT_EQ(on6.resets, 0);                // R1 K6 과 함께면 상한 도달 뒤에도 없다
  EXPECT_GT(on6.held, 0);
  EXPECT_GT(on6.t_v90, 0.0);
  EXPECT_LT(on6.t_v90, base6.t_v90);
}

TEST(RoundThreeG1, BlockedRobotIsResetByTheHardRule)
{
  const auto path = straight(8.0);
  for (int K : {0, 6}) {
    const DwaPlannerX p(deployed(), g1(K));
    DwaInputX in;
    in.pose = path.front();
    in.path = &path;
    int first = -1;
    ResetReason why = ResetReason::kNone;
    double v_last_at = 0.0;
    for (int k = 0; k < 80 && first < 0; ++k) {
      in.v_meas = 0.0;   // 막힘: 측정 0 고정
      in.w_meas = 0.0;
      const DwaResultX res = p.compute(
        in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
      if (res.center.reason != ResetReason::kNone) {
        first = k;
        why = res.center.reason;
        v_last_at = in.v_last;
      }
      in.v_last = res.v;
      in.w_last = res.w;
      in.has_last = true;
      in.research = res.next;
    }
    std::printf(
      "[G1 blocked] K %d: 첫 재설정 주기 %d, 이유 %s, 그때 명령 %.3f m/s\n", K, first,
      amr_nav_research::resetReasonName(why), v_last_at);
    ASSERT_GT(first, 0) << K;
    EXPECT_EQ(why, ResetReason::kHard) << K;
    EXPECT_GT(v_last_at, 0.6);           // 차가 reset_hard_v 를 넘은 첫 주기
    EXPECT_LT(v_last_at, 0.6 + 0.05 + 1e-9);
  }
}

TEST(RoundThreeG1, DoesNotInterveneWhileDecelerating)
{
  // 1) 명령이 줄거나 같은 상태(v_last ≤ v_last_prev)에서는 무작위 차·K 에서 G1 을 끈 판정과 같다
  std::mt19937 g(5U);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int K : {0, 6}) {
    ResearchConfig rk;
    rk.reset_persist_cycles = K;
    const DwaPlannerX off(deployed(), rk);
    const DwaPlannerX on(deployed(), g1(K));
    for (int i = 0; i < 2000; ++i) {
      const double v_last = u(g);
      const DwaInput in = gap(
        v_last, std::max(0.0, v_last + 0.9 * (u(g) - 0.5)), 1.4 * (u(g) - 0.5),
        1.4 * (u(g) - 0.5));
      ResearchState st = prev(v_last + 0.1 * u(g));
      st.persist_count = static_cast<int>(u(g) * 7.0);
      const WindowCenterX a = off.windowCenterX(in, st);
      const WindowCenterX b = on.windowCenterX(in, st);
      ASSERT_FALSE(b.accel_held) << i;
      ASSERT_DOUBLE_EQ(a.v_c, b.v_c) << i;
      ASSERT_DOUBLE_EQ(a.w_c, b.w_c) << i;
      ASSERT_EQ(a.persist_count, b.persist_count) << i;
      if (K > 0) {
        ASSERT_EQ(a.reason, b.reason) << i;
      } else {   // K = 0 이면 차 > reset_hard_v 의 이름만 threshold → hard
        ASSERT_EQ(a.reason == ResetReason::kNone, b.reason == ResetReason::kNone) << i;
      }
    }
  }
  // 2) 폐루프 (오도메트리 잡음 σ 0.02, 목표 접근 감속 포함): 명령이 줄어드는 주기에는 한 번도
  //    유예하지 않는다
  int falls = 0;
  for (unsigned seed : {1U, 2U, 3U}) {
    const Log on = chainRun(deployed(), g1(6), straight(6.0), 400, 0.02, seed);
    EXPECT_TRUE(on.reached) << seed;
    EXPECT_EQ(on.held_while_falling, 0) << seed;
    falls += on.held_while_falling;
  }
  EXPECT_EQ(falls, 0);
}

TEST(RoundThreeG1, StateCarriesThePreviousCommand)
{
  const auto path = straight(6.0);
  DwaInputX in;
  in.pose = path.front();
  in.path = &path;
  in.has_last = true;
  in.v_last = 0.42;
  in.v_meas = 0.40;
  const DwaResultX r = DwaPlannerX(deployed(), g1(6)).compute(
    in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
  EXPECT_TRUE(r.next.has_v_last_prev);
  EXPECT_DOUBLE_EQ(r.next.v_last_prev, 0.42);   // 다음 주기의 v_last(= r.v) 와 비교할 직전 값
  in.has_last = false;
  const DwaResultX s = DwaPlannerX(deployed(), g1(6)).compute(
    in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
  EXPECT_FALSE(s.next.has_v_last_prev);
}
