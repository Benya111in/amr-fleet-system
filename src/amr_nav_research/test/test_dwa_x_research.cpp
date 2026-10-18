// 연구 스위치 단위 시험 (prereg.md §2, contract.md §1):
//   R1 재설정 지속 (K 주기 연속, hard 즉시, 연속 수 초기화), R2 저크 띠 (교집합·제동 예외·
//   각저크는 제자리 회전 영역만), R3 순수 추종 후보 (후보 수·곡률·폐루프 선택),
//   stateful 목표 판정 잠금, 재설정 기록 CSV·국면.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "amr_nav_research/dwa_x.hpp"
#include "amr_nav_research/research_io.hpp"
#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"

namespace core = amr_navigation::core;
using amr_nav_research::DwaInputX;
using amr_nav_research::DwaPlannerX;
using amr_nav_research::DwaResultX;
using amr_nav_research::GoalCheckEmu;
using amr_nav_research::ResearchConfig;
using amr_nav_research::ResearchState;
using amr_nav_research::ResetLogger;
using amr_nav_research::ResetReason;
using amr_nav_research::WindowCenterX;
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
  return c;
}

std::vector<Pose2D> straight(double len, double y = 0.0)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    p.push_back({s, y, 0.0});
  }
  return p;
}

std::vector<Pose2D> arcPath(double radius, double angle)
{
  std::vector<Pose2D> p{{0.0, 0.0, 0.0}};
  const int n = static_cast<int>(std::lround(angle * radius / 0.05));
  for (int i = 1; i <= n; ++i) {
    p.push_back(integrateArc(p.front(), 1.0, 1.0 / radius, i * 0.05));
  }
  return p;
}

std::vector<Pose2D> sCurve()
{
  std::vector<Pose2D> c{{0.0, 0.0, 0.0}};
  auto arc = [&c](double r, double a) {
      const Pose2D o = c.back();
      const int n = static_cast<int>(std::lround(std::abs(a) * r / 0.05));
      for (int i = 1; i <= n; ++i) {
        c.push_back(integrateArc(o, 1.0, (a >= 0.0 ? 1.0 : -1.0) / r, i * 0.05));
      }
    };
  for (int k = 0; k < 2; ++k) {
    arc(3.0, kPi / 6.0);
    arc(3.0, -kPi / 3.0);
    arc(3.0, kPi / 6.0);
  }
  const Pose2D o = c.back();
  for (int i = 1; i <= 20; ++i) {
    c.push_back(integrateArc(o, 1.0, 0.0, i * 0.05));
  }
  return c;
}

DwaInput gapInput(double v_last, double v_meas, double w_last = 0.0, double w_meas = 0.0)
{
  DwaInput in;
  in.has_last = true;
  in.v_last = v_last;
  in.v_meas = v_meas;
  in.w_last = w_last;
  in.w_meas = w_meas;
  return in;
}

auto kFree = [](const Pose2D &) {return 0.0;};
auto kZero = [](double, double) {return 0.0;};

// chain_sim 의 서보 한 축: 순수 지연 → 1차 지연 → 가속 제한 (tools/chain_sim.cpp Servo 와 같다)
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
  double output() const {return value_;}
  double tau_;
  double a_max_;
  std::vector<double> buf_;
  double value_{0.0};
};

// 체인 폐루프 (프로파일러 50 Hz + chain_sim 서보: 지연 0.04 s, 1차 0.08 s, 가속 제한) 의 기록
struct LoopLog
{
  std::vector<double> v_cmd;
  std::vector<double> a_next;
  std::vector<bool> brake;
  std::vector<bool> band_ok;      // R2 띠와 가속 창의 교집합이 비지 않았던 주기
  int resets{0};
  int suppressed{0};
  int pp_selected{0};
  double t_v90{-1.0};
  bool reached{false};
};

LoopLog chainLoop(
  const DwaConfig & c, const ResearchConfig & r, const std::vector<Pose2D> & path, int cycles)
{
  const DwaPlannerX dwa(c, r);
  core::VelocityProfiler prof;
  Servo sv(0.08, 0.04, 1.0);
  Servo sw(0.08, 0.04, 2.0);
  DwaInputX in;
  in.pose = path.front();
  in.path = &path;
  in.path_end_is_goal = true;
  LoopLog log;
  double pv = 0.0, pw = 0.0, vc = 0.0, wc = 0.0;
  GoalCheckEmu goal(0.10, 0.05, false);
  bool done = false;
  for (int k = 0; k < cycles && !done; ++k) {
    const double dg = std::hypot(in.pose.x - path.back().x, in.pose.y - path.back().y);
    if (goal.reached(dg, core::wrapAngle(path.back().theta - in.pose.theta))) {
      done = true;
      log.reached = true;
      vc = 0.0;
      wc = 0.0;
    } else {
      const DwaResultX res = dwa.compute(in, kFree, kZero);
      const ResearchState st = in.has_last ? in.research : ResearchState{};
      const double dt = c.control_period;
      const double dj = r.jerk_lim * dt;
      const double lo = res.center.v_c + (st.a_last - dj) * dt;
      const double hi = res.center.v_c + (st.a_last + dj) * dt;
      log.band_ok.push_back(std::max(lo, res.window.v_lo) <= std::min(hi, res.window.v_hi));
      log.v_cmd.push_back(res.v);
      log.a_next.push_back(res.next.a_last);
      log.brake.push_back(res.best.is_brake);
      log.resets += res.center.reason != ResetReason::kNone;
      log.suppressed += res.center.suppressed;
      log.pp_selected += res.pp_selected;
      vc = res.v;
      wc = res.w;
      in.v_last = res.v;
      in.w_last = res.w;
      in.has_last = true;
      in.research = res.next;
    }
    for (int sub = 0; sub < 5; ++sub) {
      // chain_sim 과 같은 시각표: 0.01 s 플랜트 스텝의 전역 번호가 짝수일 때 프로파일러 (50 Hz)
      if ((k * 5 + sub) % 2 == 0) {
        const auto o = prof.update(vc, wc, true, sv.output(), sw.output(), 0.02);
        pv = o.v;
        pw = o.w;
      }
      in.pose = integrateArc(in.pose, sv.step(pv), sw.step(pw), 0.01);
      if (log.t_v90 < 0.0 && sv.output() >= 0.9) {
        log.t_v90 = k * 0.05 + (sub + 1) * 0.01;
      }
    }
    in.v_meas = sv.output();
    in.w_meas = sw.output();
  }
  return log;
}

double jerkP99(const std::vector<double> & v, double dt)
{
  std::vector<double> j;
  for (std::size_t k = 2; k < v.size(); ++k) {
    const double a1 = (v[k] - v[k - 1]) / dt;
    const double a0 = (v[k - 1] - v[k - 2]) / dt;
    j.push_back(std::abs(a1 - a0) / dt);
  }
  std::sort(j.begin(), j.end());
  return j.empty() ? 0.0 : j[static_cast<std::size_t>(0.99 * static_cast<double>(j.size() - 1))];
}
}  // namespace

// ---------------------------------------------------------------------------------------------
// R1

TEST(ResearchR1, DefaultRuleResetsImmediatelyAndCountsTheRun)
{
  const DwaPlannerX p(deployed());
  ResearchState st;
  const DwaInput in = gapInput(0.80, 0.45);   // 차 0.35 > 0.3
  WindowCenterX c = p.windowCenterX(in, st);
  EXPECT_TRUE(c.measured);
  EXPECT_EQ(c.reason, ResetReason::kThreshold);
  EXPECT_DOUBLE_EQ(c.v_c, 0.45);
  EXPECT_EQ(c.persist_count, 1);
  st.persist_count = c.persist_count;
  c = p.windowCenterX(in, st);
  EXPECT_EQ(c.persist_count, 2);   // K = 0 이어도 연속 수는 센다 (기록용)
  // 차가 임계 안: 직전 명령, 연속 수 0
  c = p.windowCenterX(gapInput(0.80, 0.60), st);
  EXPECT_FALSE(c.measured);
  EXPECT_EQ(c.reason, ResetReason::kNone);
  EXPECT_EQ(c.persist_count, 0);
  EXPECT_DOUBLE_EQ(c.v_c, 0.80);
}

TEST(ResearchR1, PersistenceDelaysTheResetForKCycles)
{
  ResearchConfig r;
  r.reset_persist_cycles = 3;
  const DwaPlannerX p(deployed(), r);
  const DwaInput in = gapInput(0.80, 0.45, 0.1, 0.0);
  ResearchState st;
  for (int k = 1; k <= 2; ++k) {
    const WindowCenterX c = p.windowCenterX(in, st);
    EXPECT_TRUE(c.suppressed) << k;
    EXPECT_FALSE(c.measured) << k;
    EXPECT_EQ(c.reason, ResetReason::kNone) << k;
    EXPECT_DOUBLE_EQ(c.v_c, 0.80) << k;   // 직전 명령 중심 유지
    EXPECT_EQ(c.persist_count, k);
    st.persist_count = c.persist_count;
  }
  const WindowCenterX c3 = p.windowCenterX(in, st);
  EXPECT_TRUE(c3.measured);
  EXPECT_EQ(c3.reason, ResetReason::kPersist);
  EXPECT_DOUBLE_EQ(c3.v_c, 0.45);
  EXPECT_DOUBLE_EQ(c3.w_c, 0.0);
  // 차가 계속 남아 있으면 그다음 주기도 측정값 (K 주기마다가 아니라 계속)
  st.persist_count = c3.persist_count;
  EXPECT_EQ(p.windowCenterX(in, st).reason, ResetReason::kPersist);
  // 차가 한 번 임계 안으로 들어오면 연속 수는 처음부터
  st.persist_count = p.windowCenterX(gapInput(0.80, 0.70), st).persist_count;
  EXPECT_EQ(st.persist_count, 0);
  EXPECT_TRUE(p.windowCenterX(in, st).suppressed);
}

TEST(ResearchR1, HardGapResetsImmediately)
{
  ResearchConfig r;
  r.reset_persist_cycles = 6;
  r.reset_hard_v = 0.6;
  const DwaPlannerX p(deployed(), r);
  const WindowCenterX c = p.windowCenterX(gapInput(1.0, 0.35), ResearchState{});   // 차 0.65
  EXPECT_EQ(c.reason, ResetReason::kHard);
  EXPECT_DOUBLE_EQ(c.v_c, 0.35);
  // 음의 차(명령 < 측정)도 크기로 본다
  EXPECT_EQ(p.windowCenterX(gapInput(0.1, 0.75), ResearchState{}).reason, ResetReason::kHard);
  // 각속도 차만 크면 hard 가 아니다 (contract: hard 는 선속도 차)
  EXPECT_TRUE(p.windowCenterX(gapInput(0.5, 0.5, 1.0, 0.0), ResearchState{}).suppressed);
}

TEST(ResearchR1, NoLastCommandOrMeasuredAblationIsNotAResetEvent)
{
  ResearchConfig r;
  r.reset_persist_cycles = 4;
  DwaConfig c = deployed();
  DwaInput in = gapInput(0.9, 0.2);
  in.has_last = false;
  ResearchState st;
  st.persist_count = 3;
  WindowCenterX w = DwaPlannerX(c, r).windowCenterX(in, st);
  EXPECT_TRUE(w.measured);
  EXPECT_EQ(w.reason, ResetReason::kNone);
  EXPECT_EQ(w.persist_count, 0);
  c.window_center_measured = true;
  in.has_last = true;
  w = DwaPlannerX(c, r).windowCenterX(in, st);
  EXPECT_TRUE(w.measured);
  EXPECT_EQ(w.reason, ResetReason::kNone);
}

TEST(ResearchR1, ComputeCarriesTheCounterThroughTheResult)
{
  // compute 는 const 이고 연속 수는 입력 research → 결과 next 로 돈다
  ResearchConfig r;
  r.reset_persist_cycles = 2;
  const DwaPlannerX p(deployed(), r);
  const auto path = straight(8.0);
  DwaInputX in;
  in.pose = path.front();
  in.path = &path;
  in.has_last = true;
  in.v_last = 0.70;
  in.v_meas = 0.35;
  const DwaResultX a = p.compute(in, kFree, kZero);
  EXPECT_TRUE(a.center.suppressed);
  EXPECT_EQ(a.next.persist_count, 1);
  EXPECT_NEAR(a.window.v_hi, std::min(1.0, 0.70 + 0.05), 1e-12);   // 직전 명령 중심 창
  in.research = a.next;
  const DwaResultX b = p.compute(in, kFree, kZero);
  EXPECT_EQ(b.center.reason, ResetReason::kPersist);
  EXPECT_NEAR(b.window.v_hi, 0.35 + 0.05, 1e-12);                    // 측정값 중심 창
}

TEST(ResearchR1, PersistenceDefersAccelerationResetsInTheChain)
{
  // 정지 출발 직선: 원본 규칙은 가속 중 명령–측정 차가 임계를 넘어 재설정한다 (results.md §6).
  // 이 루프는 chain_sim 사슬과 같은 시각표라 원본의 0 → 0.9 m/s 1.57 s 를 그대로 낸다.
  const auto path = straight(20.0);
  const LoopLog base = chainLoop(deployed(), ResearchConfig{}, path, 300);
  ResearchConfig r;
  r.reset_persist_cycles = 6;
  const LoopLog k6 = chainLoop(deployed(), r, path, 300);
  std::printf(
    "[R1 chain] resets K0 %d K6 %d (suppressed %d), t_v90 K0 %.2f K6 %.2f s\n", base.resets,
    k6.resets, k6.suppressed, base.t_v90, k6.t_v90);
  EXPECT_GT(base.resets, 0);
  EXPECT_LT(k6.resets, base.resets);
  EXPECT_GT(k6.suppressed, 0);
  // 기제만 판정한다 (가속 시간 효과는 실험이 잴 몫이다 — 여기서는 출력만 남긴다)
  EXPECT_GT(base.t_v90, 0.0);
  EXPECT_GT(k6.t_v90, 0.0);
}

// ---------------------------------------------------------------------------------------------
// R2

TEST(ResearchR2, JerkBandIntersectsTheAccelerationWindow)
{
  ResearchConfig r;
  r.jerk_consistent = true;
  r.jerk_lim = 2.0;
  const DwaPlannerX p(deployed(), r);
  const auto win = p.dynamicWindow(0.5, 0.0, 1.0);   // [0.45, 0.55]
  ResearchState st;
  st.a_last = 1.0;
  bool ang = true;
  auto j = p.jerkWindow(win, 0.5, 0.0, st, &ang);
  // 띠 = [0.5 + (1.0 − 0.1)·0.05, 0.5 + (1.0 + 0.1)·0.05] = [0.545, 0.555] → ∩ = [0.545, 0.55]
  EXPECT_NEAR(j.v_lo, 0.545, 1e-12);
  EXPECT_NEAR(j.v_hi, 0.55, 1e-12);
  EXPECT_FALSE(ang);   // 주행 중: ω 는 그대로
  EXPECT_DOUBLE_EQ(j.w_lo, win.w_lo);
  EXPECT_DOUBLE_EQ(j.w_hi, win.w_hi);
  // 교집합이 비면 창 안에서 띠에 가장 가까운 끝 (속도 상한에 붙어 가속 중)
  const auto capped = p.dynamicWindow(0.99, 0.0, 1.0);   // [0.94, 1.0]
  j = p.jerkWindow(capped, 0.99, 0.0, st);
  EXPECT_DOUBLE_EQ(j.v_lo, 1.0);
  EXPECT_DOUBLE_EQ(j.v_hi, 1.0);
  // 정지에서 감속 중이던 상태: 띠가 0 아래 → 창 바닥 0
  st.a_last = -1.0;
  const auto floor0 = p.dynamicWindow(0.01, 0.0, 1.0);
  j = p.jerkWindow(floor0, 0.01, 0.0, st);
  EXPECT_DOUBLE_EQ(j.v_lo, 0.0);
  EXPECT_DOUBLE_EQ(j.v_hi, 0.0);
}

TEST(ResearchR2, AngularBandOnlyInPlaceRotation)
{
  ResearchConfig r;
  r.jerk_consistent = true;
  const DwaConfig c = deployed();
  const DwaPlannerX p(c, r);
  ResearchState st;
  st.alpha_last = 1.0;
  bool ang = false;
  const auto win = p.dynamicWindow(0.0, 0.5, 1.0);
  const auto j = p.jerkWindow(win, 0.0, 0.5, st, &ang);
  EXPECT_TRUE(ang);   // v 창 [0, 0.005] → 하류도 제자리 회전 모드
  const double djw = c.jerk_lim_theta * c.control_period;   // 6·0.05 = 0.3 rad/s²
  EXPECT_NEAR(j.w_lo, 0.5 + (1.0 - djw) * 0.05, 1e-12);
  EXPECT_NEAR(j.w_hi, 0.5 + (1.0 + djw) * 0.05, 1e-12);
}

TEST(ResearchR2, BrakeCandidateIsExempt)
{
  ResearchConfig r;
  r.jerk_consistent = true;
  const DwaPlannerX p(deployed(), r);
  const auto path = straight(6.0);
  DwaInputX in;
  in.pose = path.front();
  in.path = &path;
  in.has_last = true;
  in.v_last = in.v_meas = 0.5;
  in.research.a_last = 1.0;   // 가속 중이었다 → 띠는 [0.545, 0.55]
  auto blocked = [](const Pose2D &) {return -1.0;};
  const DwaResultX res = p.compute(in, blocked, kZero);
  EXPECT_TRUE(res.jerk_applied);
  EXPECT_NEAR(res.sample_window.v_lo, 0.545, 1e-12);
  EXPECT_FALSE(res.found);
  EXPECT_TRUE(res.best.is_brake);
  EXPECT_NEAR(res.v, 0.45, 1e-12);    // 최대 감속 — 띠 밖이지만 제동 후보는 예외
  EXPECT_NEAR(res.next.a_last, -1.0, 1e-9);
}

TEST(ResearchR2, EmittedJerkIsBoundedAndSmallerThanTheOriginal)
{
  const auto path = sCurve();
  ResearchConfig r;
  r.jerk_consistent = true;
  const LoopLog on = chainLoop(deployed(), r, path, 600);
  const LoopLog off = chainLoop(deployed(), ResearchConfig{}, path, 600);
  ASSERT_TRUE(on.reached);
  ASSERT_TRUE(off.reached);
  int checked = 0;
  for (std::size_t k = 1; k < on.a_next.size(); ++k) {
    if (on.band_ok[k] && !on.brake[k]) {
      EXPECT_LE(std::abs(on.a_next[k] - on.a_next[k - 1]), 2.0 * 0.05 + 1e-9) << "cycle " << k;
      ++checked;
    }
  }
  EXPECT_GT(checked, 100);
  const double j_on = jerkP99(on.v_cmd, 0.05);
  const double j_off = jerkP99(off.v_cmd, 0.05);
  std::printf(
    "[R2 chain] cmd jerk p99 on %.2f off %.2f m/s^3, bounded cycles %d\n", j_on, j_off, checked);
  EXPECT_LT(j_on, j_off);
  EXPECT_LT(j_on, 0.5 * j_off);
}

// ---------------------------------------------------------------------------------------------
// R3

TEST(ResearchR3, AddsOneCandidateBeforeTheBrake)
{
  const auto path = straight(8.0);
  DwaInputX in;
  in.pose = {0.0, 0.2, 0.0};    // 경로 왼쪽 0.2 m
  in.path = &path;
  in.has_last = true;
  in.v_last = in.v_meas = 0.6;
  const DwaResultX off = DwaPlannerX(deployed()).compute(in, kFree, kZero);
  ResearchConfig r;
  r.pp_candidate = true;
  const DwaResultX on = DwaPlannerX(deployed(), r).compute(in, kFree, kZero);
  EXPECT_EQ(on.n_samples, off.n_samples + 1);
  EXPECT_TRUE(on.pp_added);
  // v* = 창으로 자른 v_des (빈 직선, 목표 먼 곳 → 창 상한 0.65), ℓ = clip(0.4·0.65 + 0.3) = 0.56
  EXPECT_NEAR(on.pp_v, 0.65, 1e-12);
  const double ell = 0.4 * 0.65 + 0.3;
  // 목표점 = 투영(0) + ℓ 의 경로 점 (ℓ, 0) → 로봇 좌표 (ℓ, −0.2)
  const double kappa = 2.0 * (-0.2) / (ell * ell + 0.04);
  EXPECT_NEAR(on.pp_kappa, kappa, 1e-9);
  EXPECT_NEAR(on.pp_w, std::clamp(0.65 * kappa, on.window.w_lo, on.window.w_hi), 1e-12);
  EXPECT_LT(on.pp_w, 0.0);   // 경로 쪽(오른쪽)으로
}

TEST(ResearchR3, CurvatureMatchesACircularPath)
{
  // 반경 2 m 원호 위, 접선 방향: 순수 추종 원호는 경로 원 자체 → κ = 0.5
  const auto path = arcPath(2.0, kPi / 2.0);
  DwaInputX in;
  in.pose = path.front();
  in.path = &path;
  in.has_last = true;
  in.v_last = in.v_meas = 0.5;
  in.w_last = in.w_meas = 0.25;
  ResearchConfig r;
  r.pp_candidate = true;
  const DwaResultX res = DwaPlannerX(deployed(), r).compute(in, kFree, kZero);
  EXPECT_NEAR(res.pp_kappa, 0.5, 2e-3);
}

TEST(ResearchR3, CandidateGoesThroughTheSameCollisionCheck)
{
  const auto path = straight(6.0);
  DwaInputX in;
  in.pose = path.front();
  in.path = &path;
  in.has_last = true;
  in.v_last = in.v_meas = 0.5;
  ResearchConfig r;
  r.pp_candidate = true;
  auto blocked = [](const Pose2D &) {return -1.0;};
  const DwaResultX res = DwaPlannerX(deployed(), r).compute(in, blocked, kZero);
  EXPECT_FALSE(res.found);
  EXPECT_FALSE(res.pp_selected);
  EXPECT_EQ(res.n_collision, res.n_samples);   // 연구 후보 포함 전부 충돌로 셌다
  EXPECT_TRUE(res.best.is_brake);
}

TEST(ResearchR3, ChainLoopSelectsTheCandidateOnCurves)
{
  ResearchConfig r;
  r.pp_candidate = true;
  const LoopLog on = chainLoop(deployed(), r, sCurve(), 600);
  ASSERT_TRUE(on.reached);
  std::printf("[R3 chain] pp candidate selected %d cycles\n", on.pp_selected);
  EXPECT_GT(on.pp_selected, 0);
}

// ---------------------------------------------------------------------------------------------
// stateful 목표 판정 (chain_sim_x)

TEST(GoalCheck, StatelessEqualsTheOriginalCondition)
{
  GoalCheckEmu g(0.10, 0.05, false);
  for (double d = 0.0; d <= 0.2; d += 0.0125) {
    for (double y = -0.1; y <= 0.1; y += 0.0125) {
      EXPECT_EQ(g.reached(d, y), d <= 0.10 && std::abs(y) <= 0.05) << d << " " << y;
    }
  }
  EXPECT_FALSE(g.xyLatched());
}

TEST(GoalCheck, StatefulLatchesXyOnceInside)
{
  GoalCheckEmu g(0.10, 0.05, true);
  EXPECT_FALSE(g.reached(0.30, 0.0));    // 밖
  EXPECT_FALSE(g.xyLatched());
  EXPECT_FALSE(g.reached(0.08, 0.20));   // 안에 들었지만 yaw 미달 → 잠금만
  EXPECT_TRUE(g.xyLatched());
  EXPECT_FALSE(g.reached(0.40, 0.10));   // 지나쳐 나가도 xy 는 이미 통과, yaw 만 본다
  EXPECT_TRUE(g.reached(0.40, 0.01));    // yaw 만 맞으면 도착 (Nav2 SimpleGoalChecker stateful)
  g.reset();                             // 새 경로 (controller_server::setPlannerPath)
  EXPECT_FALSE(g.xyLatched());
  EXPECT_FALSE(g.reached(0.40, 0.01));
}

// ---------------------------------------------------------------------------------------------
// 재설정 기록

TEST(ResetLog, WritesOnlyResetEventsWithPhase)
{
  const std::string fn = ::testing::TempDir() + "amr_nav_research_reset_log.csv";
  std::remove(fn.c_str());
  {
    ResetLogger log;
    ASSERT_TRUE(log.open(fn, "scene,arm,seed"));
    const DwaPlannerX p(deployed());
    // 가속 중 재설정: 명령이 측정보다 0.33 위
    const DwaInput acc = gapInput(0.80, 0.47);
    const WindowCenterX c1 = p.windowCenterX(acc, ResearchState{});
    EXPECT_TRUE(log.write(1.25, acc, c1, 12.5, 0.3, 0.5, "S01,dwa_full,1"));
    // 재설정 아님 → 쓰지 않는다
    const DwaInput ok = gapInput(0.80, 0.70);
    EXPECT_FALSE(
      log.write(1.30, ok, p.windowCenterX(ok, ResearchState{}), 12.4, 0.3, 0.5, "x"));
    // 감속 중 (명령 < 측정), 목표 거리 없음 → -1
    const DwaInput dec = gapInput(0.20, 0.55);
    EXPECT_TRUE(
      log.write(
        9.0, dec, p.windowCenterX(dec, ResearchState{}),
        std::numeric_limits<double>::infinity(), 0.3, 0.5, "S01,dwa_full,1"));
    // 각속도 차만: 제자리 회전 → turn
    const DwaInput turn = gapInput(0.0, 0.0, 0.9, 0.2);
    EXPECT_TRUE(
      log.write(10.0, turn, p.windowCenterX(turn, ResearchState{}), 0.05, 0.3, 0.5, "S"));
  }
  {   // 다시 열면 머리줄을 또 쓰지 않는다 (덧붙이기)
    ResetLogger log;
    ASSERT_TRUE(log.open(fn, "scene,arm,seed"));
  }
  std::ifstream f(fn);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(f, line)) {
    lines.push_back(line);
  }
  ASSERT_EQ(lines.size(), 4U);
  EXPECT_EQ(
    lines[0],
    "t,v_last,v_meas,w_last,w_meas,gap_v,gap_w,persist_count,reason,d_goal,phase,scene,arm,seed,"
    "v_hat,w_hat");
  EXPECT_EQ(
    lines[1], "1.250,0.8000,0.4700,0.0000,0.0000,0.3300,0.0000,1,threshold,12.5000,accel,"
    "S01,dwa_full,1,nan,nan");
  EXPECT_NE(lines[2].find(",threshold,-1.0000,decel,"), std::string::npos) << lines[2];
  EXPECT_NE(lines[3].find(",turn,S,nan,nan"), std::string::npos) << lines[3];
  std::remove(fn.c_str());
}

TEST(ResetLog, PhaseRule)
{
  WindowCenterX c;
  c.gap_v = 0.35;
  EXPECT_STREQ(amr_nav_research::resetPhase(c, 0.8, 0.0, 0.3, 0.5), "accel");
  c.gap_v = -0.35;
  EXPECT_STREQ(amr_nav_research::resetPhase(c, 0.3, 0.0, 0.3, 0.5), "decel");
  c.gap_v = 0.1;
  c.gap_w = 0.6;
  EXPECT_STREQ(amr_nav_research::resetPhase(c, 0.6, 0.5, 0.3, 0.5), "turn");
  EXPECT_STREQ(amr_nav_research::resetPhase(c, 0.02, 0.1, 0.3, 0.5), "turn");
  EXPECT_STREQ(amr_nav_research::resetPhase(c, 0.6, 0.1, 0.3, 0.5), "cruise");
}
