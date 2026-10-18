// prereg_r3c.md §3 E5 P4 — 와인드업 상한 (원고 3.1 절 "유예는 와인드업을 유한하게 묶는다").
//
// 코어(DwaPlannerX)에 측정 v_m = 0 을 고정 입력하고(막힘) 명령을 되먹인다. 명령이 측정보다 앞서기
// 시작한 주기(와인드업 시작, v_c^{k-1} > v_m 인 첫 주기)부터 창 재설정 주기까지
// 지난 주기 수를 잰다.
// 식: θ_h / (a·Δt_c) = 0.6 / (1.0 · 0.05) = 12 주기 (0.6 s) 이하여야 한다 (G1 켬).
// G1 을 끈 경우(임계·K 지속·모형 규칙)도 함께 출력한다 (판정 없음, 기술).
// 출력 줄 "[P4] ..." 를 하네스(analyze_r3c)가 결과에 옮긴다.
#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "amr_nav_research/dwa_x.hpp"
#include "amr_nav_research/research_io.hpp"
#include "amr_navigation/core/geometry.hpp"

namespace core = amr_navigation::core;
using amr_nav_research::DwaInputX;
using amr_nav_research::DwaPlannerX;
using amr_nav_research::DwaResultX;
using amr_nav_research::ResearchConfig;
using amr_nav_research::ResetReason;
using core::DwaConfig;
using core::Pose2D;

namespace
{
// test_dwa_x_round3.cpp 의 deployed() 와 같다 (배포 nav2_params DWA 블록의 시험용 근사)
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

std::vector<Pose2D> straight(double len)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    p.push_back({s, 0.0, 0.0});
  }
  return p;
}

struct Windup
{
  int onset{-1};            // v_last > v_meas 인 첫 주기
  int reset{-1};            // 첫 재설정 주기
  ResetReason why{ResetReason::kNone};
  double v_last_at_reset{0.0};
};

Windup blocked(const ResearchConfig & r, int max_cycles = 200)
{
  const auto path = straight(20.0);
  const DwaPlannerX p(deployed(), r);
  DwaInputX in;
  in.pose = path.front();
  in.path = &path;
  in.path_end_is_goal = true;
  Windup w;
  for (int k = 0; k < max_cycles && w.reset < 0; ++k) {
    in.v_meas = 0.0;   // 막힘: 측정 0 고정
    in.w_meas = 0.0;
    if (w.onset < 0 && in.has_last && in.v_last > in.v_meas) {
      w.onset = k;
    }
    const DwaResultX res = p.compute(
      in, [](const Pose2D &) {return 0.0;}, [](double, double) {return 0.0;});
    if (res.center.reason != ResetReason::kNone) {
      w.reset = k;
      w.why = res.center.reason;
      w.v_last_at_reset = in.v_last;
    }
    in.v_last = res.v;
    in.w_last = res.w;
    in.has_last = true;
    in.research = res.next;
  }
  return w;
}

ResearchConfig cfg(bool g1, int K, bool f1)
{
  ResearchConfig r;
  r.accel_hold = g1;
  r.reset_persist_cycles = K;
  r.reset_model = f1;
  r.reset_model_theta = 0.10;
  return r;
}

void report(const char * label, const ResearchConfig & r, const Windup & w)
{
  std::printf(
    "[P4] %s g1=%d K=%d f1=%d onset=%d reset=%d cycles=%d time_s=%.2f reason=%s "
    "v_last_at_reset=%.3f\n", label, r.accel_hold ? 1 : 0, r.reset_persist_cycles,
    r.reset_model ? 1 : 0, w.onset, w.reset, w.reset >= 0 ? w.reset - w.onset : -1,
    w.reset >= 0 ? 0.05 * (w.reset - w.onset) : -1.0,
    amr_nav_research::resetReasonName(w.why), w.v_last_at_reset);
}
}  // namespace

TEST(R3cWindup, G1OnIsBoundedByThetaHard)
{
  // θ_h / (a Δt_c) = reset_hard_v / (acc_lim_x · control_period)
  const DwaConfig c = deployed();
  const double bound = 0.6 / (c.limits.acc_lim_x * c.control_period);
  std::printf(
    "[P4] bound_cycles=%.3f (theta_h 0.6, a %.2f, dt_c %.3f)\n", bound,
    c.limits.acc_lim_x, c.control_period);
  for (int K : {0, 6, 10}) {
    for (bool f1 : {false, true}) {
      const ResearchConfig r = cfg(true, K, f1);
      const Windup w = blocked(r);
      report("g1_on", r, w);
      ASSERT_GE(w.onset, 0);
      ASSERT_GE(w.reset, 0);
      EXPECT_EQ(w.why, ResetReason::kHard);
      EXPECT_LE(w.reset - w.onset, static_cast<int>(bound + 1e-9));
    }
  }
}

TEST(R3cWindup, G1OffIsReportedToo)
{
  for (int K : {0, 6, 10}) {
    for (bool f1 : {false, true}) {
      const ResearchConfig r = cfg(false, K, f1);
      const Windup w = blocked(r);
      report("g1_off", r, w);
      EXPECT_GE(w.reset, 0);   // 어떤 규칙으로든 결국 재설정된다
    }
  }
}
