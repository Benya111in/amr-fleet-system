// 저크 제한 필터, PID(안티와인드업), 1차 지연 플랜트, 속도 프로파일러 단위 테스트.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

#include "amr_navigation/core/jerk_limiter.hpp"
#include "amr_navigation/core/pid.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"

using amr_navigation::core::FirstOrderPlant;
using amr_navigation::core::JerkLimitedFilter;
using amr_navigation::core::JerkLimits;
using amr_navigation::core::Pid;
using amr_navigation::core::PidConfig;
using amr_navigation::core::ProfilerOutput;
using amr_navigation::core::VelocityProfiler;
using amr_navigation::core::VelocityProfilerConfig;

namespace
{
constexpr double kTol = 1e-9;

// 0 → target 도달 시간과 최대 속도
std::pair<double, double> riseTime(JerkLimitedFilter & f, double target, double dt)
{
  f.reset(0.0, 0.0);
  double t = 0.0;
  double vmax = 0.0;
  for (int k = 0; k < 100000; ++k) {
    f.step(target, dt);
    t += dt;
    vmax = std::max(vmax, f.velocity());
    if (std::abs(f.velocity() - target) < 1e-12 && f.acceleration() == 0.0) {
      break;
    }
  }
  return {t, vmax};
}
}  // namespace

TEST(JerkLimiter, RandomTargetsNeverExceedLimits)
{
  for (double dt : {0.02, 0.01}) {
    JerkLimits lim;   // v [−0.5, 2.0], a 1.0, j 2.0
    JerkLimitedFilter f(lim);
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> target(-1.0, 3.0);   // 한계 밖 목표도 포함
    std::uniform_int_distribution<int> hold(1, 150);
    double ref = 0.0;
    int left = 0;
    double max_a = 0.0;
    double max_j = 0.0;
    for (int k = 0; k < 200000; ++k) {
      if (left-- <= 0) {
        ref = target(rng);
        left = hold(rng);
      }
      const double v_prev = f.velocity();
      f.step(ref, dt);
      max_a = std::max(max_a, std::abs(f.acceleration()));
      max_j = std::max(max_j, std::abs(f.lastJerk()));
      ASSERT_LE(f.velocity(), lim.v_max + kTol);
      ASSERT_GE(f.velocity(), lim.v_min - kTol);
      // 속도 변화 = 사다리꼴 적분 → |Δv| ≤ a_max·dt
      ASSERT_LE(std::abs(f.velocity() - v_prev), lim.a_max * dt + kTol);
    }
    EXPECT_LE(max_a, lim.a_max + kTol);
    EXPECT_LE(max_j, lim.j_max + kTol);
    std::printf("[ info ] dt %.3f: max |a| %.4f, max |j| %.4f\n", dt, max_a, max_j);
  }
}

TEST(JerkLimiter, SCurveRiseTimeNoOvershoot)
{
  JerkLimitedFilter f;
  // 0 → 2 m/s: 이론 S-curve = v/a + a/j = 2 + 0.5 = 2.5 s
  for (double dt : {0.02, 0.01}) {
    const auto [t, vmax] = riseTime(f, 2.0, dt);
    EXPECT_NEAR(t, 2.5, 0.02 + dt);
    EXPECT_LE(vmax, 2.0 + 1e-12);
  }
  // 짧은 스텝 0 → 0.3 (a_max 에 닿지 않음): 이론 2√(v/j) = 0.775 s
  const auto [t2, vmax2] = riseTime(f, 0.3, 0.01);
  EXPECT_NEAR(t2, 2.0 * std::sqrt(0.3 / 2.0), 0.03);
  EXPECT_LE(vmax2, 0.3 + 1e-12);
}

TEST(JerkLimiter, TrapezoidModeAndReset)
{
  JerkLimits lim;
  lim.j_max = 0.0;   // 저크 제한 없음 → 사다리꼴
  JerkLimitedFilter f(lim);
  const auto [t, vmax] = riseTime(f, 1.0, 0.02);
  EXPECT_NEAR(t, 1.0, 0.02 + 1e-9);
  EXPECT_LE(vmax, 1.0 + 1e-12);
  f.reset(0.7, 0.2);
  EXPECT_DOUBLE_EQ(f.velocity(), 0.7);
  EXPECT_DOUBLE_EQ(f.acceleration(), 0.2);
  EXPECT_DOUBLE_EQ(f.step(2.0, 0.0), 0.7);   // dt ≤ 0 → 변화 없음
  // 한계 축소 직후(|a| > a_max)에도 저크 한계를 지키며 줄인다
  JerkLimitedFilter g;
  g.reset(0.5, 1.0);
  JerkLimits small;
  small.a_max = 0.5;
  g.setLimits(small);
  g.step(2.0, 0.02);
  EXPECT_LE(std::abs(g.lastJerk()), small.j_max + kTol);
  EXPECT_DOUBLE_EQ(g.limits().a_max, 0.5);
}

TEST(Pid, ProportionalSetpointWeightAndFeedforward)
{
  PidConfig c;
  c.kp = 0.5;
  c.ki = 0.0;
  c.setpoint_weight = 0.5;
  Pid pid(c);
  // u = ff + kp (b r − y)
  EXPECT_NEAR(pid.update(1.0, 0.2, 0.02, 1.0), 1.0 + 0.5 * (0.5 - 0.2), 1e-12);
  EXPECT_FALSE(pid.saturated());
  // 이번 호출만의 출력 한계
  EXPECT_NEAR(pid.update(1.0, 0.2, 0.02, 1.0, 0.0, 1.1), 1.1, 1e-12);
  EXPECT_TRUE(pid.saturated());
  // 한계가 뒤집혀 들어오면 하한으로
  EXPECT_NEAR(pid.update(1.0, 0.2, 0.02, 1.0, 0.5, 0.3), 0.5, 1e-12);
  // dt ≤ 0 → 적분·미분 갱신 없이 ff + I (역계산이 포화 구간에서 I 를 되감았다)
  EXPECT_NEAR(pid.update(1.0, 0.2, 0.0, 0.25), 0.25 + pid.integral(), 1e-12);
  EXPECT_LT(pid.integral(), 0.0);
}

TEST(Pid, DerivativeOnMeasurementNoKick)
{
  PidConfig c;
  c.kp = 0.0;
  c.ki = 0.0;
  c.kd = 0.1;
  c.derivative_tau = 0.0;
  Pid pid(c);
  pid.update(0.0, 0.0, 0.02);
  // 기준 스텝: 측정이 그대로면 미분항 0
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.02), 0.0, 1e-12);
  // 측정 증가 → 음의 미분항 −kd·ẏ
  EXPECT_NEAR(pid.update(1.0, 0.02, 0.02), -0.1, 1e-12);
}

TEST(Pid, BackCalculationAntiWindup)
{
  // 포화 구간 2 s 뒤 기준을 낮추면: 역계산은 적분이 작게 유지돼 빨리 회복, 와인드업은 오래 과출력
  auto run = [](double tt, double & integral_at_release, double & recover_time) {
      PidConfig c;
      c.kp = 0.4;
      c.ki = 4.0;
      c.tracking_time = tt;
      c.output_min = -0.5;
      c.output_max = 0.5;
      Pid pid(c);
      FirstOrderPlant plant(1.0, 0.08, 0.04, 0.02);
      double y = 0.0;
      for (int k = 0; k < 100; ++k) {   // r = 1.0 (도달 불가, 포화)
        y = plant.step(pid.update(1.0, y, 0.02));
      }
      integral_at_release = pid.integral();
      recover_time = 0.0;
      for (int k = 0; k < 500; ++k) {   // r = 0.3
        y = plant.step(pid.update(0.3, y, 0.02));
        if (std::abs(y - 0.3) > 0.01) {
          recover_time = (k + 1) * 0.02;
        }
      }
    };
  double i_bc = 0.0;
  double t_bc = 0.0;
  double i_wu = 0.0;
  double t_wu = 0.0;
  run(0.1, i_bc, t_bc);
  run(1e9, i_wu, t_wu);   // 사실상 안티와인드업 없음
  std::printf(
    "[ info ] anti-windup: I at release %.3f vs %.3f, recovery %.2f s vs %.2f s\n", i_bc, i_wu,
    t_bc, t_wu);
  EXPECT_LT(std::abs(i_bc), 0.2 * std::abs(i_wu));
  EXPECT_LT(t_bc, t_wu);
  EXPECT_LT(t_bc, 1.0);
}

TEST(Pid, ConditionalIntegrationAndLimit)
{
  PidConfig c;
  c.kp = 0.0;
  c.ki = 1.0;
  c.tracking_time = 0.0;   // 조건부 적분
  c.output_max = 0.1;
  Pid pid(c);
  for (int k = 0; k < 100; ++k) {
    pid.update(1.0, 0.0, 0.02);
  }
  // 포화 방향으로는 적분이 멈춘다: I 는 출력 한계를 조금 넘는 수준에 머문다
  EXPECT_LE(pid.integral(), 0.1 + 0.02 + 1e-12);
  // 오차 부호가 바뀌면 다시 적분(풀림)
  const double before = pid.integral();
  pid.update(-1.0, 0.0, 0.02);
  EXPECT_LT(pid.integral(), before);
  // 적분 상한
  PidConfig lim;
  lim.kp = 0.0;
  lim.ki = 10.0;
  lim.integral_limit = 0.05;
  Pid p2(lim);
  for (int k = 0; k < 50; ++k) {
    p2.update(1.0, 0.0, 0.02);
  }
  EXPECT_NEAR(p2.integral(), 0.05, 1e-12);
  p2.reset(0.01);
  EXPECT_DOUBLE_EQ(p2.integral(), 0.01);
  EXPECT_DOUBLE_EQ(p2.lastUnsaturated(), 0.0);
}

TEST(Pid, FirstOrderPlantStepResponse)
{
  FirstOrderPlant plant(1.0, 0.1, 0.04, 0.01);
  // 순수 지연 4 스텝 동안 0
  for (int k = 0; k < 4; ++k) {
    EXPECT_DOUBLE_EQ(plant.step(1.0), 0.0);
  }
  // 이후 τ = 0.1 s 뒤 63.2 %
  double y = 0.0;
  for (int k = 0; k < 10; ++k) {
    y = plant.step(1.0);
  }
  EXPECT_NEAR(y, 1.0 - std::exp(-1.0), 1e-9);
  plant.reset(0.5);
  EXPECT_DOUBLE_EQ(plant.output(), 0.5);
  FirstOrderPlant instant(2.0, 0.0, 0.0, 0.01);
  EXPECT_DOUBLE_EQ(instant.step(0.3), 0.6);
}

TEST(Pid, TunedGainsStepAndRampAcceptance)
{
  // docs/algorithms/path_tracking.md 튜닝 결과 (tools/pid_tuning 격자 탐색, config 기본값):
  // 모델 추종 2-자유도 PI, kp 0.4, ki 2.0, T_t = √(kp/ki). 기준
  //   모델 = 명목 플랜트 (K 1, T_p 0.08, T_d 0.04).
  PidConfig c;
  c.kp = 0.4;
  c.ki = 2.0;
  c.tracking_time = std::sqrt(0.4 / 2.0);
  const double dt = 0.02;
  auto run = [&](double K, double tp, double td, double r, int steps, double & ymax) {
      Pid pid(c);
      FirstOrderPlant plant(K, tp, td, dt);
      FirstOrderPlant model(1.0, 0.08, 0.04, dt);
      double y = 0.0;
      ymax = 0.0;
      for (int k = 0; k < steps; ++k) {
        const double ym = model.output();
        model.step(r);
        y = plant.step(pid.update(ym, y, dt, r, -0.5, 2.5));
        ymax = std::max(ymax, y);
      }
      return y;
    };
  double ymax = 0.0;
  // (1) 명목 플랜트 0.15 m/s 스텝: 오버슈트 ≤ 1 %, 정상오차 < 0.01
  double y = run(1.0, 0.08, 0.04, 0.15, 100, ymax);
  EXPECT_LE(ymax, 0.15 * 1.01);
  EXPECT_LT(std::abs(y - 0.15), 0.01);
  // (2) 불일치 플랜트(느린 서보, 긴 지연, 이득 0.8): 오버슈트 ≤ 5 %, 3 s 뒤 정상오차 < 0.01
  for (double K : {0.8, 1.0}) {
    for (double tp : {0.05, 0.12}) {
      for (double td : {0.02, 0.06}) {
        y = run(K, tp, td, 0.15, 150, ymax);
        EXPECT_LE(ymax, 0.15 * 1.05) << K << " " << tp << " " << td;
        EXPECT_LT(std::abs(y - 0.15), 0.01) << K << " " << tp << " " << td;
      }
    }
  }
  // (3) 저크 제한 램프 0 → 1 m/s, 명목 플랜트: 측정 vs 기준 모델 오차 ≈ 0 (PI 개입 없음)
  {
    Pid pid(c);
    FirstOrderPlant plant(1.0, 0.08, 0.04, dt);
    FirstOrderPlant model(1.0, 0.08, 0.04, dt);
    JerkLimitedFilter f;
    double max_err = 0.0;
    double yy = 0.0;
    for (int k = 0; k < 200; ++k) {
      const double r = f.step(1.0, dt);
      const double ym = model.output();
      model.step(r);
      yy = plant.step(pid.update(ym, yy, dt, r, -0.5, 2.5));
      max_err = std::max(max_err, std::abs(model.output() - yy));
    }
    EXPECT_LT(max_err, 1e-9);
    EXPECT_LT(std::abs(yy - 1.0), 0.01);
  }
}

TEST(VelocityProfiler, CurvaturePreservedDuringAcceleration)
{
  VelocityProfilerConfig c;
  c.use_pid = false;
  VelocityProfiler vp(c);
  double max_dev = 0.0;
  double v_prev = 0.0;
  double a_prev = 0.0;
  double max_j = 0.0;
  for (int k = 0; k < 200; ++k) {
    const ProfilerOutput o = vp.update(1.0, 0.5, false, 0.0, 0.0, 0.02);
    if (o.v_ref > 0.05) {
      max_dev = std::max(max_dev, std::abs(o.w_ref / o.v_ref - 0.5));
    }
    const double a = (o.v - v_prev) / 0.02;
    if (k > 0) {
      max_j = std::max(max_j, std::abs(a - a_prev) / 0.02);
    }
    v_prev = o.v;
    a_prev = a;
  }
  EXPECT_LT(max_dev, 1e-9);
  // 출력 속도의 수치 미분으로 본 저크도 한계(2.0) 근처 이하 (사다리꼴 적분 이산화 여유 포함)
  EXPECT_LE(max_j, 2.0 * 1.05);
  EXPECT_NEAR(vp.linearFilter().velocity(), 1.0, 1e-12);
}

TEST(VelocityProfiler, PayloadScaleSlowsAcceleration)
{
  VelocityProfilerConfig c;
  c.use_pid = false;
  VelocityProfiler light(c);
  VelocityProfiler heavy(c);
  heavy.setPayloadScale(45.0 / (45.0 + 25.0));
  EXPECT_NEAR(heavy.payloadScale(), 45.0 / 70.0, 1e-12);
  int k_light = 0;
  int k_heavy = 0;
  for (int k = 0; k < 500; ++k) {
    if (light.update(1.0, 0.0, false, 0, 0, 0.02).v < 1.0) {
      k_light = k;
    }
    if (heavy.update(1.0, 0.0, false, 0, 0, 0.02).v < 1.0) {
      k_heavy = k;
    }
  }
  EXPECT_GT(k_heavy, k_light);
  EXPECT_LE(heavy.linearFilter().limits().a_max, 1.0 * 45.0 / 70.0 + 1e-12);
  heavy.setPayloadScale(5.0);   // 상한 1
  EXPECT_DOUBLE_EQ(heavy.payloadScale(), 1.0);
}

TEST(VelocityProfiler, ResyncStopDeadbandAndPid)
{
  VelocityProfilerConfig c;
  VelocityProfiler vp(c);
  for (int k = 0; k < 150; ++k) {
    vp.update(1.0, 0.0, true, vp.linearFilter().velocity(), 0.0, 0.02);
  }
  // 측정이 0 으로 급감(안전 노드 정지) → 재동기화
  ProfilerOutput o = vp.update(1.0, 0.0, true, 0.0, 0.0, 0.02);
  EXPECT_TRUE(o.resynced);
  EXPECT_LT(o.v_ref, 0.05);
  // 정지 데드밴드: 목표 0, 기준 0 이면 측정 잡음과 무관하게 정확히 0
  vp.reset();
  o = vp.update(0.0, 0.0, true, 0.003, -0.002, 0.02);
  EXPECT_DOUBLE_EQ(o.v, 0.0);
  EXPECT_DOUBLE_EQ(o.w, 0.0);
  // PID 보정: 측정이 기준보다 느리면 출력 > 기준, 단 보정 폭 이내
  vp.reset(0.5, 0.0);
  o = vp.update(0.5, 0.0, true, 0.3, 0.0, 0.02);
  EXPECT_GT(o.v, o.v_ref);
  EXPECT_LE(o.v, o.v_ref + c.max_linear_correction + 1e-12);
  // dt ≤ 0 → 상태 그대로 (직전 출력)
  const double v_prev = o.v;
  o = vp.update(1.0, 0.0, true, 0.3, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(o.v, v_prev);
  EXPECT_DOUBLE_EQ(o.v_ref, vp.linearFilter().velocity());
}

// 정지 착지: 슬립 플랜트(이득 0.8)에서 PI 보정이 남은 채 목표 0
// → 출력이 계단 없이(저크 ≤ j) 정확히 0.
TEST(VelocityProfiler, StopLandsResidualCorrectionWithinJerkLimit)
{
  VelocityProfilerConfig c;
  VelocityProfiler vp(c);
  FirstOrderPlant plant(0.8, 0.12, 0.06, 0.02);
  double y = 0.0;
  double v1 = 0.0;
  double v0 = 0.0;
  for (int k = 0; k < 200; ++k) {
    const ProfilerOutput o = vp.update(0.5, 0.0, true, y, 0.0, 0.02);
    y = plant.step(o.v);
    v0 = v1;
    v1 = o.v;
  }
  EXPECT_GT(v1 - 0.5, 0.05);   // 슬립 보정이 실제로 남아 있다
  double max_j = 0.0;
  bool zero = false;
  for (int k = 0; k < 200; ++k) {
    const ProfilerOutput o = vp.update(0.0, 0.0, true, y, 0.0, 0.02);
    y = plant.step(o.v);
    const double j = std::abs(o.v - 2.0 * v1 + v0) / (0.02 * 0.02);
    // 정지 구간(기준 0) 의 출력 저크 — 기준 램프 중에는 PI 보정 저크가 더해질 수 있어
    // 착지 구간만 본다
    if (o.v_ref == 0.0) {
      max_j = std::max(max_j, j);
    }
    v0 = v1;
    v1 = o.v;
    zero = zero || (o.v == 0.0 && o.v_ref == 0.0 && k > 0 && v0 == 0.0);
  }
  std::printf("[ info ] stop landing: max |jerk| %.3f m/s^3\n", max_j);
  EXPECT_TRUE(zero);
  EXPECT_LE(max_j, c.linear.j_max * 1.5);   // 착지 첫 스텝의 이산 오차(사다리꼴 적분) 여유
  EXPECT_DOUBLE_EQ(v1, 0.0);
}

TEST(VelocityProfiler, InPlaceRotationIsJerkLimited)
{
  VelocityProfilerConfig c;
  c.use_pid = false;
  VelocityProfiler vp(c);
  double w_prev = 0.0;
  double max_alpha = 0.0;
  for (int k = 0; k < 100; ++k) {
    const ProfilerOutput o = vp.update(0.0, 1.0, false, 0.0, 0.0, 0.02);
    max_alpha = std::max(max_alpha, std::abs(o.w - w_prev) / 0.02);
    w_prev = o.w;
    EXPECT_DOUBLE_EQ(o.v, 0.0);
  }
  EXPECT_NEAR(w_prev, 1.0, 1e-9);
  EXPECT_LE(max_alpha, c.angular.a_max + 1e-9);
  // 곡률 모드 → 회전 모드 전환 시 연속
  vp.reset();
  for (int k = 0; k < 50; ++k) {
    vp.update(0.5, 0.5, false, 0, 0, 0.02);
  }
  const double w_before = vp.update(0.5, 0.5, false, 0, 0, 0.02).w_ref;
  const double w_after = vp.update(0.0, 0.5, false, 0, 0, 0.02).w_ref;
  EXPECT_LE(std::abs(w_after - w_before), c.angular.a_max * 0.02 + 1e-9);
}

namespace
{
// 명령 흐름 시험대: 50 Hz 프로파일러 → 플랜트 (1차 + 지연), 측정 = 플랜트 + 잡음
// (stale_every 번째 주기는 직전 표본). 최종 명령의 수치 저크 (2차 차분) 통계와 재동기화 수를 본다
// (리뷰 하네스 jerk_probe 와 같은 목표 열).
struct StreamStats
{
  double jerk_max{0.0};
  double jerk_p99{0.0};
  double accel_max{0.0};
  int resyncs{0};
  double final_v{0.0};
  double overshoot{0.0};
};

StreamStats runStream(
  double gain, double tau, double delay, int stale_every, double noise, bool mark_stale,
  unsigned seed = 1)
{
  VelocityProfilerConfig cfg;
  cfg.pid_linear.tracking_time = 0.447;
  cfg.pid_angular.tracking_time = 0.447;
  VelocityProfiler prof(cfg);
  const double dt = 0.02;
  FirstOrderPlant plant(gain, tau, delay, dt, 0.0);
  std::mt19937 rng(seed);
  std::normal_distribution<double> nd(0.0, 1.0);
  double y = 0.0;
  double meas = 0.0;
  double v1 = 0.0;
  double v0 = 0.0;
  std::vector<double> js;
  StreamStats st;
  for (int k = 0; k < 1500; ++k) {
    const double t = k * dt;
    const double target = t < 1.0 ? 0.0 : (t < 12.0 ? 1.0 : (t < 20.0 ? 0.4 : 0.0));
    const bool stale = stale_every > 0 && k % stale_every == 0;
    if (!stale) {
      meas = y + noise * nd(rng);
    }
    const ProfilerOutput o = prof.update(target, 0.0, true, meas, 0.0, dt, !(stale && mark_stale));
    y = plant.step(o.v);
    if (k >= 2 && !o.resynced) {
      const double j = std::abs((o.v - v1) - (v1 - v0)) / (dt * dt);
      js.push_back(j);
      st.jerk_max = std::max(st.jerk_max, j);
    }
    st.accel_max = std::max(st.accel_max, std::abs(o.v - v1) / dt);
    if (t > 1.0 && t < 12.0) {
      st.overshoot = std::max(st.overshoot, y - 1.0);
    }
    v0 = v1;
    v1 = o.v;
  }
  std::sort(js.begin(), js.end());
  st.jerk_p99 = js[static_cast<std::size_t>(0.99 * static_cast<double>(js.size() - 1))];
  st.resyncs = prof.resyncCount();
  st.final_v = y;
  return st;
}
}  // namespace

// 보내는 명령(기준 + PI 보정)의 저크·가속도가 한계 안
// (리뷰: 잡음 5 mm/s 에서 p99 29.6, 최대 38 m/s³).
TEST(VelocityProfiler, OutputStreamJerkWithinLimit)
{
  struct Case {const char * name; double gain, tau, delay; int stale; double noise; bool mark;};
  const Case cases[] = {
    {"nominal", 1.0, 0.08, 0.04, 0, 0.0, true},
    {"slow servo T 0.15", 1.0, 0.15, 0.04, 0, 0.0, true},
    {"slip gain 0.9", 0.9, 0.08, 0.04, 0, 0.0, true},
    {"slip gain 0.8, T 0.12, delay 0.06", 0.8, 0.12, 0.06, 0, 0.0, true},
    {"odom repeated every 7th (marked stale)", 1.0, 0.08, 0.04, 7, 0.0, true},
    {"odom repeated every 7th (unmarked)", 1.0, 0.08, 0.04, 7, 0.0, false},
    {"odom noise 5 mm/s", 1.0, 0.08, 0.04, 0, 0.005, true},
    {"odom noise 14 mm/s + repeats", 1.0, 0.08, 0.04, 3, 0.014, false},
  };
  for (const auto & c : cases) {
    const StreamStats st = runStream(c.gain, c.tau, c.delay, c.stale, c.noise, c.mark);
    std::printf(
      "[ info ] %-38s jerk p99 %.3f max %.3f, |a| max %.3f, resync %d, overshoot %.4f, end %.3f\n",
      c.name, st.jerk_p99, st.jerk_max, st.accel_max, st.resyncs, st.overshoot, st.final_v);
    EXPECT_LE(st.jerk_max, 2.0 + 1e-6) << c.name;
    EXPECT_LE(st.accel_max, 1.0 + 1e-6) << c.name;
    EXPECT_EQ(st.resyncs, 0) << c.name;
    EXPECT_LT(st.overshoot, 0.05) << c.name;
    EXPECT_NEAR(st.final_v, 0.0, 1e-3) << c.name;   // 끝에서 정지
  }
}

// PI 는 성형 뒤에도 불일치 플랜트를 보정한다 (슬립 이득 0.9: 정상오차 < 1 cm/s)
TEST(VelocityProfiler, PiStillCorrectsSlipThroughShaper)
{
  VelocityProfilerConfig cfg;
  cfg.pid_linear.tracking_time = 0.447;
  VelocityProfiler prof(cfg);
  FirstOrderPlant plant(0.9, 0.08, 0.04, 0.02, 0.0);
  double y = 0.0;
  for (int k = 0; k < 400; ++k) {
    y = plant.step(prof.update(0.8, 0.0, true, y, 0.0, 0.02).v);
  }
  EXPECT_NEAR(y, 0.8, 0.01);
  EXPECT_EQ(prof.resyncCount(), 0);
}

// safety_node 가 명령을 붙잡음 (플랜트 출력 0): PI 가 +c_max 로 와인드업해 cmd_vel_smoothed 가
// cmd_vel_nav 보다 0.2 m/s 위에 머물던 문제 (리뷰 실측 평균 +0.199). 측정 기반 재동기화로
// 괴리가 작고,
// 풀린 뒤에는 측정 속도에서 저크 한계로 다시 올라간다.
TEST(VelocityProfiler, HeldOutputDoesNotWindUpAndRestartsSmoothly)
{
  for (const double v_nav : {0.05, 0.3}) {
    VelocityProfilerConfig cfg;
    cfg.pid_linear.tracking_time = 0.447;
    VelocityProfiler prof(cfg);
    FirstOrderPlant plant(1.0, 0.08, 0.04, 0.02, 0.0);
    double y = 0.0;
    double sum_gap = 0.0;
    double max_gap = 0.0;
    int n = 0;
    // 3 s 붙잡힘: 로봇은 0 (명령이 하류에서 0 으로 바뀜)
    for (int k = 0; k < 150; ++k) {
      const ProfilerOutput o = prof.update(v_nav, 0.0, true, 0.0, 0.0, 0.02);
      if (k > 25) {
        sum_gap += o.v - v_nav;
        max_gap = std::max(max_gap, o.v - v_nav);
        ++n;
      }
    }
    std::printf(
      "[ info ] held at 0 with cmd_vel_nav %.2f: mean(out − nav) %+.3f, max %+.3f, resyncs %d\n",
      v_nav,
      sum_gap / n, max_gap, prof.resyncCount());
    EXPECT_GT(prof.resyncCount(), 0);
    EXPECT_LT(sum_gap / n, 0.05);
    EXPECT_LT(max_gap, 0.15);
    // 풀림: 플랜트가 명령을 따른다 → 저크 한계, 목표 위로 튀지 않음
    double v1 = prof.update(v_nav, 0.0, true, 0.0, 0.0, 0.02).v;
    double v0 = v1;
    double jmax = 0.0;
    double ymax = 0.0;
    const int r0 = prof.resyncCount();
    for (int k = 0; k < 250; ++k) {
      const ProfilerOutput o = prof.update(v_nav, 0.0, true, y, 0.0, 0.02);
      y = plant.step(o.v);
      if (k >= 1) {
        jmax = std::max(jmax, std::abs((o.v - v1) - (v1 - v0)) / 4e-4);
      }
      ymax = std::max(ymax, y);
      v0 = v1;
      v1 = o.v;
    }
    EXPECT_EQ(prof.resyncCount(), r0);
    EXPECT_LE(jmax, 2.0 + 1e-6);
    EXPECT_LT(ymax, v_nav + 0.02);
    EXPECT_NEAR(y, v_nav, 0.01);
  }
}

// 들쭉날쭉한 주기 (부하로 50 Hz 타이머가 10–40 ms 간격): 실제 경과로 적분해도 보내는 명령의
// 수치 저크 (u_k − 2u_{k−1} + u_{k−2} 를 실제 간격으로 나눈 값)가 한계 안
// (Gazebo 부하 150+ 에서 노드 진단 저크 7.6 실측)
TEST(VelocityProfiler, IrregularPeriodsKeepOutputJerk)
{
  VelocityProfilerConfig cfg;
  cfg.pid_linear.tracking_time = 0.447;
  VelocityProfiler prof(cfg);
  FirstOrderPlant plant(1.0, 0.08, 0.04, 0.02, 0.0);
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> per(0.008, 0.045);
  std::normal_distribution<double> nd(0.0, 0.005);
  double t = 0.0;
  double y = 0.0;
  double v1 = 0.0;
  double a1 = 0.0;
  double jmax = 0.0;
  double amax = 0.0;
  double dt_prev = 0.02;
  for (int k = 0; k < 2000; ++k) {
    const double dt = per(rng);
    t += dt;
    const double target = t < 1.0 ? 0.0 : (t < 8.0 ? 1.0 : (t < 14.0 ? 0.3 : 0.0));
    const ProfilerOutput o = prof.update(target, 0.0, true, y + nd(rng), 0.0, dt);
    y = plant.step(o.v, dt);
    const double a = (o.v - v1) / dt;
    if (k >= 2 && !o.resynced) {
      jmax = std::max(jmax, std::abs(a - a1) / dt);
    }
    amax = std::max(amax, std::abs(a));
    v1 = o.v;
    a1 = a;
    dt_prev = dt;
  }
  (void)dt_prev;
  std::printf(
    "[ info ] irregular 8–45 ms periods: output jerk max %.3f, |a| max %.3f, end %.4f\n", jmax,
    amax, v1);
  EXPECT_LE(jmax, 2.0 + 1e-6);
  EXPECT_LE(amax, 1.0 + 1e-6);
  EXPECT_EQ(prof.resyncCount(), 0);
}
