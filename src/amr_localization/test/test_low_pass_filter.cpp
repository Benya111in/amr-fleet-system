// 1차 LPF 단위 테스트: 정확한 −3 dB 설계, 2×차단 주파수 감쇠(해석값 = 시뮬레이션),
// DC 이득, 통과 조건.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "amr_localization/low_pass_filter.hpp"

using amr_localization::FirstOrderLowPass;

namespace
{
/// 정현파(진폭 1)를 통과시켜 정상 상태 진폭비를 측정한다. 후반 10 s 를 동기 검파(lock-in)로
/// 진폭 추정 — 샘플 위상이 몇 개로 한정되는 고주파에서도 최대값 측정보다 정확하다.
double simulatedGain(double cutoff, double fs, double f)
{
  FirstOrderLowPass lpf(cutoff);
  const double dt = 1.0 / fs;
  const double omega = 2.0 * M_PI * f * dt;
  const int n = static_cast<int>(std::lround(fs * 20.0));
  const int start = n / 2;
  double s = 0.0;
  double c = 0.0;
  for (int i = 0; i < n; ++i) {
    const double y = lpf.filter(std::sin(omega * i), dt);
    if (i >= start) {
      s += y * std::sin(omega * i);
      c += y * std::cos(omega * i);
    }
  }
  return 2.0 * std::hypot(s, c) / static_cast<double>(n - start);
}
}  // namespace

TEST(LowPassFilter, ExactMinus3dBAtCutoff)
{
  for (double fs : {50.0, 100.0, 400.0}) {
    for (double fc : {2.0, 10.0, 20.0}) {
      if (fc >= fs / 2.0) {
        continue;
      }
      const double dt = 1.0 / fs;
      const double alpha = FirstOrderLowPass::alphaFor(fc, dt);
      const double gain = FirstOrderLowPass::magnitudeResponse(alpha, 2.0 * M_PI * fc * dt);
      EXPECT_NEAR(gain, 1.0 / std::sqrt(2.0), 1e-12) << "fs " << fs << " fc " << fc;
    }
  }
  // 브리프 수치: fs 100 Hz, fc 20 Hz → α = 0.673, fc 10 Hz → α = 0.456
  EXPECT_NEAR(FirstOrderLowPass::alphaFor(20.0, 0.01), 0.673, 1e-3);
  EXPECT_NEAR(FirstOrderLowPass::alphaFor(10.0, 0.01), 0.456, 1e-3);
}

TEST(LowPassFilter, AttenuationAtTwiceCutoff)
{
  // 기본 설정 fs = 100 Hz (IMU), fc = 20 Hz: 40 Hz 에서 |H| = 0.526 (−5.6 dB)
  const double fs = 100.0;
  const double fc = 20.0;
  const double alpha = FirstOrderLowPass::alphaFor(fc, 1.0 / fs);
  const double analytic = FirstOrderLowPass::magnitudeResponse(alpha, 2.0 * M_PI * 2.0 * fc / fs);
  EXPECT_NEAR(analytic, 0.526, 0.002);
  EXPECT_LT(analytic, 1.0 / std::sqrt(2.0));
  EXPECT_NEAR(simulatedGain(fc, fs, 2.0 * fc), analytic, 0.01);

  // 차단 주파수가 Nyquist 에 비해 낮으면 연속 1차계 1/√(1 + 2²) = 0.447 에 수렴
  const double fs2 = 1000.0;
  const double fc2 = 5.0;
  const double a2 = FirstOrderLowPass::alphaFor(fc2, 1.0 / fs2);
  const double g2 = FirstOrderLowPass::magnitudeResponse(a2, 2.0 * M_PI * 2.0 * fc2 / fs2);
  EXPECT_NEAR(g2, 1.0 / std::sqrt(5.0), 0.005);
  EXPECT_NEAR(simulatedGain(fc2, fs2, 2.0 * fc2), g2, 0.01);
}

TEST(LowPassFilter, SimulatedCutoffWithinTwoPercent)
{
  // 주파수 스윕으로 이득이 1/√2 를 지나는 점을 찾는다 → fc ± 2 %
  const double fs = 100.0;
  const double fc = 20.0;
  double crossing = 0.0;
  double prev_f = 0.0;
  double prev_g = 1.0;
  for (double f = 10.0; f < 30.0; f += 0.1) {
    const double g = simulatedGain(fc, fs, f);
    if (prev_g >= 1.0 / std::sqrt(2.0) && g < 1.0 / std::sqrt(2.0)) {
      crossing = prev_f + (f - prev_f) * (prev_g - 1.0 / std::sqrt(2.0)) / (prev_g - g);
      break;
    }
    prev_f = f;
    prev_g = g;
  }
  EXPECT_NEAR(crossing / fc, 1.0, 0.02);
}

TEST(LowPassFilter, BackwardEulerApproximationMissesCutoff)
{
  // α = Δt/(τ + Δt) 는 fs 100 Hz, fc 20 Hz 에서 실제 −3 dB 가 13.7 Hz 근처
  // → 정확 설계가 필요한 이유
  const double dt = 0.01;
  const double tau = 1.0 / (2.0 * M_PI * 20.0);
  const double alpha_be = dt / (tau + dt);
  const double gain_at_fc = FirstOrderLowPass::magnitudeResponse(alpha_be, 2.0 * M_PI * 20.0 * dt);
  EXPECT_LT(gain_at_fc, 0.65);
}

TEST(LowPassFilter, UnityDcGainAndFirstSamplePassThrough)
{
  FirstOrderLowPass lpf(5.0);
  EXPECT_FALSE(lpf.initialized());
  EXPECT_DOUBLE_EQ(lpf.filter(3.0, 0.01), 3.0);  // 첫 샘플 = 입력
  EXPECT_TRUE(lpf.initialized());
  double y = 0.0;
  for (int i = 0; i < 2000; ++i) {
    y = lpf.filter(1.0, 0.01);
  }
  EXPECT_NEAR(y, 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(lpf.value(), y);
  lpf.reset();
  EXPECT_FALSE(lpf.initialized());
  EXPECT_DOUBLE_EQ(lpf.filter(-2.0, 0.01), -2.0);
}

TEST(LowPassFilter, PassThroughConditions)
{
  EXPECT_DOUBLE_EQ(FirstOrderLowPass::alphaFor(0.0, 0.01), 1.0);    // 끔
  EXPECT_DOUBLE_EQ(FirstOrderLowPass::alphaFor(20.0, 0.0), 1.0);    // dt 없음
  EXPECT_DOUBLE_EQ(FirstOrderLowPass::alphaFor(20.0, -0.01), 1.0);
  EXPECT_DOUBLE_EQ(FirstOrderLowPass::alphaFor(60.0, 0.01), 1.0);   // Nyquist 초과
  FirstOrderLowPass lpf;
  EXPECT_DOUBLE_EQ(lpf.cutoff(), 0.0);
  lpf.filter(0.0, 0.01);
  EXPECT_DOUBLE_EQ(lpf.filter(5.0, 0.01), 5.0);
  lpf.setCutoff(1.0);
  EXPECT_DOUBLE_EQ(lpf.cutoff(), 1.0);
  EXPECT_LT(lpf.filter(10.0, 0.01), 10.0);
}
