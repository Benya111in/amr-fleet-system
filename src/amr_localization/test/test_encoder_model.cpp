// 인코더 모델 단위 테스트: 양자화 단위, 슬립 잡음 통계, 감김 처리, 양자화 오차의 MA(1) 구조.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

#include "amr_localization/encoder_model.hpp"

using amr_localization::EncoderParams;
using amr_localization::EncoderReading;
using amr_localization::WheelEncoderModel;

namespace
{
EncoderParams quantOnly()
{
  EncoderParams p;
  p.slip_noise = false;
  return p;
}
}  // namespace

TEST(EncoderModel, RejectsInvalidParams)
{
  EncoderParams p;
  p.ticks_per_revolution = 0;
  EXPECT_THROW(WheelEncoderModel(p, 1), std::invalid_argument);
  p.ticks_per_revolution = 4096;
  p.slip_noise_stddev = -0.1;
  EXPECT_THROW(WheelEncoderModel(p, 1), std::invalid_argument);
}

TEST(EncoderModel, TickResolution)
{
  WheelEncoderModel enc(quantOnly(), 1);
  const double tick = 2.0 * M_PI / 4096.0;
  EXPECT_DOUBLE_EQ(enc.tickAngle(), tick);
  // 바퀴 둘레 기준 1 틱 = 2πr/4096 = 0.1266 mm (r = 0.0825)
  EXPECT_NEAR(0.0825 * enc.tickAngle(), 2.0 * M_PI * 0.0825 / 4096.0, 1e-15);
  EXPECT_NEAR(0.0825 * enc.tickAngle(), 1.2655e-4, 1e-8);
}

TEST(EncoderModel, FirstSampleOnlySetsReference)
{
  WheelEncoderModel enc(quantOnly(), 1);
  const EncoderReading r0 = enc.update(10.0);
  EXPECT_FALSE(r0.valid);
  EXPECT_EQ(r0.delta_ticks, 0);
  EXPECT_EQ(r0.ticks, std::llround(10.0 / enc.tickAngle()));
  const EncoderReading r1 = enc.update(10.0);
  EXPECT_TRUE(r1.valid);
  EXPECT_EQ(r1.delta_ticks, 0);
  EXPECT_DOUBLE_EQ(r1.delta_angle, 0.0);
}

TEST(EncoderModel, QuantizationStepIsOneTick)
{
  WheelEncoderModel enc(quantOnly(), 1);
  const double tick = enc.tickAngle();
  enc.update(0.0);
  // 반 틱 미만 이동은 0, 1 틱 이동은 정확히 1 틱 증분
  EncoderReading r = enc.update(0.4 * tick);
  EXPECT_EQ(r.delta_ticks, 0);
  EXPECT_DOUBLE_EQ(r.delta_angle, 0.0);
  r = enc.update(1.0 * tick);
  EXPECT_EQ(r.delta_ticks, 1);
  EXPECT_DOUBLE_EQ(r.delta_angle, tick);
  r = enc.update(-3.2 * tick);
  EXPECT_EQ(r.delta_ticks, -4);
  EXPECT_NEAR(r.delta_angle, -4.0 * tick, 1e-15);
}

TEST(EncoderModel, QuantizationErrorIsBoundedOnAccumulatedAngle)
{
  // 누적 카운트의 양자화 오차는 |q| ≤ δ/2 로 유계 (랜덤워크가 아니다)
  WheelEncoderModel enc(quantOnly(), 1);
  const double tick = enc.tickAngle();
  std::mt19937_64 rng(3);
  std::uniform_real_distribution<double> step(0.0, 0.5);
  double angle = 0.0;
  enc.update(angle);
  double measured = 0.0;
  for (int i = 0; i < 20000; ++i) {
    angle += step(rng);
    measured += enc.update(angle).delta_angle;
    ASSERT_LE(std::abs(measured - angle), 0.5 * tick + 1e-9);
  }
}

TEST(EncoderModel, StepErrorIsMA1WithVarianceDeltaSquaredOverSix)
{
  // e_k = q_k − q_{k−1}: Var = δ²/6, lag-1 공분산 = −δ²/12 (docs/algorithms/kinematics.md §3.1)
  WheelEncoderModel enc(quantOnly(), 1);
  const double tick = enc.tickAngle();
  std::mt19937_64 rng(5);
  std::uniform_real_distribution<double> step(0.1, 0.5);
  double angle = 0.0;
  enc.update(angle);
  std::vector<double> err;
  for (int i = 0; i < 100000; ++i) {
    const double d = step(rng);
    angle += d;
    err.push_back(enc.update(angle).delta_angle - d);
  }
  double var = 0.0;
  double cov1 = 0.0;
  for (std::size_t i = 0; i < err.size(); ++i) {
    var += err[i] * err[i];
    if (i > 0) {
      cov1 += err[i] * err[i - 1];
    }
  }
  var /= static_cast<double>(err.size());
  cov1 /= static_cast<double>(err.size() - 1);
  EXPECT_NEAR(var / (tick * tick / 6.0), 1.0, 0.03);
  EXPECT_NEAR(cov1 / (-tick * tick / 12.0), 1.0, 0.05);
}

TEST(EncoderModel, SlipNoiseStatisticsWithinTenPercent)
{
  // ε ~ N(0, σ_s²), 10k 샘플에서 표본 σ 가 설정값의 ±10 % 이내, 평균 ≈ 0 (seed 고정)
  EncoderParams p;
  p.slip_noise_stddev = 0.01;
  WheelEncoderModel enc(p, 42);
  enc.update(0.0);
  double angle = 0.0;
  double sum = 0.0;
  double sum2 = 0.0;
  const int n = 10000;
  for (int i = 0; i < n; ++i) {
    angle += 0.5;
    const EncoderReading r = enc.update(angle);
    const double eps = r.slip_factor - 1.0;
    sum += eps;
    sum2 += eps * eps;
    // 측정 증분 = 양자화 증분 × (1 + ε)
    EXPECT_NEAR(
      r.delta_angle, static_cast<double>(r.delta_ticks) * enc.tickAngle() *
      r.slip_factor, 1e-12);
  }
  const double mean = sum / n;
  const double stddev = std::sqrt(sum2 / n - mean * mean);
  EXPECT_NEAR(stddev, 0.01, 0.001);
  EXPECT_NEAR(mean, 0.0, 4.0 * 0.01 / std::sqrt(n));
}

TEST(EncoderModel, SameSeedIsReproducible)
{
  EncoderParams p;
  WheelEncoderModel a(p, 7);
  WheelEncoderModel b(p, 7);
  a.update(0.0);
  b.update(0.0);
  for (int i = 1; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(a.update(0.3 * i).delta_angle, b.update(0.3 * i).delta_angle);
  }
}

TEST(EncoderModel, NoiseFreeModeReturnsTrueIncrement)
{
  EncoderParams p;
  p.quantize = false;
  p.slip_noise = false;
  WheelEncoderModel enc(p, 1);
  enc.update(0.0);
  const EncoderReading r = enc.update(0.123456);
  EXPECT_DOUBLE_EQ(r.delta_angle, 0.123456);
  EXPECT_DOUBLE_EQ(r.slip_factor, 1.0);
}

TEST(EncoderModel, UnwrapsWrappedJointAngles)
{
  // 조인트 각이 (−π, π] 로 감겨 들어와도 누적 틱은 연속이다
  WheelEncoderModel enc(quantOnly(), 1);
  const double step = 0.3;
  double total = 0.0;
  enc.update(std::remainder(0.0, 2.0 * M_PI));
  double measured = 0.0;
  for (int i = 0; i < 100; ++i) {
    total += step;
    measured += enc.update(std::remainder(total, 2.0 * M_PI)).delta_angle;
  }
  EXPECT_NEAR(measured, total, enc.tickAngle());
  EXPECT_EQ(enc.ticks(), std::llround(total / enc.tickAngle()));
}

TEST(EncoderModel, ResetStartsNewReference)
{
  WheelEncoderModel enc(quantOnly(), 1);
  enc.update(0.0);
  enc.update(1.0);
  enc.reset();
  const EncoderReading r = enc.update(50.0);
  EXPECT_FALSE(r.valid);
  EXPECT_TRUE(enc.update(50.0).valid);
  EXPECT_EQ(enc.params().ticks_per_revolution, 4096);
}
