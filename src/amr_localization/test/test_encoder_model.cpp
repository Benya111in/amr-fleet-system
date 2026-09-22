// 인코더 모델 단위 테스트: 양자화 단위, 거리당 슬립 잡음 통계, 다회전 카운터·감김 처리,
// 양자화 오차의 MA(1) 구조.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
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
  p.slip_noise_stddev = 0.01;
  p.slip_reference_angle = 0.0;
  EXPECT_THROW(WheelEncoderModel(p, 1), std::invalid_argument);
  p.slip_reference_angle = 0.1;
  p.max_wheel_speed = 0.0;
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
  // 거리당 슬립: η = 측정 − 양자화 증분 ~ N(0, σ_s² φ_ref |Δφ_q|). 정규화 η/√(…) 의 표본 σ 가
  // 1 의 ±10 % 이내, 평균 ≈ 0 (10k 샘플, seed 고정). 기준 회전각에서는 ε = η/Δφ 의 σ 가 σ_s.
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
    const double quantized = static_cast<double>(r.delta_ticks) * enc.tickAngle();
    const double z = (r.delta_angle - quantized) /
      (p.slip_noise_stddev * std::sqrt(p.slip_reference_angle * std::abs(quantized)));
    sum += z;
    sum2 += z * z;
    EXPECT_NEAR(r.slip_factor, r.delta_angle / quantized, 1e-12);
  }
  const double mean = sum / n;
  const double stddev = std::sqrt(sum2 / n - mean * mean);
  EXPECT_NEAR(stddev, 1.0, 0.1);
  EXPECT_NEAR(mean, 0.0, 4.0 / std::sqrt(n));
}

TEST(EncoderModel, SlipVariancePerDistanceIndependentOfSampling)
{
  // 같은 총 회전 50 rad 를 주기 0.05 rad (저속·고주기) 와 0.5 rad (고속·저주기) 로 굴려도
  // 누적 슬립 분산이 같다: σ_s² φ_ref Θ (리뷰: 이전 주기당 모델은 √(v/f) 로 달라졌다)
  EncoderParams p;
  p.quantize = false;
  for (const double step : {0.05, 0.5}) {
    const int runs = 2000;
    double sum2 = 0.0;
    for (int r = 0; r < runs; ++r) {
      WheelEncoderModel enc(p, 100 + static_cast<std::uint64_t>(r));
      enc.update(0.0);
      double measured = 0.0;
      const int steps = static_cast<int>(std::lround(50.0 / step));
      for (int k = 1; k <= steps; ++k) {
        measured += enc.update(step * k).delta_angle;
      }
      sum2 += (measured - 50.0) * (measured - 50.0);
    }
    const double expected = p.slip_noise_stddev * p.slip_noise_stddev *
      p.slip_reference_angle * 50.0;
    EXPECT_NEAR(sum2 / runs / expected, 1.0, 0.1) << "step " << step;
  }
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
  // 감긴 입력(wrapped_input): 조인트 각이 (−π, π] 로 들어와도 누적 틱은 연속이다 (주기당 < π)
  EncoderParams p = quantOnly();
  p.wrapped_input = true;
  WheelEncoderModel enc(p, 1);
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

TEST(EncoderModel, MultiTurnCounterKeepsLargeIncrements)
{
  // 연속 누적각 입력(Gazebo): 주기 차가 π 를 넘어도(샘플 공백) 접지 않는다 — 한 바퀴를 잃지 않는다
  WheelEncoderModel enc(quantOnly(), 1);
  enc.update(0.0, 12.1, 0.0);
  EncoderReading r = enc.update(3.63, 12.1, 0.3);       // 1 m/s, 0.3 s 공백
  EXPECT_NEAR(r.delta_angle, 3.63, enc.tickAngle());
  EXPECT_FALSE(r.ambiguous);
  r = enc.update(3.63 + 7.27, 24.2, 0.3);               // 2 m/s, 0.3 s 공백
  EXPECT_NEAR(enc.unwrappedAngle(), 10.9, 1e-12);
  EXPECT_FALSE(r.ambiguous);
  // ω_max·Δt + π 를 넘는 점프는 불가능 → 표시
  r = enc.update(200.0, 24.2, 0.02);
  EXPECT_TRUE(r.ambiguous);
}

TEST(EncoderModel, WrappedInputBranchFromVelocityHint)
{
  EncoderParams p = quantOnly();
  p.wrapped_input = true;
  WheelEncoderModel enc(p, 1);
  // 참 회전 7.27 rad (0.3 s × 24.2 rad/s) 가 (−π, π] 로 감겨 0.987 rad 로 보인다
  enc.update(0.0, 24.2, 0.0);
  EncoderReading r = enc.update(std::remainder(7.27, 2.0 * M_PI), 24.2, 0.3);
  EXPECT_NEAR(r.true_delta_angle, 7.27, 1e-9);
  EXPECT_FALSE(r.ambiguous);
  // 힌트 없음 + 공백 0.3 s (> π/ω_max = 0.105 s): 접되 모호로 표시
  WheelEncoderModel blind(p, 1);
  blind.update(0.0);
  r = blind.update(
    std::remainder(7.27, 2.0 * M_PI), std::numeric_limits<double>::quiet_NaN(), 0.3);
  EXPECT_TRUE(r.ambiguous);
  EXPECT_NEAR(r.true_delta_angle, std::remainder(7.27, 2.0 * M_PI), 1e-12);
  // 공백 0.02 s 는 모호하지 않다
  r = blind.update(
    std::remainder(7.27 + 0.4, 2.0 * M_PI), std::numeric_limits<double>::quiet_NaN(), 0.02);
  EXPECT_FALSE(r.ambiguous);
  EXPECT_NEAR(r.true_delta_angle, 0.4, 1e-9);
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
