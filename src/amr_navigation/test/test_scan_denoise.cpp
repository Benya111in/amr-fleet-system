// core::denoiseRanges — σ 0.03 m 벽 잡음 억제, 얇은 물체·불연속 보존, 비유한 값·wrap 처리.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

#include "amr_navigation/core/scan_denoise.hpp"

using amr_navigation::core::denoiseRanges;
using amr_navigation::core::excludeDiscsFromScan;
using amr_navigation::core::Point2D;
using amr_navigation::core::ScanDenoiseConfig;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kBeams = 720;          // 0.5° (config/sensors.yaml lidar.samples)
constexpr double kInc = 2.0 * kPi / kBeams;

// 0.60 m 통로 한가운데의 LiDAR: 벽 x = ±0.30, 통로 길이 방향(y)으로 4 m 안에서만 끝점
std::vector<float> aisleScan(double sigma, std::mt19937 & rng)
{
  std::normal_distribution<double> noise(0.0, sigma);
  std::vector<float> r(kBeams, std::numeric_limits<float>::infinity());
  for (int i = 0; i < kBeams; ++i) {
    const double th = -kPi + i * kInc;
    const double c = std::cos(th);
    if (std::abs(c) < 1e-6) {
      continue;
    }
    const double d = 0.30 / std::abs(c);
    if (d * std::abs(std::sin(th)) > 4.0) {
      continue;
    }
    r[static_cast<std::size_t>(i)] = static_cast<float>(d + noise(rng));
  }
  return r;
}

// 끝점의 통로 안쪽 침범량 0.30 − |x| (> 0: 벽 안쪽, 로봇 쪽)
std::vector<double> intrusions(const std::vector<float> & r)
{
  std::vector<double> out;
  for (int i = 0; i < kBeams; ++i) {
    const float v = r[static_cast<std::size_t>(i)];
    if (std::isfinite(v)) {
      out.push_back(0.30 - std::abs(v * std::cos(-kPi + i * kInc)));
    }
  }
  return out;
}
}  // namespace

TEST(ScanDenoise, AisleWallNoiseStaysOutsideTheRobotBand)
{
  // σ 0.03 의 원 끝점은 매 스캔 여러 개가 0.05 m 이상 안쪽에 찍힌다
  // (0.60 m 통로에서 로봇 외곽선 셀이 치명).
  // 게이트 중앙값 뒤에는 200 스캔 동안 0.05 m 침범이 없고 벽 위치 편향도 없어야 한다.
  std::mt19937 rng(7);
  const ScanDenoiseConfig cfg;   // ±5 빔, 게이트 0.15, 지지 3
  int raw_ge5 = 0;
  int filt_ge5 = 0;
  int raw_n = 0;
  double filt_sum = 0.0;
  double filt_max = -1.0;
  int filt_n = 0;
  for (int s = 0; s < 200; ++s) {
    const auto raw = aisleScan(0.03, rng);
    const auto filt = denoiseRanges(raw, cfg, true);
    for (double e : intrusions(raw)) {
      raw_ge5 += e >= 0.05;
      ++raw_n;
    }
    for (double e : intrusions(filt)) {
      filt_ge5 += e >= 0.05;
      filt_sum += e;
      filt_max = std::max(filt_max, e);
      ++filt_n;
    }
  }
  std::printf(
    "[ info ] 200 scans, %d wall points: raw >= 0.05 m inward %d (%.2f %%), filtered %d, "
    "filtered max intrusion %.4f m, mean %.4f m\n", raw_n, raw_ge5, 100.0 * raw_ge5 / raw_n,
    filt_ge5,
    filt_max, filt_sum / filt_n);
  EXPECT_GT(raw_ge5, 200);                         // 원 스캔: 스캔당 1 개 이상
  EXPECT_EQ(filt_ge5, 0) << "max intrusion " << filt_max;
  EXPECT_LT(filt_max, 0.05);
  EXPECT_LT(std::abs(filt_sum / filt_n), 0.003);   // 벽 위치 편향 < 3 mm
  EXPECT_GT(raw_n, 100000);
}

TEST(ScanDenoise, ThinObjectsAndStepsArePreserved)
{
  // 벽 5 m 앞의 1 빔·2 빔 기둥(2 m)과 1 m → 3 m 계단: 배경 빔은 게이트로 빠져
  // 물체가 지워지지 않는다
  std::vector<float> r(100, 5.0f);
  r[20] = 2.0f;                  // 1 빔 → 지지 1 < 3 → 원래 값
  r[40] = 2.0f;
  r[41] = 2.04f;                 // 2 빔 → 지지 2 < 3 → 원래 값
  for (int i = 60; i < 100; ++i) {
    r[static_cast<std::size_t>(i)] = i < 80 ? 1.0f : 3.0f;
  }
  const auto f = denoiseRanges(r, ScanDenoiseConfig(), false);
  EXPECT_FLOAT_EQ(f[20], 2.0f);
  EXPECT_FLOAT_EQ(f[40], 2.0f);
  EXPECT_FLOAT_EQ(f[41], 2.04f);
  EXPECT_FLOAT_EQ(f[19], 5.0f);
  EXPECT_FLOAT_EQ(f[79], 1.0f);
  EXPECT_FLOAT_EQ(f[80], 3.0f);
  // 3 빔 물체는 자기들끼리의 중앙값 (배경 5 m 가 섞이지 않는다)
  std::vector<float> q(30, 5.0f);
  q[10] = 2.0f;
  q[11] = 2.1f;
  q[12] = 2.02f;
  const auto g = denoiseRanges(q, ScanDenoiseConfig(), false);
  EXPECT_FLOAT_EQ(g[10], 2.02f);
  EXPECT_FLOAT_EQ(g[11], 2.02f);
  EXPECT_FLOAT_EQ(g[12], 2.02f);
}

TEST(ScanDenoise, NonFiniteBeamsAndWrap)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> r = {1.0f, 1.1f, nan, 1.05f, inf, 1.0f, 3.0f, 3.0f, 3.0f, 0.9f};
  ScanDenoiseConfig c;
  c.half_window = 2;
  c.min_support = 2;
  const auto f = denoiseRanges(r, c, true);
  EXPECT_TRUE(std::isnan(f[2]));
  EXPECT_TRUE(std::isinf(f[4]));
  // wrap: 빔 0 의 이웃 = 8(3.0, 게이트 밖), 9(0.9), 1(1.1), 2(NaN) → {1.0, 0.9, 1.1} 중앙값 1.0
  EXPECT_FLOAT_EQ(f[0], 1.0f);
  // wrap 없음: 빔 9 의 이웃 = 7, 8 (3.0, 게이트 밖) → 지지 1 < 2 → 원래 값
  const auto g = denoiseRanges(r, c, false);
  EXPECT_FLOAT_EQ(g[9], 0.9f);
  // 빔 1 의 이웃 {1.0, 1.1, 1.05} (NaN 제외, wrap 없음) → 1.05
  EXPECT_FLOAT_EQ(g[1], 1.05f);
  c.half_window = 0;
  EXPECT_EQ(denoiseRanges(r, c, true)[1], r[1]);
  EXPECT_TRUE(denoiseRanges({}, ScanDenoiseConfig(), true).empty());
}

TEST(ScanDenoise, EvenSupportAveragesMiddlePair)
{
  std::vector<float> r = {1.0f, 1.1f, 1.2f, 1.3f};
  ScanDenoiseConfig c;
  c.half_window = 1;
  c.min_support = 2;
  const auto f = denoiseRanges(r, c, false);
  EXPECT_FLOAT_EQ(f[0], 1.05f);   // {1.0, 1.1}
  EXPECT_FLOAT_EQ(f[1], 1.1f);    // {1.0, 1.1, 1.2}
  EXPECT_FLOAT_EQ(f[3], 1.25f);   // {1.2, 1.3}
}

TEST(ScanDenoise, ExcludeDiscsRemovesOnlyBeamsInsideTracks)
{
  // 벽 3 m (전 방향) 과 앞 1.5 m 의 사람(반경 0.25): 사람 끝점만 +inf, 벽은 그대로
  std::vector<float> r(720, 3.0f);
  for (int i = 0; i < 720; ++i) {
    const double a = -kPi + i * kInc;
    if (std::abs(a) < std::atan2(0.25, 1.5)) {
      r[static_cast<std::size_t>(i)] = static_cast<float>(1.5 - 0.25 * std::cos(a));
    }
  }
  std::vector<float> f = r;
  const std::size_t n = excludeDiscsFromScan(f, -kPi, kInc, {Point2D{1.5, 0.0}}, 0.55);
  EXPECT_GT(n, 10U);
  int person = 0;
  for (int i = 0; i < 720; ++i) {
    if (r[static_cast<std::size_t>(i)] < 2.0f) {
      ++person;
      EXPECT_TRUE(std::isinf(f[static_cast<std::size_t>(i)]));
    } else {
      EXPECT_FLOAT_EQ(f[static_cast<std::size_t>(i)], 3.0f);
    }
  }
  EXPECT_EQ(static_cast<int>(n), person);
  // 트랙 없음·반경 0 → 그대로
  std::vector<float> g = r;
  EXPECT_EQ(excludeDiscsFromScan(g, -kPi, kInc, {}, 0.55), 0U);
  EXPECT_EQ(excludeDiscsFromScan(g, -kPi, kInc, {Point2D{1.5, 0.0}}, 0.0), 0U);
  EXPECT_EQ(g, r);
}
