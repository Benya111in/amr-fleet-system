// 스캔 필터 단위 테스트: 거리/각도 필터 규약, 고립 아웃라이어 제거, 벽 점 보존, 섀도우 제거,
// 메타데이터(빔 수) 보존, 360° 감김.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "amr_localization/scan_filter.hpp"

using amr_localization::ScanFilter;
using amr_localization::ScanFilterParams;
using amr_localization::ScanFilterStats;

namespace
{
// config/sensors.yaml: 720 빔, −180° ~ +179.5°, 0.5° 간격
constexpr int kBeams = 720;
constexpr double kAngleMin = -M_PI;
const double kInc = 2.0 * M_PI / kBeams;

/// 로봇을 중심으로 반지름 r 의 원형 벽 (모든 빔이 r).
std::vector<float> circleScan(float r)
{
  return std::vector<float>(kBeams, r);
}

/// x = d 인 직선 벽 (전방 ±60° 만 보임, 나머지 미검출).
std::vector<float> wallScan(double d)
{
  std::vector<float> s(kBeams, std::numeric_limits<float>::infinity());
  for (int i = 0; i < kBeams; ++i) {
    const double a = kAngleMin + i * kInc;
    if (std::abs(a) < M_PI / 3.0) {
      s[i] = static_cast<float>(d / std::cos(a));
    }
  }
  return s;
}

int indexOf(double angle)
{
  return static_cast<int>(std::lround((angle - kAngleMin) / kInc));
}
}  // namespace

TEST(ScanFilter, RangeClipFollowsRep117)
{
  ScanFilterParams p;
  p.range_min = 0.2;
  p.range_max = 10.0;
  p.outlier_window = 0;
  p.shadow_filter_enabled = false;
  ScanFilter f(p);
  std::vector<float> in = circleScan(5.0f);
  in[0] = 0.05f;                                         // 너무 가까움
  in[1] = 12.0f;                                         // 너무 멂
  in[2] = std::numeric_limits<float>::infinity();        // 미검출
  in[3] = std::numeric_limits<float>::quiet_NaN();       // 오류
  in[4] = -std::numeric_limits<float>::infinity();
  std::vector<float> out;
  const ScanFilterStats s = f.apply(in, kAngleMin, kInc, 0.1, 25.0, out);
  ASSERT_EQ(out.size(), in.size());                      // 빔 수 보존
  EXPECT_TRUE(std::isinf(out[0]) && out[0] < 0);
  EXPECT_TRUE(std::isinf(out[1]) && out[1] > 0);
  EXPECT_TRUE(std::isinf(out[2]) && out[2] > 0);
  EXPECT_TRUE(std::isnan(out[3]));
  EXPECT_TRUE(std::isinf(out[4]) && out[4] < 0);
  EXPECT_FLOAT_EQ(out[5], 5.0f);
  EXPECT_EQ(s.too_close, 2u);
  EXPECT_EQ(s.too_far, 2u);
  EXPECT_EQ(s.nan_input, 1u);
  EXPECT_EQ(s.valid_output, static_cast<std::size_t>(kBeams - 5));
  // 적용 한계 = 센서와 파라미터 중 좁은 쪽
  EXPECT_DOUBLE_EQ(f.effectiveRangeMin(0.1), 0.2);
  EXPECT_DOUBLE_EQ(f.effectiveRangeMax(25.0), 10.0);
  EXPECT_DOUBLE_EQ(f.effectiveRangeMax(8.0), 8.0);
}

TEST(ScanFilter, AngleWindowAndMask)
{
  ScanFilterParams p;
  p.outlier_window = 0;
  p.shadow_filter_enabled = false;
  p.angle_window_min = -M_PI / 2.0;
  p.angle_window_max = M_PI / 2.0;           // 전방 반원만
  p.angle_mask = {{0.2, 0.3}};               // 적재물 가림 섹터
  ScanFilter f(p);
  std::vector<float> out;
  const auto s = f.apply(circleScan(3.0f), kAngleMin, kInc, 0.1, 25.0, out);
  EXPECT_TRUE(std::isnan(out[indexOf(M_PI * 0.75)]));
  EXPECT_TRUE(std::isnan(out[indexOf(-M_PI * 0.75)]));
  EXPECT_FLOAT_EQ(out[indexOf(0.0)], 3.0f);
  EXPECT_TRUE(std::isnan(out[indexOf(0.25)]));
  EXPECT_FLOAT_EQ(out[indexOf(0.35)], 3.0f);
  // 반원 361 빔 − 마스크 (0.2~0.3 rad → 12 빔)
  EXPECT_NEAR(static_cast<double>(s.valid_output), 361.0 - 12.0, 2.0);
  EXPECT_EQ(s.angle_removed, static_cast<std::size_t>(kBeams) - s.valid_output);
}

TEST(ScanFilter, WrappedAngleSectors)
{
  // 후방 섹터 [2.8, −2.8] 는 ±π 를 넘어가는 반시계 호
  EXPECT_TRUE(ScanFilter::inArc(M_PI, 2.8, -2.8));
  EXPECT_TRUE(ScanFilter::inArc(-3.0, 2.8, -2.8));
  EXPECT_FALSE(ScanFilter::inArc(0.0, 2.8, -2.8));
  EXPECT_TRUE(ScanFilter::inArc(1.0, -M_PI, M_PI));   // 전체
  ScanFilterParams p;
  p.outlier_window = 0;
  p.shadow_filter_enabled = false;
  p.angle_mask = {{2.8, -2.8}};
  ScanFilter f(p);
  EXPECT_TRUE(f.isAngleRemoved(-M_PI));
  EXPECT_FALSE(f.isAngleRemoved(1.0));
}

TEST(ScanFilter, RemovesIsolatedOutliersKeepsWall)
{
  ScanFilterParams p;
  p.shadow_filter_enabled = false;
  ScanFilter f(p);
  std::vector<float> in = wallScan(4.0);
  // σ = 0.03 m 가우시안 잡음 (명세)
  std::mt19937 rng(3);
  std::normal_distribution<float> noise(0.0f, 0.03f);
  for (auto & r : in) {
    if (std::isfinite(r)) {
      r += noise(rng);
    }
  }
  // 고립 스파이크 3 개 (먼지/반사): 벽 앞 1.5 m, 벽 뒤, 빈 공간 속 단일 점
  const int a = indexOf(0.1);
  const int b = indexOf(-0.4);
  const int c = indexOf(2.0);
  in[a] = 1.5f;
  in[b] = 9.0f;
  in[c] = 2.5f;
  std::vector<float> out;
  const auto s = f.apply(in, kAngleMin, kInc, 0.1, 25.0, out);
  EXPECT_TRUE(std::isnan(out[a]));
  EXPECT_TRUE(std::isnan(out[b]));
  EXPECT_TRUE(std::isnan(out[c]));
  EXPECT_EQ(s.outliers, 3u);
  // 잡음이 있는 벽 점은 모두 유지
  int wall_kept = 0;
  int wall_total = 0;
  for (int i = 0; i < kBeams; ++i) {
    if (i == a || i == b || i == c || !std::isfinite(in[i])) {
      continue;
    }
    ++wall_total;
    if (std::isfinite(out[i])) {
      ++wall_kept;
    }
  }
  EXPECT_EQ(wall_kept, wall_total);
}

TEST(ScanFilter, SmallClusterSurvivesWithNeighborSupport)
{
  // 두 빔짜리 작은 물체(예: 기둥 모서리)는 서로 지지하므로 유지 (min_neighbors = 1)
  ScanFilterParams p;
  p.shadow_filter_enabled = false;
  ScanFilter f(p);
  std::vector<float> in = circleScan(8.0f);
  in[100] = 2.0f;
  in[101] = 2.01f;
  std::vector<float> out;
  f.apply(in, kAngleMin, kInc, 0.1, 25.0, out);
  EXPECT_FLOAT_EQ(out[100], 2.0f);
  EXPECT_FLOAT_EQ(out[101], 2.01f);
  // 지지 이웃 2 개를 요구하면 두 빔 군집도 제거된다
  p.outlier_min_neighbors = 2;
  p.outlier_window = 1;
  ScanFilter strict(p);
  strict.apply(in, kAngleMin, kInc, 0.1, 25.0, out);
  EXPECT_TRUE(std::isnan(out[100]));
}

TEST(ScanFilter, FullCircleWrapsNeighborSearch)
{
  // 첫 빔(−180°)과 끝 빔(+179.5°)은 이웃이다: 첫 빔의 유일한 지지가 끝 빔일 때 유지돼야 한다
  ScanFilterParams p;
  p.shadow_filter_enabled = false;
  p.outlier_window = 1;
  ScanFilter f(p);
  std::vector<float> in(kBeams, std::numeric_limits<float>::infinity());
  in[0] = 3.0f;
  in[kBeams - 1] = 3.0f;
  std::vector<float> out;
  f.apply(in, kAngleMin, kInc, 0.1, 25.0, out);
  EXPECT_FLOAT_EQ(out[0], 3.0f);
  EXPECT_FLOAT_EQ(out[kBeams - 1], 3.0f);
  // 180° 스캔(감김 없음)에서는 같은 배치가 고립점
  std::vector<float> half(360, std::numeric_limits<float>::infinity());
  half[0] = 3.0f;
  half[359] = 3.0f;
  f.apply(half, -M_PI / 2.0, kInc, 0.1, 25.0, out);
  EXPECT_TRUE(std::isnan(out[0]));
}

TEST(ScanFilter, RemovesShadowVeilPoints)
{
  // 전경 물체(2 m) 모서리와 배경 벽(6 m) 사이의 혼합 픽셀(베일 점)
  ScanFilterParams p;
  p.outlier_window = 0;
  ScanFilter f(p);
  std::vector<float> in = circleScan(6.0f);
  for (int i = 300; i < 340; ++i) {
    in[i] = 2.0f;
  }
  in[340] = 4.0f;  // 베일: 2 m 와 6 m 사이
  std::vector<float> out;
  const auto s = f.apply(in, kAngleMin, kInc, 0.1, 25.0, out);
  EXPECT_TRUE(std::isnan(out[340]));
  EXPECT_GE(s.shadows, 1u);
  EXPECT_FLOAT_EQ(out[320], 2.0f);   // 전경 유지
  EXPECT_FLOAT_EQ(out[200], 6.0f);   // 배경 유지
  // 벽 정면의 인접 점(β ≈ 90°)은 섀도우가 아니다
  EXPECT_FLOAT_EQ(out[330], 2.0f);
}

TEST(ScanFilter, EmptyScan)
{
  ScanFilter f;
  std::vector<float> out{1.0f};
  const auto s = f.apply({}, kAngleMin, kInc, 0.1, 25.0, out);
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(s.input, 0u);
  EXPECT_DOUBLE_EQ(f.params().range_max, 25.0);
}
