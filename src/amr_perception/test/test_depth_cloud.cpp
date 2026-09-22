// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 깊이 → 점군 단위 테스트: Pinhole 역투영, 인코딩, 거리 컷, 거리 제곱 노이즈 통계, voxel 다운샘플.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include "amr_perception/depth_cloud.hpp"

using amr_perception::DepthCloudParams;
using amr_perception::DepthEncoding;
using amr_perception::PinholeIntrinsics;
using amr_perception::Point3;

namespace
{
// sensors.yaml: 640×480, HFOV 87° → fx = 320 / tan(43.5°) = 337.2
PinholeIntrinsics camera()
{
  PinholeIntrinsics k;
  k.fx = 320.0 / std::tan(1.518436 / 2.0);
  k.fy = k.fx;
  k.cx = 319.5;
  k.cy = 239.5;
  return k;
}

std::vector<uint8_t> floatImage(int w, int h, float value)
{
  std::vector<uint8_t> buf(static_cast<std::size_t>(w) * h * sizeof(float));
  for (int i = 0; i < w * h; ++i) {
    std::memcpy(&buf[static_cast<std::size_t>(i) * sizeof(float)], &value, sizeof(float));
  }
  return buf;
}

void setFloat(std::vector<uint8_t> & buf, int w, int u, int v, float value)
{
  std::memcpy(&buf[(static_cast<std::size_t>(v) * w + u) * sizeof(float)], &value, sizeof(float));
}

DepthCloudParams noNoise()
{
  DepthCloudParams p;
  p.add_noise = false;
  p.leaf_size = 0.0;
  return p;
}
}  // namespace

TEST(DepthCloud, BackProjectInvertsProjection)
{
  const PinholeIntrinsics k = camera();
  EXPECT_NEAR(k.fx, 337.2, 0.1);
  // 광학 프레임 점 (X, Y, Z) → 픽셀 → 역투영 = 원래 점
  const double X = 0.4;
  const double Y = -0.25;
  const double Z = 2.5;
  const double u = k.fx * X / Z + k.cx;
  const double v = k.fy * Y / Z + k.cy;
  const Point3 p = amr_perception::backProject(u, v, Z, k);
  EXPECT_NEAR(p.x, X, 1e-6);
  EXPECT_NEAR(p.y, Y, 1e-6);
  EXPECT_NEAR(p.z, Z, 1e-6);
  EXPECT_FALSE(PinholeIntrinsics{}.valid());
}

TEST(DepthCloud, FlatWallFloat32AndInvalidPixels)
{
  const int w = 64;
  const int h = 48;
  PinholeIntrinsics k;
  k.fx = k.fy = 50.0;
  k.cx = 31.5;
  k.cy = 23.5;
  auto img = floatImage(w, h, 2.0F);
  setFloat(img, w, 0, 0, std::numeric_limits<float>::quiet_NaN());
  setFloat(img, w, 1, 0, std::numeric_limits<float>::infinity());
  setFloat(img, w, 2, 0, 0.0F);
  setFloat(img, w, 3, 0, 0.1F);   // range_min 0.2 미만
  setFloat(img, w, 4, 0, 7.0F);   // max_range 5 초과
  const auto pts = amr_perception::depthImageToPoints(
    img.data(), w, h, w * 4, DepthEncoding::kFloat32Meters, k, noNoise(), nullptr);
  ASSERT_EQ(pts.size(), static_cast<std::size_t>(w * h - 5));
  for (const auto & p : pts) {
    EXPECT_FLOAT_EQ(p.z, 2.0F);
  }
  // 첫 유효 픽셀 (u=5, v=0): X = (5 - 31.5)·2/50
  EXPECT_NEAR(pts.front().x, (5.0 - 31.5) * 2.0 / 50.0, 1e-6);
  EXPECT_NEAR(pts.front().y, (0.0 - 23.5) * 2.0 / 50.0, 1e-6);
  // pixel_step 부표본
  DepthCloudParams step = noNoise();
  step.pixel_step = 4;
  const auto sub = amr_perception::depthImageToPoints(
    img.data(), w, h, w * 4, DepthEncoding::kFloat32Meters, k, step, nullptr);
  // (0,0) NaN, (4,0) 7 m 는 제외
  EXPECT_EQ(sub.size(), static_cast<std::size_t>((w / 4) * (h / 4) - 2));
  // 잘못된 입력
  EXPECT_TRUE(
    amr_perception::depthImageToPoints(
      nullptr, w, h, w * 4, DepthEncoding::kFloat32Meters, k, noNoise(), nullptr).empty());
  EXPECT_TRUE(
    amr_perception::depthImageToPoints(
      img.data(), w, h, w * 4, DepthEncoding::kFloat32Meters, PinholeIntrinsics{}, noNoise(),
      nullptr).empty());
}

TEST(DepthCloud, Uint16MillimetersWithRowPadding)
{
  const int w = 8;
  const int h = 4;
  const int step = w * 2 + 6;  // 행 패딩
  std::vector<uint8_t> img(static_cast<std::size_t>(step) * h, 0);
  const uint16_t mm = 1500;
  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u) {
      if (u == 3 && v == 1) {
        continue;  // 0 = 무효
      }
      std::memcpy(&img[static_cast<std::size_t>(v) * step + u * 2], &mm, 2);
    }
  }
  PinholeIntrinsics k;
  k.fx = k.fy = 10.0;
  k.cx = 3.5;
  k.cy = 1.5;
  const auto pts = amr_perception::depthImageToPoints(
    img.data(), w, h, step, DepthEncoding::kUint16Millimeters, k, noNoise(), nullptr);
  ASSERT_EQ(pts.size(), static_cast<std::size_t>(w * h - 1));
  EXPECT_NEAR(pts.back().z, 1.5, 1e-6);
  EXPECT_NEAR(pts.back().x, (7 - 3.5) * 1.5 / 10.0, 1e-6);
}

TEST(DepthCloud, QuadraticNoiseHasExpectedSigma)
{
  // 3 m 평면: 가산 σ = k·Z² = 0.002·9 = 0.018 m
  const int w = 200;
  const int h = 100;
  auto img = floatImage(w, h, 3.0F);
  DepthCloudParams p;
  p.leaf_size = 0.0;
  p.add_noise = true;
  std::mt19937 rng(7);
  const auto pts = amr_perception::depthImageToPoints(
    img.data(), w, h, w * 4, DepthEncoding::kFloat32Meters, camera(), p, &rng);
  ASSERT_EQ(pts.size(), static_cast<std::size_t>(w * h));
  double mean = 0.0;
  for (const auto & q : pts) {
    mean += q.z;
  }
  mean /= static_cast<double>(pts.size());
  double var = 0.0;
  for (const auto & q : pts) {
    var += (q.z - mean) * (q.z - mean);
  }
  var /= static_cast<double>(pts.size() - 1);
  EXPECT_NEAR(mean, 3.0, 0.001);
  EXPECT_NEAR(std::sqrt(var), 0.018, 0.0008);  // 표본 2만 개: σ 추정 상대오차 ≈ 0.5 %
  // 노이즈 끔 / rng 없음 → 결정적
  p.add_noise = false;
  const auto clean = amr_perception::depthImageToPoints(
    img.data(), w, h, w * 4, DepthEncoding::kFloat32Meters, camera(), p, &rng);
  EXPECT_FLOAT_EQ(clean.front().z, 3.0F);
}

TEST(DepthCloud, VoxelDownsampleCentroidsAndFilters)
{
  std::vector<Point3> pts = {
    {0.01F, 0.01F, 1.01F}, {0.03F, 0.02F, 1.03F},  // 같은 칸 (leaf 0.05)
    {0.26F, 0.0F, 1.0F},                           // 다른 칸, 1 점
    {-0.01F, 0.0F, 1.0F},                          // 음수 인덱스 칸
  };
  const auto out = amr_perception::voxelDownsample(pts, 0.05, 1);
  ASSERT_EQ(out.size(), 3U);
  // 첫 등장 순서 유지 + 무게중심
  EXPECT_NEAR(out[0].x, 0.02, 1e-6);
  EXPECT_NEAR(out[0].y, 0.015, 1e-6);
  EXPECT_NEAR(out[0].z, 1.02, 1e-6);
  EXPECT_NEAR(out[1].x, 0.26, 1e-6);
  EXPECT_NEAR(out[2].x, -0.01, 1e-6);
  // 최소 점 수 필터 (고립 점 제거)
  const auto dense = amr_perception::voxelDownsample(pts, 0.05, 2);
  ASSERT_EQ(dense.size(), 1U);
  // leaf <= 0 → 그대로
  EXPECT_EQ(amr_perception::voxelDownsample(pts, 0.0, 1).size(), pts.size());
  // 큰 평면: 1 m × 1 m, 1 cm 간격 → 0.05 m 칸 20×20 = 400
  std::vector<Point3> plane;
  for (int i = 0; i < 100; ++i) {
    for (int j = 0; j < 100; ++j) {
      plane.push_back({0.005F + 0.01F * i, 0.005F + 0.01F * j, 2.0F});
    }
  }
  EXPECT_EQ(amr_perception::voxelDownsample(plane, 0.05, 1).size(), 400U);
}
