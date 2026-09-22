// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 스캔 클러스터링 단위 테스트: 거리 변환, ABD 임계, 분할/병합/재분할, 배경 분리, 측정 모델.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "amr_perception/scan_clustering.hpp"
#include "scan_sim.hpp"

using amr_perception::Cluster;
using amr_perception::ClusterModelParams;
using amr_perception::LaserScanData;
using amr_perception::Pose2D;
using amr_perception::ScanPoint;
using amr_perception::SegmentationParams;
using amr_perception::StaticMapDistance;
using amr_perception::Vec2;

namespace
{
// 60 x 40 셀 (3 x 2 m) 지도에 x = 2.0 m 세로 벽 하나
StaticMapDistance wallMap()
{
  const int w = 60;
  const int h = 40;
  std::vector<int8_t> data(w * h, 0);
  for (int y = 0; y < h; ++y) {
    data[y * w + 40] = 100;  // 셀 40 → x ∈ [2.00, 2.05)
  }
  StaticMapDistance m;
  m.build(w, h, 0.05, Pose2D{0.0, -1.0, 0.0}, data, 65);
  return m;
}
}  // namespace

TEST(ScanClustering, DistanceTransform1dMatchesBruteForce)
{
  std::mt19937 rng(3);
  std::uniform_int_distribution<int> coin(0, 4);
  for (int trial = 0; trial < 50; ++trial) {
    std::vector<double> f(37);
    for (auto & v : f) {
      v = coin(rng) == 0 ? 0.0 : 1e20;
    }
    std::vector<double> d;
    amr_perception::distanceTransform1d(f, d);
    for (int q = 0; q < 37; ++q) {
      double best = 1e20;
      for (int p = 0; p < 37; ++p) {
        best = std::min(best, (q - p) * (q - p) + f[p]);
      }
      if (best < 1e19) {
        EXPECT_NEAR(d[q], best, 1e-6);
      } else {
        EXPECT_GT(d[q], 1e19);
      }
    }
  }
  std::vector<double> d;
  amr_perception::distanceTransform1d({}, d);
  EXPECT_TRUE(d.empty());
}

TEST(ScanClustering, StaticMapDistanceLookup)
{
  const StaticMapDistance m = wallMap();
  ASSERT_TRUE(m.valid());
  EXPECT_EQ(m.width(), 60);
  EXPECT_EQ(m.height(), 40);
  EXPECT_NEAR(m.distance(Vec2(2.02, 0.0)), 0.0, 1e-6);
  EXPECT_NEAR(m.distance(Vec2(1.52, 0.0)), 0.5, 0.051);
  EXPECT_TRUE(std::isinf(m.distance(Vec2(10.0, 0.0))));  // 지도 밖
  StaticMapDistance empty;
  EXPECT_FALSE(empty.valid());
  EXPECT_TRUE(std::isinf(empty.distance(Vec2(0.0, 0.0))));
  // 크기 불일치 → invalid
  StaticMapDistance bad;
  bad.build(10, 10, 0.05, Pose2D{}, std::vector<int8_t>(5, 0), 65);
  EXPECT_FALSE(bad.valid());
  // 점유 셀 없음 → 모든 거리 inf
  StaticMapDistance free_map;
  free_map.build(4, 4, 0.05, Pose2D{}, std::vector<int8_t>(16, 0), 65);
  EXPECT_TRUE(std::isinf(free_map.distance(Vec2(0.1, 0.1))));
}

TEST(ScanClustering, AbdThresholdValues)
{
  const double lambda = 10.0 * M_PI / 180.0;
  const double dphi = 0.5 * M_PI / 180.0;
  // 연구 브리프 §3.3 수치: 3 m 0.25, 5 m 0.35, 8 m 0.51 m
  EXPECT_NEAR(amr_perception::abdThreshold(3.0, dphi, lambda, 0.03), 0.25, 0.005);
  EXPECT_NEAR(amr_perception::abdThreshold(5.0, dphi, lambda, 0.03), 0.35, 0.01);
  EXPECT_NEAR(amr_perception::abdThreshold(8.0, dphi, lambda, 0.03), 0.51, 0.01);
  EXPECT_LT(amr_perception::abdThreshold(3.0, lambda, lambda, 0.03), 0.0);
}

TEST(ScanClustering, SeparatesTwoObjectsAndBuildsMeasurement)
{
  std::mt19937 rng(1);
  const Pose2D sensor{0.0, 0.0, 0.0};
  const auto scan = scan_sim::makeScan(
    sensor, {{Vec2(3.0, 0.5), 0.2}, {Vec2(3.0, -0.5), 0.2}}, {}, 0.01, &rng);
  const auto clusters = amr_perception::extractClusters(
    scan, sensor, nullptr, Pose2D{}, SegmentationParams{}, ClusterModelParams{});
  ASSERT_EQ(clusters.size(), 2U);
  for (const auto & c : clusters) {
    // 편향 보정 측정은 보이는 표면 중심보다 원 중심에 가깝다 (반원호 중심 = 2R/π = 0.127 m 앞)
    const Vec2 center = c.centroid.y() > 0 ? Vec2(3.0, 0.5) : Vec2(3.0, -0.5);
    EXPECT_LT((c.measurement - center).norm(), (c.centroid - center).norm());
    EXPECT_LT((c.measurement - center).norm(), 0.06);
    EXPECT_GE(c.num_points, 3);
    EXPECT_NEAR(c.mean_range, 3.0 - 0.18, 0.08);
    // 시선 방향(대략 x) 분산이 가로(y) 분산보다 크다: σ_δ 0.10 이 지배
    EXPECT_GT(c.R(0, 0), c.R(1, 1));
    EXPECT_GT(c.radius, 0.1);
    EXPECT_LT(c.length, 0.5);
    EXPECT_LE(c.bearing_min, c.bearing_max);
  }
}

TEST(ScanClustering, BackgroundWallRemovedAndPersonNearWallKept)
{
  std::mt19937 rng(2);
  const StaticMapDistance map = wallMap();
  const Pose2D sensor{0.0, 0.0, 0.0};
  // 벽(지도에 있음) + 벽에서 0.35 m 떨어진 사람(지도에 없음)
  const auto scan = scan_sim::makeScan(
    sensor, {{Vec2(1.45, 0.3), 0.15}}, {{Vec2(2.025, -0.95), Vec2(2.025, 0.95)}}, 0.01, &rng);
  SegmentationParams seg;
  const auto with_map = amr_perception::extractClusters(
    scan, sensor, &map, Pose2D{}, seg, ClusterModelParams{});
  ASSERT_EQ(with_map.size(), 1U);
  EXPECT_NEAR(with_map.front().measurement.x(), 1.45, 0.08);
  EXPECT_NEAR(with_map.front().measurement.y(), 0.3, 0.08);
  EXPECT_LT(with_map.front().map_overlap, 0.5);
  // 지도 없이 돌리면 벽도 클러스터가 된다 (재분할로 여러 개일 수 있음)
  const auto without_map = amr_perception::extractClusters(
    scan, sensor, nullptr, Pose2D{}, seg, ClusterModelParams{});
  EXPECT_GT(without_map.size(), 1U);
}

TEST(ScanClustering, BackgroundLabelUsesMapTransform)
{
  // 추적 프레임(odom)과 map 사이 오프셋: map_from_tracking = (+1, 0)
  const StaticMapDistance map = wallMap();
  std::vector<ScanPoint> pts(2);
  pts[0].position = Vec2(1.02, 0.0);  // map (2.02, 0) → 벽
  pts[1].position = Vec2(0.0, 0.0);   // map (1.0, 0) → 자유
  amr_perception::labelBackground(pts, map, Pose2D{1.0, 0.0, 0.0}, 0.10);
  EXPECT_TRUE(pts[0].background);
  EXPECT_FALSE(pts[1].background);
  EXPECT_NEAR(pts[1].map_distance, 1.0, 0.051);
}

TEST(ScanClustering, FullCircleSeamJoinsSegments)
{
  std::mt19937 rng(4);
  const Pose2D sensor{0.0, 0.0, 0.0};
  // 정확히 후방(±180° 경계)에 걸친 물체
  const auto scan = scan_sim::makeScan(sensor, {{Vec2(-2.0, 0.0), 0.25}}, {}, 0.005, &rng);
  const auto clusters = amr_perception::extractClusters(
    scan, sensor, nullptr, Pose2D{}, SegmentationParams{}, ClusterModelParams{});
  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_NEAR(clusters.front().measurement.x(), -2.0, 0.08);
  EXPECT_NEAR(clusters.front().measurement.y(), 0.0, 0.05);
}

TEST(ScanClustering, OversizedSegmentIsSplit)
{
  // 1 m 간격으로 두 줄 점: ABD 는 이어 붙이지만(간격 0.12 m 연속) 장축 3 m → 재분할
  std::vector<ScanPoint> pts;
  for (int i = 0; i < 30; ++i) {
    ScanPoint p;
    p.position = Vec2(2.0, -1.5 + 0.1 * i + (i >= 15 ? 0.3 : 0.0));
    p.range = p.position.norm();
    p.beam = i;
    pts.push_back(p);
  }
  std::vector<std::vector<int>> seg(1);
  for (int i = 0; i < 30; ++i) {
    seg[0].push_back(i);
  }
  const auto out = amr_perception::splitOversized(pts, seg, 1.5);
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].size(), 15U);
  EXPECT_EQ(out[1].size(), 15U);
  // 짧은 세그먼트는 그대로
  EXPECT_EQ(amr_perception::splitOversized(pts, seg, 5.0).size(), 1U);
}

TEST(ScanClustering, MergeFragmentsWithSmallGap)
{
  std::vector<ScanPoint> pts(4);
  pts[0].position = Vec2(1.0, 0.0);
  pts[1].position = Vec2(1.0, 0.05);
  pts[2].position = Vec2(1.0, 0.12);   // 0.07 m 간격 → 병합
  pts[3].position = Vec2(1.0, 0.60);   // 멀다
  std::vector<std::vector<int>> seg{{0, 1}, {2}, {3}};
  auto out = amr_perception::mergeSegments(pts, seg, 0.10);
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].size(), 3U);
  // 라벨이 다르면 병합하지 않는다
  pts[2].background = true;
  out = amr_perception::mergeSegments(pts, seg, 0.10);
  EXPECT_EQ(out.size(), 3U);
}

TEST(ScanClustering, MinPointsAndRangeFilters)
{
  LaserScanData scan;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / 720;
  scan.range_min = 0.1;
  scan.range_max = 25.0;
  scan.ranges.assign(720, std::numeric_limits<float>::infinity());
  // 2 점짜리 가까운 클러스터 (최소 3 점 필요) → 버림
  scan.ranges[360] = 2.0F;
  scan.ranges[361] = 2.0F;
  // 너무 가까운(range_min 미만) 점 / NaN
  scan.ranges[100] = 0.05F;
  scan.ranges[101] = std::numeric_limits<float>::quiet_NaN();
  // 원거리 2 점 (far_range 6 m 밖 → 최소 2 점) → 유지
  scan.ranges[500] = 7.0F;
  scan.ranges[501] = 7.0F;
  const auto clusters = amr_perception::extractClusters(
    scan, Pose2D{}, nullptr, Pose2D{}, SegmentationParams{}, ClusterModelParams{});
  ASSERT_EQ(clusters.size(), 1U);
  EXPECT_NEAR(clusters.front().mean_range, 7.0, 1e-6);
  const auto pts = amr_perception::projectScan(scan, Pose2D{}, 5.0);  // max_range 5 → 원거리 제외
  EXPECT_EQ(pts.size(), 2U);
}

TEST(ScanClustering, SegmentScanHandlesEmptyAndLabelChange)
{
  EXPECT_TRUE(
    amr_perception::segmentScan({}, 0.0087, SegmentationParams{}, false, 720).empty());
  std::vector<ScanPoint> pts(3);
  for (int i = 0; i < 3; ++i) {
    pts[i].position = Vec2(2.0, 0.017 * i);
    pts[i].range = 2.0;
    pts[i].beam = i;
  }
  pts[2].background = true;  // 라벨이 바뀌면 끊긴다
  const auto seg = amr_perception::segmentScan(pts, 0.0087, SegmentationParams{}, false, 720);
  EXPECT_EQ(seg.size(), 2U);
  // 빈 인덱스 클러스터
  const Cluster c = amr_perception::buildCluster(pts, {}, Vec2::Zero(), 0.0087, {});
  EXPECT_EQ(c.num_points, 0);
}

TEST(ScanClustering, PartialOcclusionInflatesLateralCovariance)
{
  // 센서 앞 2 m 기둥이 5 m 뒤 물체의 한쪽을 가린다 → 뒤 물체는 occluded, 가로 분산 증가
  std::mt19937 rng(6);
  const Pose2D sensor{0.0, 0.0, 0.0};
  const auto scan = scan_sim::makeScan(
    sensor, {{Vec2(2.0, 0.0), 0.15}, {Vec2(5.0, 0.3), 0.25}}, {}, 0.005, &rng);
  const auto clusters = amr_perception::extractClusters(
    scan, sensor, nullptr, Pose2D{}, SegmentationParams{}, ClusterModelParams{});
  ASSERT_EQ(clusters.size(), 2U);
  const auto & near = clusters[0].mean_range < clusters[1].mean_range ? clusters[0] : clusters[1];
  const auto & far = clusters[0].mean_range < clusters[1].mean_range ? clusters[1] : clusters[0];
  EXPECT_FALSE(near.occluded);
  EXPECT_TRUE(far.occluded);
  // 시선(≈x) 수직(y) 분산 ≥ σ_occ² = 0.04
  EXPECT_GT(far.R(1, 1), 0.035);
  EXPECT_LT(near.R(1, 1), 0.01);
  // 가리는 것이 없으면 표시 안 함
  const auto alone = amr_perception::extractClusters(
    scan_sim::makeScan(sensor, {{Vec2(5.0, 0.3), 0.25}}, {}, 0.005, &rng), sensor, nullptr,
    Pose2D{}, SegmentationParams{}, ClusterModelParams{});
  ASSERT_EQ(alone.size(), 1U);
  EXPECT_FALSE(alone.front().occluded);
}
