// 경로 후처리(숏컷·재표본·경사 하강 평활화·방향) 단위 테스트.
// 핵심: 평활화 후에도 여유거리(최대 셀 비용)가 원래 A* 경로보다 나빠지지 않는다.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "amr_navigation/core/astar.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/path_smoother.hpp"

using amr_navigation::core::AStar;
using amr_navigation::core::AStarResult;
using amr_navigation::core::Cell;
using amr_navigation::core::CostGrid;
using amr_navigation::core::OwnedGrid;
using amr_navigation::core::PathSmoother;
using amr_navigation::core::Point2D;
using amr_navigation::core::Pose2D;
using amr_navigation::core::SmootherConfig;
using amr_navigation::core::SmootherStats;
using amr_navigation::core::inflate;
using amr_navigation::core::kLethalObstacle;
using amr_navigation::core::kNoInformation;
using amr_navigation::core::kPi;
using amr_navigation::core::wrapAngle;

namespace
{
// 10 x 6 m 방: 가운데 선반 두 개 사이 1.2 m 통로, Nav2 식 팽창 (r_ins 0.2, R 1.2, s 2.0)
OwnedGrid rackRoom()
{
  OwnedGrid g(200, 120, 0.05);
  g.fillRect(0.0, 0.0, 10.0, 0.05, kLethalObstacle);
  g.fillRect(0.0, 5.95, 10.0, 6.0, kLethalObstacle);
  g.fillRect(3.0, 0.0, 7.0, 2.4, kLethalObstacle);
  g.fillRect(3.0, 3.6, 7.0, 6.0, kLethalObstacle);
  inflate(g, 0.2, 1.2, 2.0);
  return g;
}

// 연속 경로를 5 mm 간격으로 따라가며 본 최대 셀 비용 (미지 제외)
int maxCostAlong(const CostGrid & g, const std::vector<Point2D> & pts)
{
  int mx = 0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const double len = std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    const int n = std::max(1, static_cast<int>(std::ceil(len / 0.005)));
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      const uint8_t c = g.costAtWorld(
        pts[i - 1].x + t * (pts[i].x - pts[i - 1].x), pts[i - 1].y + t * (pts[i].y - pts[i - 1].y));
      if (c != kNoInformation) {
        mx = std::max(mx, static_cast<int>(c));
      }
    }
  }
  return mx;
}

std::vector<Point2D> toPoints(const std::vector<Pose2D> & poses)
{
  std::vector<Point2D> out;
  for (const auto & p : poses) {
    out.push_back({p.x, p.y});
  }
  return out;
}
}  // namespace

TEST(PathSmoother, ShortcutRemovesStaircase)
{
  OwnedGrid g(100, 100, 0.05);
  AStar astar;
  // 계단 경로: (0,0) → (40,20) 의 8-연결 격자 경로
  const AStarResult r = astar.plan(g.view(), {5, 5}, {45, 25});
  ASSERT_TRUE(r.ok());
  PathSmoother sm;
  const auto pts = PathSmoother::cellsToWorld(g.view(), r.path);
  const auto keep = sm.shortcut(g.view(), pts, astar.traversalTable());
  // 빈 방에서는 현 하나로 충분하다
  ASSERT_EQ(keep.size(), 2U);
  EXPECT_EQ(keep.front(), 0U);
  EXPECT_EQ(keep.back(), pts.size() - 1);
}

TEST(PathSmoother, ShortcutRespectsLengthAndObstacles)
{
  OwnedGrid g(100, 40, 0.05);
  g.fillRect(2.0, 0.0, 2.1, 1.2, kLethalObstacle);   // 벽 (위쪽 0.8 m 열림)
  AStar astar;
  const AStarResult r = astar.plan(g.view(), {10, 5}, {90, 5});
  ASSERT_TRUE(r.ok());
  SmootherConfig cfg;
  cfg.shortcut_max_length = 1.0;
  PathSmoother sm(cfg);
  const auto pts = PathSmoother::cellsToWorld(g.view(), r.path);
  const auto keep = sm.shortcut(g.view(), pts, astar.traversalTable());
  for (std::size_t i = 1; i < keep.size(); ++i) {
    const Point2D a = pts[keep[i - 1]];
    const Point2D b = pts[keep[i]];
    EXPECT_TRUE(sm.segmentFree(g.view(), a, b));
    // 한 칸 전진(인접 셀)이거나 길이 상한 이내
    EXPECT_TRUE(keep[i] == keep[i - 1] + 1 || std::hypot(b.x - a.x, b.y - a.y) <= 1.0 + 1e-9);
  }
}

TEST(PathSmoother, SegmentCostAndFreedom)
{
  OwnedGrid g(40, 40, 0.05);
  AStar astar;
  // 빈 격자: 비용 = 길이[셀] × m(0) = 길이
  EXPECT_NEAR(
    PathSmoother::segmentCost(g.view(), {0.1, 0.1}, {1.1, 0.1}, astar.traversalTable()), 20.0,
    1e-9);
  EXPECT_NEAR(
    PathSmoother::segmentCost(g.view(), {0.1, 0.1}, {0.1, 0.1}, astar.traversalTable()), 0.0,
    1e-12);
  g.at(10, 2) = kLethalObstacle;
  EXPECT_TRUE(
    std::isinf(
      PathSmoother::segmentCost(g.view(), {0.1, 0.1}, {1.1, 0.1}, astar.traversalTable())));
  PathSmoother sm;
  EXPECT_FALSE(sm.segmentFree(g.view(), {0.1, 0.1}, {1.1, 0.1}));
  EXPECT_TRUE(sm.segmentFree(g.view(), {0.1, 1.0}, {1.1, 1.0}));
  // 미지 셀: allow_unknown 에 따름
  g.at(5, 20) = kNoInformation;
  EXPECT_TRUE(sm.segmentFree(g.view(), {0.1, 1.025}, {1.1, 1.025}));
  SmootherConfig cfg;
  cfg.allow_unknown = false;
  PathSmoother strict(cfg);
  EXPECT_FALSE(strict.segmentFree(g.view(), {0.1, 1.025}, {1.1, 1.025}));
}

TEST(PathSmoother, ResampleSpacing)
{
  const std::vector<Point2D> pts{{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.52}};
  const auto out = PathSmoother::resample(pts, 0.1);
  ASSERT_GE(out.size(), 15U);
  for (std::size_t i = 1; i + 1 < out.size(); ++i) {
    const double d = std::hypot(out[i].x - out[i - 1].x, out[i].y - out[i - 1].y);
    EXPECT_LE(d, 0.1 + 1e-9);
    EXPECT_GE(d, 0.05);
  }
  EXPECT_NEAR(out.back().x, 1.0, 1e-12);
  EXPECT_NEAR(out.back().y, 0.52, 1e-12);
  // 간격 0 이하 → 그대로
  EXPECT_EQ(PathSmoother::resample(pts, 0.0).size(), pts.size());
}

TEST(PathSmoother, SmoothingKeepsClearanceInRackAisle)
{
  const OwnedGrid g = rackRoom();
  AStar astar;
  Cell s;
  Cell t;
  g.view().worldToMap(1.0, 1.0, s.x, s.y);
  g.view().worldToMap(9.0, 5.0, t.x, t.y);
  const AStarResult r = astar.plan(g.view(), s, t);
  ASSERT_TRUE(r.ok());
  const auto raw = PathSmoother::cellsToWorld(g.view(), r.path);
  const int raw_max = maxCostAlong(g.view(), raw);

  PathSmoother sm;
  SmootherStats st;
  const Point2D start{1.0, 1.0};
  const Point2D goal{9.0, 5.0};
  const double yaw = 0.5;
  const auto out = sm.process(g.view(), r.path, astar.traversalTable(), &start, &goal, &yaw, &st);
  ASSERT_GE(out.size(), 2U);
  const auto pts = toPoints(out);
  // 1) 여유거리 보존: 평활 경로의 최대 비용 ≤ 원래 격자 경로의 최대 비용 + 천장 여유(≈ 1 셀)
  EXPECT_LE(maxCostAlong(g.view(), pts), raw_max + sm.config().clearance_cost_margin);
  std::printf(
    "[ info ] rack aisle: raw max cost %d, smoothed %d, length %.3f -> %.3f m\n", raw_max,
    maxCostAlong(g.view(), pts), st.raw_length, st.smoothed_length);
  // 2) 통행 가능
  for (std::size_t i = 1; i < pts.size(); ++i) {
    EXPECT_TRUE(sm.segmentFree(g.view(), pts[i - 1], pts[i]));
  }
  // 3) 더 짧다(지그재그 제거), 끝점 정확
  EXPECT_LT(st.smoothed_length, st.raw_length);
  EXPECT_LE(st.shortcut_points, st.raw_points);
  EXPECT_NEAR(out.front().x, 1.0, 1e-12);
  EXPECT_NEAR(out.back().y, 5.0, 1e-12);
  EXPECT_NEAR(out.back().theta, yaw, 1e-12);
  EXPECT_GT(st.iterations, 0);
  // 4) 방향은 접선: 연속 자세의 방향 변화가 작다 (급격한 꺾임 없음)
  for (std::size_t i = 1; i + 2 < out.size(); ++i) {
    EXPECT_LT(std::abs(wrapAngle(out[i + 1].theta - out[i].theta)), 0.5);
  }
}

TEST(PathSmoother, ClearanceTermPushesAwayFromObstacle)
{
  // 벽을 스치는 직선 입력을 평활화하면 비용이 줄어드는 쪽(벽 반대)으로 밀린다
  OwnedGrid g(120, 60, 0.05);
  g.fillRect(0.0, 0.0, 6.0, 0.9, kLethalObstacle);
  inflate(g, 0.2, 1.2, 2.0);
  std::vector<Point2D> pts;
  for (int i = 0; i <= 80; ++i) {
    const double x = 1.0 + 0.05 * i;
    pts.push_back({x, 1.3 + 0.3 * std::sin(kPi * i / 80.0)});
  }
  SmootherConfig cfg;
  cfg.w_data = 0.05;
  cfg.w_smooth = 0.2;
  cfg.w_clearance = 0.5;
  cfg.clearance_cost_threshold = 50;
  PathSmoother sm(cfg);
  auto work = pts;
  bool rb = true;
  const int it = sm.smooth(g.view(), work, rb);
  EXPECT_FALSE(rb);
  EXPECT_GT(it, 0);
  // 양 끝 고정
  EXPECT_DOUBLE_EQ(work.front().y, pts.front().y);
  EXPECT_DOUBLE_EQ(work.back().y, pts.back().y);
  // 끝 근처(벽에 가까운 점)는 위로(+y) 밀린다
  EXPECT_GT(work[5].y, pts[5].y);
  EXPECT_LE(maxCostAlong(g.view(), work), maxCostAlong(g.view(), pts));
}

TEST(PathSmoother, ProjectionStopsAtObstacle)
{
  // 평활화 항이 점을 치명 셀로 끌어당기는 V 자 경로: 점별 사영이 막힌 결과를 남기지 않는다
  OwnedGrid g(60, 60, 0.05);
  g.fillRect(1.2, 0.0, 1.8, 1.2, kLethalObstacle);   // V 의 안쪽에 장애물
  std::vector<Point2D> pts;
  for (int i = 0; i <= 20; ++i) {
    pts.push_back({0.5 + 0.05 * i, 0.3 + 0.1 * i});   // 올라감
  }
  for (int i = 1; i <= 20; ++i) {
    pts.push_back({1.5 + 0.05 * i, 2.3 - 0.1 * i});   // 내려감
  }
  SmootherConfig cfg;
  cfg.w_data = 0.0;       // 데이터 항 없음 → 직선으로 수축하며 장애물로 들어가려 한다
  cfg.w_smooth = 0.5;
  cfg.w_clearance = 0.0;
  cfg.max_iterations = 2000;
  cfg.tolerance = 0.0;
  PathSmoother sm(cfg);
  auto work = pts;
  bool rb = true;
  sm.smooth(g.view(), work, rb);
  EXPECT_FALSE(rb);
  for (std::size_t i = 1; i < work.size(); ++i) {
    EXPECT_TRUE(sm.segmentFree(g.view(), work[i - 1], work[i]));
  }
  // 꼭짓점은 장애물 쪽(아래)으로 수축했지만 장애물 위(y ≥ 1.2)에 멈춘다
  EXPECT_LT(work[20].y, pts[20].y);
  EXPECT_GE(work[20].y, 1.2);
}

TEST(PathSmoother, RollbackGuardOnBlockedInput)
{
  // 입력 선분 일부가 이미 막힘(시작점이 장애물 안): 그 선분은 가드에서 면제, 나머지는 평활화
  OwnedGrid g(60, 60, 0.05);
  g.fillRect(0.0, 0.0, 0.3, 0.3, kLethalObstacle);
  std::vector<Point2D> pts;
  for (int i = 0; i <= 30; ++i) {
    pts.push_back({0.1 + 0.05 * i, 0.1 + 0.02 * i + 0.1 * std::sin(0.8 * i)});
  }
  PathSmoother sm;
  bool rb = true;
  const int it = sm.smooth(g.view(), pts, rb);
  EXPECT_FALSE(rb);
  EXPECT_GT(it, 0);
  EXPECT_EQ(sm.segmentMaxCost(g.view(), {0.1, 0.1}, {0.2, 0.1}), kLethalObstacle);
  EXPECT_EQ(sm.segmentMaxCost(g.view(), {1.0, 1.0}, {1.5, 1.0}), 0);
}

TEST(PathSmoother, OrientationsAndDegenerateInputs)
{
  const std::vector<Point2D> pts{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  const auto out = PathSmoother::assignOrientations(pts, nullptr);
  EXPECT_NEAR(out[0].theta, 0.0, 1e-12);
  EXPECT_NEAR(out[1].theta, std::atan2(1.0, 1.0), 1e-12);   // 중심차분
  EXPECT_NEAR(out[2].theta, kPi / 2.0, 1e-12);
  const double yaw = -1.0;
  const auto single = PathSmoother::assignOrientations({{2.0, 2.0}}, &yaw);
  ASSERT_EQ(single.size(), 1U);
  EXPECT_NEAR(single[0].theta, -1.0, 1e-12);

  OwnedGrid g(20, 20, 0.05);
  PathSmoother sm;
  AStar astar;
  EXPECT_TRUE(sm.process(g.view(), {}, astar.traversalTable(), nullptr, nullptr, nullptr).empty());
  // 한 셀 경로 + 시작·목표 좌표 → 두 점
  const Point2D s{0.51, 0.51};
  const Point2D t{0.54, 0.53};
  const auto two = sm.process(g.view(), {{10, 10}}, astar.traversalTable(), &s, &t, nullptr);
  ASSERT_EQ(two.size(), 2U);
  EXPECT_NEAR(two.back().x, 0.54, 1e-12);
  // 짧은 입력은 평활화하지 않는다
  std::vector<Point2D> short_pts{{0.0, 0.0}, {0.1, 0.0}};
  bool rb = true;
  EXPECT_EQ(sm.smooth(g.view(), short_pts, rb), 0);
  EXPECT_FALSE(rb);
}
