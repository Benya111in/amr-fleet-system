// 격자 뷰·팽창·선분 순회 단위 테스트.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <set>
#include <utility>

#include "amr_navigation/core/grid.hpp"

using amr_navigation::core::CostGrid;
using amr_navigation::core::OwnedGrid;
using amr_navigation::core::inflate;
using amr_navigation::core::inflationCost;
using amr_navigation::core::kInscribedInflated;
using amr_navigation::core::kLethalObstacle;
using amr_navigation::core::kNoInformation;
using amr_navigation::core::traceLine;
using amr_navigation::core::traceSupercover;

TEST(Grid, WorldMapRoundTrip)
{
  OwnedGrid g(40, 20, 0.05, 0, -1.0, -0.5);
  const CostGrid v = g.view();
  int mx = 0;
  int my = 0;
  ASSERT_TRUE(v.worldToMap(-1.0 + 0.051, -0.5 + 0.099, mx, my));
  EXPECT_EQ(mx, 1);
  EXPECT_EQ(my, 1);
  double wx = 0.0;
  double wy = 0.0;
  v.mapToWorld(3, 4, wx, wy);
  EXPECT_NEAR(wx, -1.0 + 3.5 * 0.05, 1e-12);
  EXPECT_NEAR(wy, -0.5 + 4.5 * 0.05, 1e-12);
  EXPECT_FALSE(v.worldToMap(-1.01, 0.0, mx, my));
  EXPECT_EQ(v.costAtWorld(100.0, 0.0), kLethalObstacle);   // 범위 밖 = 치명
  EXPECT_EQ(v.atOrLethal(-1, 0), kLethalObstacle);
}

TEST(Grid, InflationCostMatchesNav2Formula)
{
  const double r_ins = 0.2;
  const double s = 3.0;
  EXPECT_EQ(inflationCost(0.0, r_ins, s), kLethalObstacle);
  EXPECT_EQ(inflationCost(0.2, r_ins, s), kInscribedInflated);
  // 0.6 m 통로 중앙(벽까지 0.30 m): floor(252·e^{-0.3}) = 186
  EXPECT_EQ(inflationCost(0.30, r_ins, s), 186);
  EXPECT_EQ(inflationCost(0.30, r_ins, 2.0), 206);
  for (double d = 0.21; d < 2.0; d += 0.05) {
    const int expect = static_cast<int>(252.0 * std::exp(-s * (d - r_ins)));
    EXPECT_EQ(inflationCost(d, r_ins, s), expect) << d;
  }
}

TEST(Grid, InflateSingleObstacle)
{
  OwnedGrid g(41, 41, 0.05);
  g.at(20, 20) = kLethalObstacle;
  inflate(g, 0.2, 1.0, 3.0);
  EXPECT_EQ(g.at(20, 20), kLethalObstacle);
  EXPECT_EQ(g.at(24, 20), kInscribedInflated);      // 0.20 m
  EXPECT_EQ(g.at(26, 20), inflationCost(0.30, 0.2, 3.0));
  EXPECT_EQ(g.at(20, 40), inflationCost(1.0, 0.2, 3.0));   // 반경 1.0 m 경계는 포함
  EXPECT_EQ(g.at(40, 40), 0);                              // 1.41 m: 반경 밖
  EXPECT_EQ(g.at(0, 0), 0);
  // 대칭
  EXPECT_EQ(g.at(17, 23), g.at(23, 17));
}

TEST(Grid, InflateKeepsUnknown)
{
  OwnedGrid g(20, 20, 0.05);
  g.at(10, 10) = kLethalObstacle;
  g.at(11, 10) = kNoInformation;
  inflate(g, 0.1, 0.5, 3.0);
  EXPECT_EQ(g.at(11, 10), kNoInformation);
}

TEST(Grid, FillRect)
{
  OwnedGrid g(20, 20, 0.1);
  g.fillRect(0.5, 0.5, 0.95, 0.75, 7);
  EXPECT_EQ(g.at(5, 5), 7);
  EXPECT_EQ(g.at(9, 7), 7);
  EXPECT_EQ(g.at(9, 8), 0);
  EXPECT_EQ(g.at(4, 5), 0);
}

TEST(Grid, TraceLineVisitsEndpointsContiguously)
{
  std::vector<std::pair<int, int>> cells;
  traceLine(
    0, 0, 7, 3, [&](int x, int y) {
      cells.emplace_back(x, y);
      return true;
    });
  ASSERT_FALSE(cells.empty());
  EXPECT_EQ(cells.front(), std::make_pair(0, 0));
  EXPECT_EQ(cells.back(), std::make_pair(7, 3));
  for (std::size_t i = 1; i < cells.size(); ++i) {
    EXPECT_LE(std::abs(cells[i].first - cells[i - 1].first), 1);
    EXPECT_LE(std::abs(cells[i].second - cells[i - 1].second), 1);
  }
  // 조기 중단
  int count = 0;
  EXPECT_FALSE(traceLine(0, 0, 10, 0, [&](int, int) {return ++count < 3;}));
  EXPECT_EQ(count, 3);
}

TEST(Grid, SupercoverContainsAllSampledCells)
{
  OwnedGrid g(50, 50, 0.1);
  const CostGrid v = g.view();
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(0.05, 4.95);
  for (int trial = 0; trial < 300; ++trial) {
    const double x0 = u(rng);
    const double y0 = u(rng);
    const double x1 = u(rng);
    const double y1 = u(rng);
    std::set<std::pair<int, int>> visited;
    traceSupercover(
      v, x0, y0, x1, y1, [&](int x, int y) {
        visited.emplace(x, y);
        return true;
      });
    // 조밀 표본이 지나는 모든 셀이 방문 집합에 있어야 한다 (보수성)
    const int n = 2000;
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      int mx = 0;
      int my = 0;
      v.worldToMap(x0 + t * (x1 - x0), y0 + t * (y1 - y0), mx, my);
      ASSERT_TRUE(visited.count({mx, my})) << trial << " " << k;
    }
  }
}

TEST(Grid, SupercoverThroughExactCornerVisitsBothSides)
{
  OwnedGrid g(10, 10, 1.0);
  const CostGrid v = g.view();
  std::set<std::pair<int, int>> visited;
  traceSupercover(
    v, 0.5, 0.5, 2.5, 2.5, [&](int x, int y) {
      visited.emplace(x, y);
      return true;
    });
  EXPECT_TRUE(visited.count({1, 0}));
  EXPECT_TRUE(visited.count({0, 1}));
  EXPECT_TRUE(visited.count({2, 2}));
}
