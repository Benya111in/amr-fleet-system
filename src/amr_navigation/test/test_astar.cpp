// A* 단위 테스트: Dijkstra 대비 최적성, 휴리스틱 허용성·일관성, 도달 불가·허용 오차·경계 조건.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <random>
#include <utility>
#include <vector>

#include "amr_navigation/core/astar.hpp"
#include "amr_navigation/core/grid.hpp"

using amr_navigation::core::AStar;
using amr_navigation::core::AStarConfig;
using amr_navigation::core::AStarResult;
using amr_navigation::core::AStarStatus;
using amr_navigation::core::Cell;
using amr_navigation::core::CostGrid;
using amr_navigation::core::OwnedGrid;
using amr_navigation::core::kInscribedInflated;
using amr_navigation::core::kLethalObstacle;
using amr_navigation::core::kNoInformation;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
const double kSqrt2 = std::sqrt(2.0);

// 같은 그래프(도착 셀 비용 배율 × 이동 길이, 코너컷 금지)의 기준 Dijkstra.
// forward=true: start 에서의 거리, false: goal 까지의 거리(역방향).
std::vector<double> dijkstra(
  const CostGrid & g, const AStar & astar, const Cell & src, bool forward)
{
  const int w = g.width;
  const int h = g.height;
  std::vector<double> dist(static_cast<std::size_t>(w * h), kInf);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  auto idx = [w](int x, int y) {return y * w + x;};
  auto trav = [&](int x, int y) {
      return astar.multiplier(g.at(x, y)) < std::numeric_limits<float>::infinity();
    };
  dist[static_cast<std::size_t>(idx(src.x, src.y))] = 0.0;
  pq.emplace(0.0, idx(src.x, src.y));
  while (!pq.empty()) {
    auto [d, i] = pq.top();
    pq.pop();
    if (d > dist[static_cast<std::size_t>(i)]) {
      continue;
    }
    const int x = i % w;
    const int y = i / w;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h || !trav(nx, ny)) {
          continue;
        }
        if (dx != 0 && dy != 0 && (!trav(x + dx, y) || !trav(x, y + dy))) {
          continue;
        }
        if (!forward && !trav(x, y)) {
          continue;
        }
        const double len = (dx != 0 && dy != 0) ? kSqrt2 : 1.0;
        // 순방향: 간선 (x,y)->(nx,ny) 비용 = len·m(c(nx,ny))
        // 역방향: 간선 (nx,ny)->(x,y) 비용 = len·m(c(x,y))
        const double m = forward ? astar.multiplier(g.at(nx, ny)) : astar.multiplier(g.at(x, y));
        const double nd = d + len * m;
        if (nd < dist[static_cast<std::size_t>(idx(nx, ny))]) {
          dist[static_cast<std::size_t>(idx(nx, ny))] = nd;
          pq.emplace(nd, idx(nx, ny));
        }
      }
    }
  }
  return dist;
}

OwnedGrid randomGrid(std::mt19937 & rng, int w, int h, double p_obstacle)
{
  OwnedGrid g(w, h, 0.05);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::uniform_int_distribution<int> cost(0, 252);
  for (auto & c : g.cells) {
    const double r = u(rng);
    if (r < p_obstacle) {
      c = kLethalObstacle;
    } else if (r < p_obstacle + 0.05) {
      c = kInscribedInflated;
    } else {
      c = static_cast<uint8_t>(u(rng) < 0.5 ? 0 : cost(rng));
    }
  }
  return g;
}

Cell randomFreeCell(std::mt19937 & rng, const OwnedGrid & g, const AStar & a)
{
  std::uniform_int_distribution<int> ux(0, g.width - 1);
  std::uniform_int_distribution<int> uy(0, g.height - 1);
  for (int k = 0; k < 10000; ++k) {
    Cell c{ux(rng), uy(rng)};
    if (a.multiplier(g.at(c.x, c.y)) < std::numeric_limits<float>::infinity()) {
      return c;
    }
  }
  return {0, 0};
}
}  // namespace

TEST(AStar, OctileDistance)
{
  EXPECT_FLOAT_EQ(AStar::octile(3, 0), 3.0F);
  EXPECT_FLOAT_EQ(AStar::octile(0, -4), 4.0F);
  EXPECT_NEAR(AStar::octile(3, 3), 3.0 * kSqrt2, 1e-5);
  EXPECT_NEAR(AStar::octile(5, 2), 3.0 + 2.0 * kSqrt2, 1e-5);
}

TEST(AStar, MultiplierTable)
{
  AStarConfig cfg;
  cfg.cost_weight = 2.0;
  AStar a(cfg);
  EXPECT_FLOAT_EQ(a.multiplier(0), 1.0F);
  EXPECT_NEAR(a.multiplier(126), 2.0, 1e-6);
  EXPECT_NEAR(a.multiplier(252), 3.0, 1e-6);
  EXPECT_TRUE(std::isinf(a.multiplier(kInscribedInflated)));
  EXPECT_TRUE(std::isinf(a.multiplier(kLethalObstacle)));
  EXPECT_NEAR(a.multiplier(kNoInformation), 3.0, 1e-6);   // unknown_cost 252
  cfg.allow_unknown = false;
  a.setConfig(cfg);
  EXPECT_TRUE(std::isinf(a.multiplier(kNoInformation)));
}

// 무작위 격자에서 A* 경로 비용 = Dijkstra 최단 비용 (최적성)
TEST(AStar, OptimalVersusDijkstraOnRandomGrids)
{
  std::mt19937 rng(12345);
  int compared = 0;
  for (int trial = 0; trial < 300; ++trial) {
    AStarConfig cfg;
    cfg.cost_weight = (trial % 3 == 0) ? 0.0 : (trial % 3 == 1 ? 1.0 : 3.0);
    AStar astar(cfg);
    const int w = 8 + trial % 25;
    const int h = 8 + (trial * 7) % 23;
    OwnedGrid g = randomGrid(rng, w, h, 0.25);
    const Cell s = randomFreeCell(rng, g, astar);
    const Cell t = randomFreeCell(rng, g, astar);
    const auto dist = dijkstra(g.view(), astar, s, true);
    const double ref = dist[static_cast<std::size_t>(t.y * w + t.x)];
    const AStarResult res = astar.plan(g.view(), s, t);
    if (std::isinf(ref)) {
      EXPECT_EQ(res.status, AStarStatus::kNoPath) << trial;
      continue;
    }
    ASSERT_TRUE(res.ok()) << trial << " " << toString(res.status);
    EXPECT_NEAR(res.cost, ref, 1e-3 * std::max(1.0, ref)) << trial;
    // 경로 자체의 비용 재계산도 일치해야 한다 (연속성·통행성 확인)
    EXPECT_NEAR(astar.pathCost(g.view(), res.path), ref, 1e-3 * std::max(1.0, ref)) << trial;
    EXPECT_EQ(res.path.front(), s);
    EXPECT_EQ(res.path.back(), t);
    ++compared;
  }
  EXPECT_GT(compared, 150);
}

// 휴리스틱: h(u) ≤ h*(u) (허용성), h(u) ≤ w(u,v) + h(v) (일관성)
TEST(AStar, HeuristicAdmissibleAndConsistent)
{
  std::mt19937 rng(99);
  for (int trial = 0; trial < 40; ++trial) {
    AStarConfig cfg;
    cfg.cost_weight = 0.5 * (trial % 5);
    AStar astar(cfg);
    OwnedGrid g = randomGrid(rng, 25, 20, 0.2);
    const CostGrid v = g.view();
    const Cell goal = randomFreeCell(rng, g, astar);
    const auto hstar = dijkstra(v, astar, goal, false);   // 각 셀 → goal 최단 비용
    for (int y = 0; y < v.height; ++y) {
      for (int x = 0; x < v.width; ++x) {
        const double h = AStar::octile(x - goal.x, y - goal.y);   // min 배율 = 1
        const double hs = hstar[static_cast<std::size_t>(y * v.width + x)];
        if (!std::isinf(hs)) {
          EXPECT_LE(h, hs + 1e-4) << x << "," << y;
        }
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = x + dx;
            const int ny = y + dy;
            if ((dx == 0 && dy == 0) || !v.inBounds(nx, ny)) {
              continue;
            }
            const double m = astar.multiplier(v.at(nx, ny));
            if (std::isinf(m)) {
              continue;
            }
            const double wuv = ((dx != 0 && dy != 0) ? kSqrt2 : 1.0) * m;
            const double hv = AStar::octile(nx - goal.x, ny - goal.y);
            EXPECT_LE(h, wuv + hv + 1e-5);
          }
        }
      }
    }
  }
}

TEST(AStar, UnreachableGoalReportsNoPath)
{
  OwnedGrid g(30, 30, 0.05);
  // 목표 (20,20) 을 벽으로 완전히 둘러싼다
  for (int i = 17; i <= 23; ++i) {
    g.at(i, 17) = kLethalObstacle;
    g.at(i, 23) = kLethalObstacle;
    g.at(17, i) = kLethalObstacle;
    g.at(23, i) = kLethalObstacle;
  }
  AStar astar;
  AStarResult r = astar.plan(g.view(), {2, 2}, {20, 20});
  EXPECT_EQ(r.status, AStarStatus::kNoPath);
  EXPECT_FALSE(r.ok());
  EXPECT_GT(r.expansions, 0U);
  // 허용 오차 5셀 → 벽 바깥의 가장 가까운 셀로 대체 (20,24) 또는 대칭점: 원 목표까지 4셀
  r = astar.plan(g.view(), {2, 2}, {20, 20}, 5);
  ASSERT_TRUE(r.ok());
  const double d = std::hypot(r.reached_goal.x - 20, r.reached_goal.y - 20);
  EXPECT_NEAR(d, 4.0, 1e-9);
  EXPECT_EQ(r.path.back(), r.reached_goal);
}

TEST(AStar, BlockedGoalUsesTolerance)
{
  OwnedGrid g(20, 20, 0.05);
  g.at(10, 10) = kLethalObstacle;
  AStar astar;
  EXPECT_EQ(astar.plan(g.view(), {0, 0}, {10, 10}).status, AStarStatus::kGoalBlocked);
  const AStarResult r = astar.plan(g.view(), {0, 0}, {10, 10}, 2);
  ASSERT_TRUE(r.ok());
  EXPECT_NEAR(std::hypot(r.reached_goal.x - 10, r.reached_goal.y - 10), 1.0, 1e-9);
}

TEST(AStar, StartConditions)
{
  OwnedGrid g(20, 20, 0.05);
  AStar astar;
  EXPECT_EQ(astar.plan(g.view(), {-1, 0}, {5, 5}).status, AStarStatus::kStartOutOfBounds);
  EXPECT_EQ(astar.plan(g.view(), {0, 0}, {50, 5}).status, AStarStatus::kGoalOutOfBounds);
  g.at(3, 3) = kLethalObstacle;
  EXPECT_EQ(astar.plan(g.view(), {3, 3}, {10, 10}).status, AStarStatus::kStartBlocked);
  CostGrid invalid;
  EXPECT_EQ(astar.plan(invalid, {0, 0}, {1, 1}).status, AStarStatus::kInvalidInput);
}

TEST(AStar, EscapesWhenStartInsideInscribedZone)
{
  OwnedGrid g(30, 30, 0.05);
  // 시작점 주변 3셀 반경이 INSCRIBED (예: 장애물 옆에서 계획 요청)
  for (int y = 7; y <= 13; ++y) {
    for (int x = 7; x <= 13; ++x) {
      g.at(x, y) = kInscribedInflated;
    }
  }
  AStarConfig cfg;
  AStar astar(cfg);
  AStarResult r = astar.plan(g.view(), {10, 10}, {25, 25});
  ASSERT_TRUE(r.ok());
  // 탈출 후에는 다시 253 구역으로 들어가지 않는다
  bool left = false;
  for (const auto & c : r.path) {
    const bool ins = g.at(c.x, c.y) == kInscribedInflated;
    if (!ins) {
      left = true;
    }
    EXPECT_FALSE(left && ins);
  }
  cfg.allow_start_in_inscribed = false;
  astar.setConfig(cfg);
  EXPECT_EQ(astar.plan(g.view(), {10, 10}, {25, 25}).status, AStarStatus::kStartBlocked);
}

TEST(AStar, NoCornerCutting)
{
  // 대각선으로만 통과 가능한 틈: (5,5) 와 (6,6) 사이를 두 장애물이 막음
  OwnedGrid g(12, 12, 0.05);
  for (int i = 0; i < 12; ++i) {
    if (i != 5) {
      g.at(i, 5) = kLethalObstacle;   // y=5 가로벽, x=5 만 열림
    }
  }
  g.at(5, 5) = kLethalObstacle;
  g.at(6, 5) = 0;   // 열린 칸을 x=6 으로
  g.at(5, 6) = 0;
  AStarConfig cfg;
  cfg.allow_corner_cutting = false;
  AStar astar(cfg);
  AStarResult r = astar.plan(g.view(), {0, 0}, {0, 11});
  ASSERT_TRUE(r.ok());
  for (std::size_t i = 1; i < r.path.size(); ++i) {
    const int dx = r.path[i].x - r.path[i - 1].x;
    const int dy = r.path[i].y - r.path[i - 1].y;
    if (dx != 0 && dy != 0) {
      EXPECT_NE(g.at(r.path[i - 1].x + dx, r.path[i - 1].y), kLethalObstacle);
      EXPECT_NE(g.at(r.path[i - 1].x, r.path[i - 1].y + dy), kLethalObstacle);
    }
  }
}

TEST(AStar, UnknownCellsRespectAllowUnknown)
{
  OwnedGrid g(20, 10, 0.05);
  for (int y = 0; y < 10; ++y) {
    g.at(10, y) = kNoInformation;   // 미지 벽
  }
  AStarConfig cfg;
  cfg.allow_unknown = true;
  AStar astar(cfg);
  EXPECT_TRUE(astar.plan(g.view(), {2, 5}, {17, 5}).ok());
  cfg.allow_unknown = false;
  astar.setConfig(cfg);
  EXPECT_EQ(astar.plan(g.view(), {2, 5}, {17, 5}).status, AStarStatus::kNoPath);
}

TEST(AStar, DeadlineAndIterationLimit)
{
  OwnedGrid g(400, 400, 0.05);
  AStar astar;
  const auto past = AStar::Clock::now() - std::chrono::seconds(1);
  // 첫 확장에서 시각을 확인하므로 이미 지난 마감은 즉시 타임아웃
  EXPECT_EQ(astar.plan(g.view(), {0, 0}, {399, 399}, 0, past).status, AStarStatus::kTimeout);
  AStarConfig cfg;
  cfg.max_iterations = 50;
  astar.setConfig(cfg);
  EXPECT_EQ(astar.plan(g.view(), {0, 0}, {399, 399}).status, AStarStatus::kIterationLimit);
}

TEST(AStar, DeterministicAndReusable)
{
  std::mt19937 rng(5);
  OwnedGrid g = randomGrid(rng, 60, 40, 0.15);
  AStar astar;
  const Cell s = randomFreeCell(rng, g, astar);
  const Cell t = randomFreeCell(rng, g, astar);
  const AStarResult a = astar.plan(g.view(), s, t);
  const AStarResult b = astar.plan(g.view(), s, t);   // 세대 번호 재사용 경로
  ASSERT_EQ(a.status, b.status);
  ASSERT_EQ(a.path.size(), b.path.size());
  for (std::size_t i = 0; i < a.path.size(); ++i) {
    EXPECT_EQ(a.path[i], b.path[i]);
  }
}

// 1200×800 (60×40 m @ 0.05) 대각 최장 경로: 무장애물 κ=0 이면 비용 = 옥타일 거리, 시간 출력
TEST(AStar, LargeOpenGridCornerToCorner)
{
  OwnedGrid g(1200, 800, 0.05);
  AStarConfig cfg;
  cfg.cost_weight = 0.0;
  AStar astar(cfg);
  const auto t0 = std::chrono::steady_clock::now();
  const AStarResult r = astar.plan(g.view(), {0, 0}, {1199, 799});
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  ASSERT_TRUE(r.ok());
  // float 누적 오차: 1530 셀에 상대 1e-4 이내
  EXPECT_NEAR(r.cost, AStar::octile(1199, 799), 1e-4 * AStar::octile(1199, 799));
  // 동점 처리 (f,h) 덕분에 무장애물에서는 경로 위 셀만 확장한다
  EXPECT_LT(r.expansions, 5000U);
  std::printf(
    "[ info ] 1200x800 open corner-to-corner: %.2f ms, %zu expansions\n", ms,
    r.expansions);
}
