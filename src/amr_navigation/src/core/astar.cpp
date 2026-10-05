#include "amr_navigation/core/astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

namespace amr_navigation
{
namespace core
{
namespace
{
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kSqrt2 = 1.41421356237F;
constexpr double kFQuant = 1024.0;   // f 양자화 [1/셀]

// 8-이웃: (dx, dy, 길이)
struct Move
{
  int dx;
  int dy;
  float len;
};
constexpr std::array<Move, 8> kMoves{{
  {1, 0, 1.0F}, {-1, 0, 1.0F}, {0, 1, 1.0F}, {0, -1, 1.0F},
  {1, 1, kSqrt2}, {1, -1, kSqrt2}, {-1, 1, kSqrt2}, {-1, -1, kSqrt2}}};

// 힙 비교: (f, h) 사전순 최소 힙 — std::push_heap 은 최대 힙이므로 "크다" 를 반대로 정의
struct HeapGreater
{
  template<typename T>
  bool operator()(const T & a, const T & b) const
  {
    return a.fq > b.fq || (a.fq == b.fq && a.h > b.h);
  }
};
}  // namespace

std::string toString(AStarStatus status)
{
  switch (status) {
    case AStarStatus::kSuccess: return "success";
    case AStarStatus::kInvalidInput: return "invalid input";
    case AStarStatus::kStartOutOfBounds: return "start out of bounds";
    case AStarStatus::kGoalOutOfBounds: return "goal out of bounds";
    case AStarStatus::kStartBlocked: return "start blocked";
    case AStarStatus::kGoalBlocked: return "goal blocked";
    case AStarStatus::kNoPath: return "no path";
    case AStarStatus::kIterationLimit: return "iteration limit";
    case AStarStatus::kTimeout: return "timeout";
  }
  return "unknown";
}

AStar::AStar(const AStarConfig & config)
: config_(config)
{
  rebuildTable();
}

void AStar::setConfig(const AStarConfig & config)
{
  config_ = config;
  rebuildTable();
}

void AStar::rebuildTable()
{
  const double kappa = std::max(0.0, config_.cost_weight);
  for (int c = 0; c < 256; ++c) {
    float m = kInf;
    if (c == kNoInformation) {
      if (config_.allow_unknown) {
        m = static_cast<float>(1.0 + kappa * config_.unknown_cost / 252.0);
      }
    } else if (c < config_.obstacle_threshold && c < kLethalObstacle) {
      m = static_cast<float>(1.0 + kappa * c / 252.0);
    }
    table_[static_cast<std::size_t>(c)] = m;
  }
  min_multiplier_ = kInf;
  for (float m : table_) {
    min_multiplier_ = std::min(min_multiplier_, m);
  }
  if (!(min_multiplier_ < kInf)) {
    min_multiplier_ = 1.0F;
  }
}

void AStar::setTraversalTable(const std::array<float, 256> & table)
{
  table_ = table;
  min_multiplier_ = kInf;
  for (float m : table_) {
    if (m > 0.0F) {
      min_multiplier_ = std::min(min_multiplier_, m);
    }
  }
  if (!(min_multiplier_ < kInf)) {
    min_multiplier_ = 1.0F;
  }
}

bool AStar::traversable(uint8_t c) const
{
  return table_[c] < kInf;
}

float AStar::octile(int dx, int dy)
{
  const int ax = std::abs(dx);
  const int ay = std::abs(dy);
  const int mx = std::max(ax, ay);
  const int mn = std::min(ax, ay);
  return static_cast<float>(mx) + (kSqrt2 - 1.0F) * static_cast<float>(mn);
}

uint32_t AStar::quantize(float f)
{
  const double q = std::round(static_cast<double>(f) * kFQuant);
  constexpr double kMax = static_cast<double>(std::numeric_limits<uint32_t>::max());
  return q >= kMax ? std::numeric_limits<uint32_t>::max() : static_cast<uint32_t>(q);
}

float AStar::heuristic(int x, int y, int gx, int gy) const
{
  return octile(x - gx, y - gy) * min_multiplier_;
}

void AStar::heapPush(const HeapNode & n)
{
  heap_.push_back(n);
  std::push_heap(heap_.begin(), heap_.end(), HeapGreater());
}

AStar::HeapNode AStar::heapPop()
{
  std::pop_heap(heap_.begin(), heap_.end(), HeapGreater());
  const HeapNode n = heap_.back();
  heap_.pop_back();
  return n;
}

std::vector<Cell> AStar::backtrack(int width, uint32_t goal_idx) const
{
  std::vector<Cell> out;
  int32_t idx = static_cast<int32_t>(goal_idx);
  while (idx >= 0) {
    out.push_back({idx % width, idx / width});
    const int32_t p = parent_[static_cast<std::size_t>(idx)];
    if (p == idx) {
      break;
    }
    idx = p;
  }
  std::reverse(out.begin(), out.end());
  return out;
}

bool AStar::nearestTraversable(
  const CostGrid & grid, const Cell & goal, int radius, Cell & out) const
{
  double best = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const double d = std::hypot(dx, dy);
      if (d > radius || d >= best) {
        continue;
      }
      const int x = goal.x + dx;
      const int y = goal.y + dy;
      if (!grid.inBounds(x, y) || !traversable(grid.at(x, y))) {
        continue;
      }
      best = d;
      out = {x, y};
      found = true;
    }
  }
  return found;
}

AStarResult AStar::plan(
  const CostGrid & grid, const Cell & start, const Cell & goal_in, int goal_tolerance_cells,
  Clock::time_point deadline)
{
  AStarResult res;
  res.reached_goal = goal_in;
  if (!grid.valid()) {
    res.status = AStarStatus::kInvalidInput;
    return res;
  }
  if (!grid.inBounds(start.x, start.y)) {
    res.status = AStarStatus::kStartOutOfBounds;
    return res;
  }
  if (!grid.inBounds(goal_in.x, goal_in.y)) {
    res.status = AStarStatus::kGoalOutOfBounds;
    return res;
  }
  const uint8_t start_cost = grid.at(start.x, start.y);
  // "탈출 구역": 막혔지만 치명/미지가 아닌 셀 (기본 253). 시작
  //   셀이 여기 있으면 이 구역 안에서만 통행 허용.
  auto escape_cell = [this](uint8_t c) {
      return !traversable(c) && c != kLethalObstacle && c != kNoInformation;
    };
  const bool escaping = config_.allow_start_in_inscribed && escape_cell(start_cost);
  if (!traversable(start_cost) && !escaping) {
    res.status = AStarStatus::kStartBlocked;
    return res;
  }
  Cell goal = goal_in;
  if (!traversable(grid.at(goal.x, goal.y))) {
    if (goal_tolerance_cells <= 0 ||
      !nearestTraversable(grid, goal_in, goal_tolerance_cells, goal))
    {
      res.status = AStarStatus::kGoalBlocked;
      return res;
    }
  }

  const std::size_t n = static_cast<std::size_t>(grid.width) *
    static_cast<std::size_t>(grid.height);
  if (g_.size() != n) {
    g_.assign(n, kInf);
    parent_.assign(n, -1);
    stamp_.assign(n, 0U);
    gen_ = 0;
  }
  // 세대 증가 (오버플로 시 전체 초기화)
  if (gen_ >= (std::numeric_limits<uint32_t>::max() / 2U) - 2U) {
    std::fill(stamp_.begin(), stamp_.end(), 0U);
    gen_ = 0;
  }
  ++gen_;
  const uint32_t open_mark = 2U * gen_;
  const uint32_t closed_mark = 2U * gen_ + 1U;
  heap_.clear();

  const int w = grid.width;
  const int h = grid.height;
  const uint8_t * data = grid.data;
  const std::size_t max_iter = config_.max_iterations > 0 ? config_.max_iterations : n;
  const float escape_mult = table_[kMaxNonObstacle] < kInf ? table_[kMaxNonObstacle] :
    static_cast<float>(1.0 + std::max(0.0, config_.cost_weight));

  const uint32_t s_idx = static_cast<uint32_t>(grid.index(start.x, start.y));
  const uint32_t goal_idx = static_cast<uint32_t>(grid.index(goal.x, goal.y));
  g_[s_idx] = 0.0F;
  parent_[s_idx] = static_cast<int32_t>(s_idx);
  stamp_[s_idx] = open_mark;
  {
    const float h0 = heuristic(start.x, start.y, goal.x, goal.y);
    heapPush({quantize(h0), h0, s_idx});
  }

  bool found = false;
  std::size_t expansions = 0;
  while (!heap_.empty()) {
    const HeapNode cur = heapPop();
    const uint32_t ci = cur.idx;
    if (stamp_[ci] == closed_mark) {
      continue;   // lazy deletion: 이미 확정된 셀의 오래된 항목
    }
    stamp_[ci] = closed_mark;
    ++expansions;
    if (ci == goal_idx) {
      found = true;
      break;
    }
    if (expansions >= max_iter) {
      res.status = AStarStatus::kIterationLimit;
      res.expansions = expansions;
      return res;
    }
    // 첫 확장과 이후 1024 회마다 시각 확인 (이미 지난 마감이면 즉시 kTimeout)
    if ((expansions & 1023U) == 1U && Clock::now() > deadline) {
      res.status = AStarStatus::kTimeout;
      res.expansions = expansions;
      return res;
    }
    const int cx = static_cast<int>(ci % static_cast<uint32_t>(w));
    const int cy = static_cast<int>(ci / static_cast<uint32_t>(w));
    const uint8_t c_cur = data[ci];
    const bool cur_escaping = escaping && escape_cell(c_cur);
    const float g_cur = g_[ci];
    for (const Move & mv : kMoves) {
      const int nx = cx + mv.dx;
      const int ny = cy + mv.dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
        continue;
      }
      const uint32_t ni = static_cast<uint32_t>(ny * w + nx);
      if (stamp_[ni] == closed_mark) {
        continue;
      }
      const uint8_t c = data[ni];
      float m = table_[c];
      if (!(m < kInf)) {
        if (cur_escaping && escape_cell(c)) {
          m = escape_mult;
        } else {
          continue;
        }
      }
      if (mv.dx != 0 && mv.dy != 0 && !config_.allow_corner_cutting) {
        const uint8_t ca = data[static_cast<std::size_t>(cy) * w + nx];
        const uint8_t cb = data[static_cast<std::size_t>(ny) * w + cx];
        const bool a_ok = traversable(ca) || (cur_escaping && escape_cell(ca));
        const bool b_ok = traversable(cb) || (cur_escaping && escape_cell(cb));
        if (!a_ok || !b_ok) {
          continue;
        }
      }
      const float ng = g_cur + mv.len * m;
      if (stamp_[ni] != open_mark || ng < g_[ni]) {
        g_[ni] = ng;
        parent_[ni] = static_cast<int32_t>(ci);
        stamp_[ni] = open_mark;
        const float hn = heuristic(nx, ny, goal.x, goal.y);
        heapPush({quantize(ng + hn), hn, ni});
      }
    }
  }
  res.expansions = expansions;

  uint32_t end_idx = goal_idx;
  if (!found) {
    if (goal_tolerance_cells <= 0) {
      res.status = AStarStatus::kNoPath;
      return res;
    }
    // 도달 가능 영역을 모두 확장했으므로, 허용 반경 안의 닫힌
    //   셀 중 원래 목표에 가장 가까운 셀로 대체
    double best = std::numeric_limits<double>::infinity();
    bool any = false;
    for (int dy = -goal_tolerance_cells; dy <= goal_tolerance_cells; ++dy) {
      for (int dx = -goal_tolerance_cells; dx <= goal_tolerance_cells; ++dx) {
        const double d = std::hypot(dx, dy);
        const int x = goal_in.x + dx;
        const int y = goal_in.y + dy;
        if (d > goal_tolerance_cells || d >= best || !grid.inBounds(x, y)) {
          continue;
        }
        const uint32_t idx = static_cast<uint32_t>(grid.index(x, y));
        if (stamp_[idx] == closed_mark) {
          best = d;
          end_idx = idx;
          any = true;
        }
      }
    }
    if (!any) {
      res.status = AStarStatus::kNoPath;
      return res;
    }
  }
  res.path = backtrack(w, end_idx);
  res.cost = static_cast<double>(g_[end_idx]);
  res.reached_goal = {static_cast<int>(end_idx % static_cast<uint32_t>(w)),
    static_cast<int>(end_idx / static_cast<uint32_t>(w))};
  res.status = AStarStatus::kSuccess;
  return res;
}

double AStar::pathCost(const CostGrid & grid, const std::vector<Cell> & path) const
{
  double cost = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const int dx = path[i].x - path[i - 1].x;
    const int dy = path[i].y - path[i - 1].y;
    if (std::abs(dx) > 1 || std::abs(dy) > 1 || (dx == 0 && dy == 0) ||
      !grid.inBounds(path[i].x, path[i].y))
    {
      return std::numeric_limits<double>::infinity();
    }
    const float m = table_[grid.at(path[i].x, path[i].y)];
    if (!(m < kInf)) {
      return std::numeric_limits<double>::infinity();
    }
    const double len = (dx != 0 && dy != 0) ? static_cast<double>(kSqrt2) : 1.0;
    cost += len * static_cast<double>(m);
  }
  return cost;
}

}  // namespace core
}  // namespace amr_navigation
