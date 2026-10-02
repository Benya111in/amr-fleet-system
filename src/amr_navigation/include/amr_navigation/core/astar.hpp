// 코스트맵 위 8-연결 비용 인지 A* — ROS 비의존 코어 (명세 4.4 "A* 직접 구현").
//
// 그래프: 셀 = 정점, 8-이웃 = 간선. 간선 가중치 w(u,v) = ℓ(u,v)·m(c(v)),
//   ℓ = 1 (직교) / √2 (대각)  [셀 단위],  m(c) = 1 + κ·c/252  (κ = cost_weight, Smac2D 의
//   cost_travel_multiplier 와 동형).
// 휴리스틱: 옥타일 거리 h = max(|dx|,|dy|) + (√2−1)·min(|dx|,|dy|) 에 min_c m(c) 를 곱한 값.
//   m ≥ m_min 이므로 허용적(admissible), 삼각부등식으로 일관적(consistent)
//   → 각 셀은 최대 1회 확장된다.
// 자료구조: 이진 힙(open, lazy deletion) + 세대 번호(stamp) 로
//   닫힌 집합/방문 표시 → 계획마다 O(N) 초기화가 없다.
//   메모리 = g(float) + parent(int32) + stamp(uint32) = 12 B/셀 (1200×800 → 11.5 MB).
// 동점 처리: (f_q, h) 사전순, f_q = round(f·1024) — f 를 1/1024 셀로 양자화해 float 누적
//   오차가 수학적 동점을 깨지 않게 하고, 같은 f_q 면 목표에 가까운(h 작은) 셀을 먼저 확장해
//   평원에서의 확장 수를 줄인다. 양자화로 인한 준최적 한계는 1/1024 셀(0.05 mm @ 0.05 m) 이하.
// 확장 지점: setTraversalTable() 로 m(c) 표를 통째로 바꿀 수 있다 (예: 여유거리→속도 맵의
//   시간 비용, docs/algorithms/astar.md "확장" 절). 표 값이 +inf 인 비용은 통행 불가.
#ifndef AMR_NAVIGATION__CORE__ASTAR_HPP_
#define AMR_NAVIGATION__CORE__ASTAR_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "amr_navigation/core/grid.hpp"

namespace amr_navigation
{
namespace core
{

struct AStarConfig
{
  /// κ: 셀 비용 가중. 0 이면 순수 최단거리, 클수록 장애물에서 멀어진다(경로는 길어짐).
  double cost_weight{2.0};
  /// 미지 셀(255) 통행 허용 여부. 허용 시 가중치는 unknown_cost 로 계산한다.
  bool allow_unknown{true};
  uint8_t unknown_cost{kMaxNonObstacle};
  /// 이 값 이상인 셀은 통행 불가 (기본 253 = INSCRIBED: 로봇
  ///   중심이 내접원 안에 장애물을 두게 되는 셀).
  uint8_t obstacle_threshold{kInscribedInflated};
  /// 대각 이동 시 두 직교 이웃 중 하나라도 막혀 있으면 금지 (모서리 스침 방지).
  bool allow_corner_cutting{false};
  /// 시작 셀이 INSCRIBED(253) 구역 안이면 그 구역에서 빠져나오는 동안만 253 셀 통행을 허용한다.
  bool allow_start_in_inscribed{true};
  /// 확장 수 상한 (0 = 셀 수).
  std::size_t max_iterations{0};
};

enum class AStarStatus
{
  kSuccess,
  kInvalidInput,
  kStartOutOfBounds,
  kGoalOutOfBounds,
  kStartBlocked,
  kGoalBlocked,
  kNoPath,
  kIterationLimit,
  kTimeout
};

std::string toString(AStarStatus status);

struct AStarResult
{
  AStarStatus status{AStarStatus::kNoPath};
  std::vector<Cell> path;       // 시작 → 도달 셀 (성공 시)
  double cost{0.0};             // 경로 비용 [셀 단위 가중 길이]
  std::size_t expansions{0};    // 닫힌 셀 수
  Cell reached_goal;            // 실제 도달 셀 (허용 오차로 바뀔 수 있음)
  bool ok() const {return status == AStarStatus::kSuccess;}
};

class AStar
{
public:
  using Clock = std::chrono::steady_clock;

  explicit AStar(const AStarConfig & config = AStarConfig());

  void setConfig(const AStarConfig & config);
  const AStarConfig & config() const {return config_;}

  /// 통행 배율 표 m(c) 를 직접 지정한다 (확장 지점). +inf = 통행 불가. 휴리스틱 배율은 표의 최솟값.
  void setTraversalTable(const std::array<float, 256> & table);
  const std::array<float, 256> & traversalTable() const {return table_;}

  /// start → goal 계획. goal_tolerance_cells > 0 이면 목표가 막혔거나 도달 불가일 때 반경 안의
  /// 도달 가능한 셀 중 목표에 가장 가까운 셀로 대신한다. deadline 을 넘기면 kTimeout.
  AStarResult plan(
    const CostGrid & grid, const Cell & start, const Cell & goal, int goal_tolerance_cells = 0,
    Clock::time_point deadline = Clock::time_point::max());

  /// 옥타일 거리 [셀].
  static float octile(int dx, int dy);

  /// 주어진 셀 경로의 비용(같은 가중치). 연속하지 않거나 막힌 셀이 있으면 +inf.
  double pathCost(const CostGrid & grid, const std::vector<Cell> & path) const;

  /// 셀 비용 c 의 통행 배율.
  float multiplier(uint8_t c) const {return table_[c];}

private:
  struct HeapNode
  {
    uint32_t fq;    // round(f·kFQuant)
    float h;
    uint32_t idx;
  };
  static uint32_t quantize(float f);

  void rebuildTable();
  bool traversable(uint8_t c) const;
  float heuristic(int x, int y, int gx, int gy) const;
  void heapPush(const HeapNode & n);
  HeapNode heapPop();
  std::vector<Cell> backtrack(int width, uint32_t goal_idx) const;
  bool nearestTraversable(
    const CostGrid & grid, const Cell & goal, int radius, Cell & out) const;

  AStarConfig config_;
  std::array<float, 256> table_{};
  float min_multiplier_{1.0F};

  // 작업 버퍼 (계획 간 재사용; 격자 크기가 바뀔 때만 재할당)
  std::vector<float> g_;
  std::vector<int32_t> parent_;
  std::vector<uint32_t> stamp_;   // 2·gen = open/방문, 2·gen+1 = closed
  uint32_t gen_{0};
  std::vector<HeapNode> heap_;
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__ASTAR_HPP_
