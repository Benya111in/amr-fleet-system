// 격자(코스트맵) 공통 자료형 — ROS 비의존.
//
// Nav2 Costmap2D 와 같은 메모리 배치(행 우선, index = mx
//   + my * width, 셀 값 0~255)를 그대로 가리키는
// 비소유 뷰(CostGrid)와, 테스트·벤치마크용 소유 격자(OwnedGrid),
//   Nav2 InflationLayer 와 같은 식의 팽창 함수를 둔다.
#ifndef AMR_NAVIGATION__CORE__GRID_HPP_
#define AMR_NAVIGATION__CORE__GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace amr_navigation
{
namespace core
{

// Nav2 nav2_costmap_2d/cost_values.hpp 와 같은 값 (ROS 의존을 피하려고 복제)
constexpr uint8_t kFreeSpace = 0;
constexpr uint8_t kMaxNonObstacle = 252;
constexpr uint8_t kInscribedInflated = 253;
constexpr uint8_t kLethalObstacle = 254;
constexpr uint8_t kNoInformation = 255;

/// 정수 격자 좌표.
struct Cell
{
  int x{0};
  int y{0};
  bool operator==(const Cell & o) const {return x == o.x && y == o.y;}
  bool operator!=(const Cell & o) const {return !(*this == o);}
};

/// 코스트맵 비소유 뷰. data 는 width*height 바이트, 행 우선.
struct CostGrid
{
  const uint8_t * data{nullptr};
  int width{0};
  int height{0};
  double resolution{0.05};   // [m/셀]
  double origin_x{0.0};      // [m] 셀 (0,0) 의 좌하단 모서리 (Nav2 규약)
  double origin_y{0.0};

  bool valid() const {return data != nullptr && width > 0 && height > 0 && resolution > 0.0;}
  bool inBounds(int mx, int my) const {return mx >= 0 && my >= 0 && mx < width && my < height;}
  std::size_t index(int mx, int my) const
  {
    return static_cast<std::size_t>(my) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(mx);
  }
  uint8_t at(int mx, int my) const {return data[index(mx, my)];}
  /// 범위 밖은 치명(LETHAL)으로 본다 — 지도 밖으로 나가는 궤적/경로를 막는다.
  uint8_t atOrLethal(int mx, int my) const
  {
    return inBounds(mx, my) ? at(mx, my) : kLethalObstacle;
  }
  /// 월드 [m] → 셀. 범위 밖이면 false (mx, my 는 내림 값으로 채움).
  bool worldToMap(double wx, double wy, int & mx, int & my) const;
  /// 셀 중심의 월드 좌표.
  void mapToWorld(int mx, int my, double & wx, double & wy) const;
  /// 연속 좌표의 셀 비용 (범위 밖 = LETHAL).
  uint8_t costAtWorld(double wx, double wy) const;
};

/// 소유 격자 (테스트·벤치마크·합성 지도용).
struct OwnedGrid
{
  std::vector<uint8_t> cells;
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};

  OwnedGrid() = default;
  OwnedGrid(int w, int h, double res, uint8_t fill = kFreeSpace, double ox = 0.0, double oy = 0.0);
  CostGrid view() const;
  uint8_t & at(int mx, int my)
  {
    return cells[static_cast<std::size_t>(my) * static_cast<std::size_t>(width) +
             static_cast<std::size_t>(mx)];
  }
  uint8_t at(int mx, int my) const
  {
    return cells[static_cast<std::size_t>(my) * static_cast<std::size_t>(width) +
             static_cast<std::size_t>(mx)];
  }
  /// 월드 좌표 축정렬 사각형 [x0,x1]×[y0,y1] 을 value 로 채운다 (셀 중심 포함 기준).
  void fillRect(double x0, double y0, double x1, double y1, uint8_t value);
};

/// Nav2 InflationLayer::computeCost 와 같은 식. distance_m = 셀 중심 간 거리 [m].
uint8_t inflationCost(
  double distance_m, double inscribed_radius, double cost_scaling_factor);

/// 치명 셀(=254)에서 inflation_radius 까지 Nav2 식으로 비용을
///   팽창한다 (정확한 유클리드 거리, 브루트포스
/// 커널). 원래 값보다 큰 경우에만 덮어쓴다. 테스트·벤치마크용
///   — 실행 시에는 Nav2 InflationLayer 가 한다.
void inflate(
  OwnedGrid & grid, double inscribed_radius, double inflation_radius, double cost_scaling_factor);

/// 두 셀 사이 Bresenham 선분의 각 셀에 대해 visit(x, y) 를 부른다.
///   visit 가 false 를 돌려주면 중단하고 false.
bool traceLine(int x0, int y0, int x1, int y1, const std::function<bool(int, int)> & visit);

/// 연속 좌표 선분이 지나는 모든 셀(supercover: 모서리를 스치는
///   셀 포함)을 방문한다. 보수적 가시선 검사용.
bool traceSupercover(
  const CostGrid & grid, double x0, double y0, double x1, double y1,
  const std::function<bool(int, int)> & visit);

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__GRID_HPP_
