// 다각형 풋프린트 충돌 검사 — ROS 비의존.
//
// 3단계 (docs/algorithms/dwa.md):
//   1) 중심 셀 비용 c0 ≥ 253 (INSCRIBED/LETHAL) → 충돌 (내접원 안에 장애물)
//   2) c0 < c_circ (외접 반경에서의 inflation 비용) → 자유:
//   외접원 안에 치명 셀이 없으므로 외곽선도 안전
//   3) 그 사이 → 외곽선 4변을 Bresenham 으로 래스터화해
//   치명(254) 셀이 있으면 충돌, 없으면 최대 비용
// Nav2 FootprintCollisionChecker 와 같은 원리(외곽선 검사)이며,
//   2)의 조기 판정으로 대부분의 자세를 O(1)에 처리한다.
#ifndef AMR_NAVIGATION__CORE__FOOTPRINT_HPP_
#define AMR_NAVIGATION__CORE__FOOTPRINT_HPP_

#include <cstdint>
#include <vector>

#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"

namespace amr_navigation
{
namespace core
{

class FootprintChecker
{
public:
  /// footprint: 로봇 좌표계 다각형 꼭짓점 [m]. circumscribed_cost: 외접 반경의 inflation 비용
  /// (모르면 0 → 항상 외곽선 검사).
  FootprintChecker(
    const CostGrid & grid, std::vector<Point2D> footprint, uint8_t circumscribed_cost,
    bool allow_unknown = false);

  /// 자세의 풋프린트 비용 (0~253 범위의 최대 비용) 또는 충돌 시 −1.
  double cost(const Pose2D & pose) const;
  bool collides(const Pose2D & pose) const {return cost(pose) < 0.0;}
  uint8_t centerCost(double x, double y) const;

  const CostGrid & grid() const {return grid_;}
  const std::vector<Point2D> & footprint() const {return footprint_;}

  static double inscribedRadius(const std::vector<Point2D> & footprint);
  static double circumscribedRadius(const std::vector<Point2D> & footprint);
  /// 길이 × 폭 사각형 풋프린트 (base_link 중심).
  static std::vector<Point2D> rectangle(double length, double width);

private:
  CostGrid grid_;
  std::vector<Point2D> footprint_;
  uint8_t circumscribed_cost_;
  bool allow_unknown_;
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__FOOTPRINT_HPP_
