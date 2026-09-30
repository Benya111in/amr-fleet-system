// 예측 점유를 코스트맵에 찍는 **순수 로직** (ROS 비의존).
//
// 저장소 규약대로 판단 로직을 core 로 빼고 ros/predicted_obstacle_layer.cpp 는 구독·발행만
// 하는 얇은 래퍼로 둔다. 그래야 단위 계약이 가능하다 — 커버리지 측정이 이 파일을 1.0 %
// (1/96) 로 지목했고, 그것은 통합 측정에만 기대고 있었다는 뜻이었다.
#ifndef AMR_NAVIGATION__CORE__PREDICTED_OCCUPANCY_HPP_
#define AMR_NAVIGATION__CORE__PREDICTED_OCCUPANCY_HPP_

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace amr_navigation
{
namespace core
{

/// 코스트맵 한 칸의 좌표와 값 (ROS 형에 의존하지 않기 위한 최소 표현)
struct CellCost
{
  int i;
  int j;
  unsigned char cost;
};

/// 점 (x, y) 둘레 반경 r 를 덮는 셀 목록. 원점 (ox, oy), 해상도 res, 격자 w x h.
/// 격자 밖은 내지 않는다.
inline std::vector<CellCost> discCells(
  double x, double y, double r, unsigned char cost,
  double ox, double oy, double res, int w, int h)
{
  std::vector<CellCost> out;
  if (!(res > 0.0) || !(r >= 0.0)) {
    return out;
  }
  // floor 여야 한다. static_cast<int> 는 0 쪽으로 절삭하므로 원점보다 작은 좌표
  // (x < ox) 에서 한 칸 어긋난다 — 예: (x-ox)/res = -2.3 -> -2 인데 올바른 셀은 -3 이다.
  // 추출 전 코드는 Costmap2D::worldToMap 을 썼고 그것은 floor 한다.
  const int ci = static_cast<int>(std::floor((x - ox) / res));
  const int cj = static_cast<int>(std::floor((y - oy) / res));
  const int rc = static_cast<int>(std::ceil(r / res));
  out.reserve(static_cast<std::size_t>((2 * rc + 1) * (2 * rc + 1)));
  for (int dj = -rc; dj <= rc; ++dj) {
    for (int di = -rc; di <= rc; ++di) {
      if (di * di + dj * dj > rc * rc) {
        continue;
      }
      const int i = ci + di, j = cj + dj;
      if (i < 0 || i >= w || j < 0 || j >= h) {
        continue;
      }
      out.push_back({i, j, cost});
    }
  }
  return out;
}

/// 기피는 **올리기만** 한다 — 정적 장애물·팽창 같은 더 비싼 값을 덮으면 안 된다.
/// no_information 은 값이 없다는 뜻이므로 덮는다.
inline bool shouldRaise(unsigned char old_cost, unsigned char new_cost, unsigned char no_info)
{
  return old_cost == no_info || old_cost < new_cost;
}

/// 예측이 아직 쓸 만한가. timeout <= 0 이면 만료를 보지 않는다.
/// 미래 스탬프(음수 나이)도 같은 폭으로 막는다 — 시계가 튀면 옛 예측이 되살아난다.
inline bool predictionFresh(double age_s, double timeout_s)
{
  return timeout_s <= 0.0 || (age_s <= timeout_s && age_s >= -timeout_s);
}

/// 점들을 덮는 사각 범위 (반경 포함). 비었으면 false.
inline bool pointsBounds(
  const std::vector<std::pair<double, double>> & pts, double r,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (pts.empty()) {
    return false;
  }
  *min_x = *min_y = 1e300;
  *max_x = *max_y = -1e300;
  for (const auto & p : pts) {
    *min_x = std::min(*min_x, p.first - r);
    *min_y = std::min(*min_y, p.second - r);
    *max_x = std::max(*max_x, p.first + r);
    *max_y = std::max(*max_y, p.second + r);
  }
  return true;
}

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__PREDICTED_OCCUPANCY_HPP_
