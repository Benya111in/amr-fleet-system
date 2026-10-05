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

/// 이 셀에 **치명(LETHAL)** 을 찍어도 되는가 — 로봇 자기 자리에는 찍으면 안 된다.
///
/// 왜 필요한가 (실측). 예측 점유를 cost 254 로 찍는 구성에서만 A* 가
/// "start blocked (0 expansions)" 로 실패했다 — 168 실행 중 그 구성 4 개에서만 나오고
/// 나머지 164 개에서 0 건이다. 그 뒤 BT 가 follow_path 를 취소해 **제어 주기가 통째로
/// 비고, 접촉이 그 공백 안에서 난다** (logs/NW1a 3 건, logs/MX1a 1 건. 공백 1.6~6.4 s).
/// 연구 브리프 §11.
///
/// 기제: 치명 집합은 예측 중심 둘레 dynamic_radius(0.55) 링을 radius(0.30) 로 채운
/// 고리다. 그런데 예측 중심의 시각은 추적기 TTC 이고, ttc.cpp 가 로봇 속도를 0.2 로
/// 바닥 치므로 정면 조우에서 "만나는 지점" 이 곧 로봇이 서 있는 자리가 된다. 그러면
/// astar.cpp 의 start 탈출 예외가 kLethalObstacle 을 제외하므로 start 가 막힌다.
///
/// 처방: 로봇 외접원 + 여유 안의 셀에는 치명을 찍지 않는다. **기피(253 미만)는 그대로
/// 찍는다** — 계획기가 그 자리를 피하도록 유도하는 것은 유효하고, 막는 것만이 문제다.
/// 그 자리의 실제 안전은 안전 게이트(STOP 래치)가 맡는다. 불변식 A(게이트 거부권 단방향)
/// 와 충돌하지 않는다 — 이것은 게이트를 **약화하지 않고** 계획기를 되살릴 뿐이다.
inline bool mayStampLethal(
  double cx, double cy, double robot_x, double robot_y, double keepout_r)
{
  if (!(keepout_r > 0.0)) {
    return true;
  }
  const double dx = cx - robot_x, dy = cy - robot_y;
  return dx * dx + dy * dy > keepout_r * keepout_r;
}

/// 치명 금지 반경 안이면 한 단계 낮춘 값(기피)으로, 밖이면 원래 값 그대로.
/// inscribed(253) 미만이어야 계획기가 통과할 수 있다.
inline unsigned char stampCost(
  unsigned char want, double cx, double cy,
  double robot_x, double robot_y, double keepout_r,
  unsigned char lethal_floor, unsigned char demoted)
{
  if (want < lethal_floor) {
    return want;                       // 애초에 기피값이면 그대로
  }
  return mayStampLethal(cx, cy, robot_x, robot_y, keepout_r) ? want : demoted;
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
