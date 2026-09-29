// 로봇 발자국(직사각형)의 방위 의존 도달거리.
//
// 왜 필요한가
// -----------
// 접촉 판정(amr_simulation/collision_monitor_node.py, footprint.rect_circle)은 로봇 **직사각형**
// 0.60 × 0.40 과 장애물 원의 부호 거리로 한다. 그런데 DWA 는 **동적** 장애물에 대해서만 자기를
// 외접원(robot_radius 0.361)으로 본다 (dwa.cpp 의 in_body · VO · TTC · 통로 반폭). 정적 장애물에는
// 이미 발자국을 쓴다 (footprint_cost).
//
// 외접원은 회전 불변이라 **"몸을 돌리면 장애물 쪽 여유가 생긴다"가 비용 함수에 표현될 수 없다.**
// 그 결과 회전 후보와 정지 후보가 동적 장애물 관점에서 구별되지 않았고, 그것을 근거로 몸 원통 안
// 제자리 회전을 금지한 적이 있다 (6c584bd, "회전해도 위치가 변하지 않는다" — 로봇을 점으로 본 논리).
//
// 실측 근거 (접촉 132 건 전수, 침투 중앙 6.0 mm · p90 14.4 · 최대 18.8 mm):
//   도달거리는 0.200 m(정횡) ~ 0.3606 m(모서리) 사이에서 변한다 — 변동폭이 침투의 8~27 배다.
//   방위가 균등하다고 보면 **접촉의 84.8 %가 최적 방위였다면 일어나지 않았다.**
//   구제 불가능한 것은 장애물이 정횡 90° ± 24° 에 있을 때뿐이다 (그때 이미 반폭 = 최소).
//   Y8a 실측 1 건도 일치한다: 상대 방위 48.3°, 이득 67.7 mm > 침투 17.8 mm.
#ifndef AMR_NAVIGATION__CORE__FOOTPRINT_REACH_HPP_
#define AMR_NAVIGATION__CORE__FOOTPRINT_REACH_HPP_

#include <cmath>
#include <limits>

namespace amr_navigation
{
namespace core
{

/// 중심축 정렬 직사각형(반길이 hl · 반폭 hw)의 중심에서 상대 방위 th 로 본 경계까지 거리 [m].
/// th 는 로봇 전방 기준 (0 = 정면, ±π/2 = 정횡). hl·hw ≤ 0 이면 0 을 돌려준다 (호출부가 방어).
inline double footprintReach(double hl, double hw, double th)
{
  if (!(hl > 0.0) || !(hw > 0.0)) {
    return 0.0;
  }
  const double c = std::abs(std::cos(th));
  const double s = std::abs(std::sin(th));
  const double kInf = std::numeric_limits<double>::infinity();
  return std::min(c > 1e-12 ? hl / c : kInf, s > 1e-12 ? hw / s : kInf);
}

/// 로봇 자세 yaw 에서 세계 좌표 (ox, oy) 의 장애물을 볼 때의 발자국 도달거리 [m].
inline double footprintReachTo(
  double hl, double hw, double rx, double ry, double yaw, double ox, double oy)
{
  return footprintReach(hl, hw, std::atan2(oy - ry, ox - rx) - yaw);
}

/// 어떤 방위에서도 보장되는 최소·최대 도달거리 (= 반폭 · 외접원).
inline double footprintReachMin(double hl, double hw) {return std::min(hl, hw);}
inline double footprintReachMax(double hl, double hw) {return std::hypot(hl, hw);}

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__FOOTPRINT_REACH_HPP_
