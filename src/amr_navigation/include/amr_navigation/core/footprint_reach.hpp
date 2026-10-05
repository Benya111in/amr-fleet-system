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
// 제자리 회전을 금지한 적이 있다
// (6c584bd, "회전해도 위치가 변하지 않는다" — 로봇을 점으로 본 논리).
//
// 실측 근거 — **2026-09-29 정정**
// ------------------------------------
// 예전 주석은 "접촉 132 건의 침투가 중앙 6.0 mm 이고 도달거리 변동폭이 0.200~0.3606 이므로
// 최적 방위였다면 84.8 % 가 회피됐다" 고 적었다. **이 논증은 무효다.**
//   collision_monitor_node.py:127 은 부호거리가 음수가 되는 **첫 표본**을 기록하고 래치하는데
//   지면 진실은 50 Hz (dynamic_obstacles.yaml:21) 다. 따라서 기록 가능한 침투의 상한은
//   |v_rel| x 0.020 이고, 접촉 147 건 중 이 상한을 넘는 표본이 **0 건**이다
//   (docs/research/dynamic-avoidance-root-cause/checks/c1_census.py).
//   즉 침투는 "얼마나 모자랐는가" 가 아니라 **검열된(censored) 관측량**이다. 6 mm 로 기록된
//   접촉이 6 mm 만 더 있으면 됐다는 뜻이 아니다 — 첫 교차에서 측정을 멈출 뿐 실제 침투는 자란다.
//
// 이 헤더가 여전히 필요한 이유는 침투 크기가 아니라 **표현력**이다: 외접원은 SO(2) 회전에
// 불변인데, 차동구동 로봇이 v→0 에서 온전히 보유한 유일한 제어 입력이 바로 그 회전이다.
// 즉 "몸을 돌려 좁은 쪽을 향한다" 가 동적 비용에 **표현될 수 없다** (연구 브리프 §3.2).
// 기대 효과는 다시 유도해야 한다 — 옛 84.8 % 수치를 근거로 쓰지 마라.

#ifndef AMR_NAVIGATION__CORE__FOOTPRINT_REACH_HPP_
#define AMR_NAVIGATION__CORE__FOOTPRINT_REACH_HPP_

#include <algorithm>
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
