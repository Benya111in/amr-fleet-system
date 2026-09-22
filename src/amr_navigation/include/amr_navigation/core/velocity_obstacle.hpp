// 동적 장애물 예측과 Velocity Obstacle(VO) — ROS 비의존 (명세
//   4.7 "Velocity Obstacle 또는 ORCA 개념").
//
// 장애물 모델: 등속(CV) 원판. 추적기(perception/tracked_obstacles)의 위치·속도로 o(t) = o + u·t.
//
// VO 원뿔 판정 (Fiorini & Shiller 1998, 시간 절단형):
//   상대 위치 p = o − r, 합성 반경 R = r_robot + r_obs + margin, 상대 속도 w = v_robot − u_obs.
//   VO^τ = { w : ∃ t∈[0,τ], ‖p − t·w‖ < R }  — 꼭짓점이 원점, 축이 p, 반각 asin(R/‖p‖) 인 원뿔을
//   시간 τ 에서 원판 D(p/τ, R/τ) 로 자른 집합. 판정은 최근접 시각 t* = clamp(p·w/‖w‖², 0, τ),
//   d_min = ‖p − t*·w‖ < R 과 같다(원뿔 각도식과 동치, docs/algorithms/dwa.md 유도).
//   ‖p‖ < R(이미 여유 안)이면 접근(w·p > 0)하는 속도만 VO 로 본다.
// 차동구동 샘플 (v, ω) 의 등가 직선 속도: τ 동안의 현(chord) 속도
//   v_eff = v·sinc(ωτ/2)·(cos(θ+ωτ/2), sin(θ+ωτ/2)) (|ω|→0 이면 v·(cosθ, sinθ)).
//   현 근사는 원호 충돌을 일부 놓칠 수 있으므로(연구 브리프 c03: 24.6 %), 원호 롤아웃 기반의
//   예측 충돌 시각(TTC)을 비용 항으로 함께 쓴다 → firstContactTime().
#ifndef AMR_NAVIGATION__CORE__VELOCITY_OBSTACLE_HPP_
#define AMR_NAVIGATION__CORE__VELOCITY_OBSTACLE_HPP_

#include <limits>
#include <vector>

#include "amr_navigation/core/geometry.hpp"

namespace amr_navigation
{
namespace core
{

struct DynamicObstacle
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double radius{0.25};

  Point2D predict(double t) const {return {x + vx * t, y + vy * t};}
  double speed() const;
};

/// 차동구동 샘플 (v, ω) 의 τ 구간 현(chord) 속도 [m/s].
Point2D chordVelocity(double theta, double v, double w, double tau);

/// 절단 VO 원뿔 판정: 상대 위치 p, 상대 속도 w_rel, 합성 반경 R, 시간 지평 τ.
bool inVelocityObstacle(const Point2D & p, const Point2D & w_rel, double R, double tau);

/// 로봇 궤적(시각 t_k 의 자세들, 등간격 dt)과 장애물 예측 위치
///   사이 거리가 처음으로 R 미만이 되는 시각 [s].
/// 구간 사이는 선형 보간해 상대 운동의 2차식 근으로 정확한 진입 시각을 구한다. 없으면 +inf.
double firstContactTime(
  const std::vector<Pose2D> & traj, double dt, const DynamicObstacle & obs, double R);

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__VELOCITY_OBSTACLE_HPP_
