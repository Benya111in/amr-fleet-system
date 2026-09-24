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
  // 통로 예측(횡단 양보)에 쓰는 평활 속도. VO·TTC 는 원시 (vx, vy) 를 그대로 쓴다 — 즉각
  // 회피를 늦추면 안 되기 때문이다. 추적기가 내보내는 진행각은 직선 보행자에 대해서도
  // 주기간 90 % 12.6° · 최대 144.9° 흔들려(08 실측), 그대로 쓰면 예측 통로가 그만큼 회전해
  // 경로 위 교차 구간이 미터 단위로 이동한다. 설정하지 않으면 원시 속도를 쓴다.
  double vx_pred{0.0};
  double vy_pred{0.0};
  bool has_pred{false};

  Point2D predict(double t) const {return {x + vx * t, y + vy * t};}
  double speed() const;
  /// 통로 예측용 속도 (평활값이 없으면 원시 속도)
  double predVx() const {return has_pred ? vx_pred : vx;}
  double predVy() const {return has_pred ? vy_pred : vy;}
};

/// 차동구동 샘플 (v, ω) 의 τ 구간 현(chord) 속도 [m/s].
Point2D chordVelocity(double theta, double v, double w, double tau);

/// 절단 VO 원뿔 판정: 상대 위치 p, 상대 속도 w_rel, 합성 반경 R, 시간 지평 τ.
bool inVelocityObstacle(const Point2D & p, const Point2D & w_rel, double R, double tau);

/// VO 진입 시각: 상대 등속 운동 p − t·w_rel 이 처음 반경 R 안에 드는 t ∈ [0, τ] (없으면 +inf).
/// 이미 R 안이면 접근 중일 때 0, 멀어지는 중이면 +inf. inVelocityObstacle ⇔ 이 값이 유한.
/// VO 가 동적 창의 모든 샘플을 덮을 때(포화) 가장 늦게 진입하는 샘플을 고르는 기준 (dwa.md §2.1).
double velocityObstacleTime(const Point2D & p, const Point2D & w_rel, double R, double tau);

/// 로봇 궤적(시각 t_k 의 자세들, 등간격 dt)과 장애물 예측 위치
///   사이 거리가 처음으로 R 미만이 되는 시각 [s].
/// 구간 사이는 선형 보간해 상대 운동의 2차식 근으로 정확한 진입 시각을 구한다. 없으면 +inf.
double firstContactTime(
  const std::vector<Pose2D> & traj, double dt, const DynamicObstacle & obs, double R);

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__VELOCITY_OBSTACLE_HPP_
