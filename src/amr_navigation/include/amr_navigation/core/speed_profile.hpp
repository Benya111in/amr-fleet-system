// 경로 위 속도 상한과 주행 시간 예측 — ROS 비의존.
//
// 곡률 상한 (정점 i, 곡률 κ_i = 이산 회전각/호길이):
//   v_cap,i = min(v_des, ω_max/|κ_i|, √(a_lat/|κ_i|)),  마지막 정점(목표) v_cap = 0.
// 앞보기 허용 속도 (사다리꼴 감속 원뿔): v_allow(s) = min_{s_k
//   ≥ s} √(v_cap,k² + 2·a_dec·(s_k − s)).
//   Pure Pursuit 속도 조절과 주행 시간 예측이 같은 식을 써야
//   예측 오차가 작다(명세 4.4 "예측 시간 오차 15 %").
// 주행 시간 예측:
//   전진/후진 패스 v_i = min(v_cap,i, √(v_{i−1}² + 2aΔs), √(v_{i+1}² + 2dΔs)), v_0 = v_start, v_M =
//   0, T = τ_rot(|Δθ_0|) + Σ 2Δs_i/(v_i + v_{i+1}) + a/j (S-curve 저크 보정, 정지-출발 1쌍) +
//   τ_rot(|Δθ_M|), τ_rot(θ) = θ/ω + ω/α (θ ≥ ω²/α), 2√(θ/α) (그 밖) — 사다리꼴 제자리 회전.
// 목표 정지 접근 (제어기용 allowedSpeed 의 목표 정점만): 사다리꼴 √(2·a·d) 대신 저크·지연을 넣은
//   정지거리 d_stop(v) = v·t_c + [감속도 0 → −a 저크 램프] + 등감속 의 역함수 v = d_stop⁻¹(d).
//   하류 저크 필터(velocity_profiler_node)와 서보 지연 때문에 사다리꼴 명령은 목표를 지나친다
//   (폐루프 실측: path_tracking.md §5).
#ifndef AMR_NAVIGATION__CORE__SPEED_PROFILE_HPP_
#define AMR_NAVIGATION__CORE__SPEED_PROFILE_HPP_

#include <cstddef>
#include <vector>

#include "amr_navigation/core/geometry.hpp"

namespace amr_navigation
{
namespace core
{

struct SpeedProfileConfig
{
  double desired_speed{1.0};       // [m/s] 순항 속도
  double max_accel{1.0};           // [m/s²]
  double max_decel{1.0};           // [m/s²]
  double max_angular_vel{1.5};     // [rad/s]
  double max_angular_accel{2.0};   // [rad/s²] 제자리 회전 시간 예측
  double max_lateral_accel{0.8};   // [m/s²] 곡률 감속 (적재물 안정)
  double max_jerk{2.0};            // [m/s³] S-curve 시간 보정 (≤ 0 이면 보정 없음)
  double min_speed{0.05};          // [m/s] 목표 전 최저 속도 (0 에 수렴해 멈추지 않도록)
  double stop_latency{0.0};        // [s] 목표 정지 접근의 명령 → 실제 속도 지연 t_c
  double curvature_window{0.25};   // [m] 곡률 추정 창
};

class SpeedProfile
{
public:
  SpeedProfile() = default;

  /// 경로와 설정으로 곡률·속도 상한을 계산한다. end_is_goal 이면 끝점 상한 = 0.
  void build(
    const std::vector<Pose2D> & path, const SpeedProfileConfig & config,
    bool end_is_goal = true);

  bool empty() const {return s_.empty();}
  std::size_t size() const {return s_.size();}
  const std::vector<double> & arcLength() const {return s_;}
  const std::vector<double> & curvature() const {return kappa_;}
  const std::vector<double> & caps() const {return cap_;}
  double totalLength() const {return s_.empty() ? 0.0 : s_.back();}

  /// 곡률 κ 에서의 속도 상한.
  static double curvatureCap(double kappa, const SpeedProfileConfig & config);

  /// 호길이 s 에서 앞보기 허용 속도. horizon [m] 까지만 본다 (≤ 0 이면 끝까지).
  double allowedSpeed(double s, double horizon = 0.0) const;

  /// 전체 속도 프로파일 v_i (시작 속도 v0).
  std::vector<double> velocityProfile(double v0 = 0.0) const;

  /// 주행 시간 예측 [s]. start_heading_error / goal_heading_error: 출발·도착 제자리 회전 각 [rad].
  double predictTravelTime(
    double v0 = 0.0, double start_heading_error = 0.0, double goal_heading_error = 0.0) const;

  /// 사다리꼴 제자리 회전 시간.
  static double rotationTime(double angle, double w_max, double alpha_max);

  /// 저크 제한 정지거리 [m]: t_c 동안 v 유지 → 감속도 0 에서 저크 j 로 −a 까지 램프 → 등감속 정지.
  /// j ≤ 0 이면 램프 없이 사다리꼴, a ≤ 0 이면 무한대.
  static double stoppingDistance(double v, double a, double j, double t_c);
  /// stoppingDistance(v) ≤ d 인 최대 속도 (단조 증가 함수의 이분법 역함수, d ≤ 0 → 0).
  static double maxSpeedForStop(double d, double a, double j, double t_c);

private:
  SpeedProfileConfig config_;
  bool end_is_goal_{true};
  std::vector<double> s_;
  std::vector<double> kappa_;
  std::vector<double> cap_;
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__SPEED_PROFILE_HPP_
