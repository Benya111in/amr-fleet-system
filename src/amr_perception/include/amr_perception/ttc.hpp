// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 충돌 예측과 TTC(Time-To-Collision) — ROS 비의존.
//
// 장애물: CV 예측 p_o(τ) = p + τ v, 위치 공분산 Σ(τ) = P_pp + τ(P_pv + P_vp) + τ² P_vv + qτ³/3 I.
// 로봇  : 현재 경로(plan)를 속도 max(|v_r|, v_floor) 로 따라가는 p_r(τ) (경로가 없거나 멀면 현재
//         방향 직선, 실제 속도).
// 충돌  : g(τ) = |p_r(τ) - p_o(τ)| - (r_robot + r_o + min(k·σ_o(τ), σ_cap)) <= 0,
//         σ_o(τ) = 연결 방향 d 로 본 표준편차 √(dᵀ Σ(τ) d).
// TTC   = g 가 처음 0 이하가 되는 τ (격자 step 으로 찾고 이분법으로 정밀화), 없으면 +inf.
// 유도는 docs/algorithms/tracking.md §5.

#ifndef AMR_PERCEPTION__TTC_HPP_
#define AMR_PERCEPTION__TTC_HPP_

#include <limits>
#include <vector>

#include "amr_perception/geometry2d.hpp"

namespace amr_perception
{

struct TtcParams
{
  double horizon{5.0};             ///< 예측 지평 T_h [s] (> τ_warn 3.0 s)
  double step{0.1};                ///< 격자 간격 [s]
  double k_sigma{1.0};             ///< 불확실성 팽창 배수 k
  double sigma_cap{0.5};           ///< 팽창 상한 [m]
  double robot_radius{0.361};      ///< 풋프린트 외접원 반경 √(0.3²+0.2²) [m]
  double min_robot_speed{0.2};     ///< 경로 추종 가정 최소 속도 v_floor [m/s]
  double max_path_deviation{1.0};  ///< 로봇이 경로에서 이보다 멀면 경로 무시 [m]
  int refine_iterations{12};       ///< 이분법 반복 (0.1 s / 2^12 ≈ 25 µs)
};

/// 로봇 미래 위치 모델
class RobotMotionModel
{
public:
  /// 정지 (속도 0)
  static RobotMotionModel stationary(const Vec2 & position);
  /// 현재 방향 직선 등속 (speed 부호 = 전/후진)
  static RobotMotionModel straightLine(const Vec2 & position, double yaw, double speed);
  /// 경로 추종: 로봇 위치를 경로에 사영한 점에서 speed 로 경로를 따라 전진 (끝점에서 정지).
  /// 경로가 2점 미만이거나 로봇이 경로에서 max_deviation 보다 멀면 false.
  static bool fromPath(
    const std::vector<Vec2> & path, const Vec2 & position, double speed, double max_deviation,
    RobotMotionModel & out);
  /// 경로가 유효하면 경로(속도 max(|v|, v_floor)), 아니면 직선(실제 속도)
  static RobotMotionModel select(
    const std::vector<Vec2> & path, const Vec2 & position, double yaw, double speed,
    const TtcParams & params);

  Vec2 positionAt(double tau) const;
  bool followsPath() const {return kind_ == Kind::Path;}
  double speed() const {return speed_;}
  /// 경로 위 사영점까지의 호 길이 [m] (경로 모드)
  double startArcLength() const {return s0_;}

private:
  enum class Kind { Line, Path };
  Kind kind_{Kind::Line};
  Vec2 origin_{Vec2::Zero()};
  Vec2 direction_{Vec2(1.0, 0.0)};
  double speed_{0.0};
  std::vector<Vec2> path_;
  std::vector<double> arc_;
  double s0_{0.0};
};

/// 장애물 등속 예측 (추적기 출력에서 만든다)
struct ObstacleMotion
{
  Vec2 position{Vec2::Zero()};
  Vec2 velocity{Vec2::Zero()};
  Mat2 P_pp{Mat2::Zero()};
  Mat2 P_pv{Mat2::Zero()};
  Mat2 P_vv{Mat2::Zero()};
  double q{0.0};        ///< CWNA q [m^2/s^3]
  double radius{0.1};   ///< 장애물 반경 r_o [m]

  Vec2 positionAt(double tau) const {return position + tau * velocity;}
  Mat2 covarianceAt(double tau) const;
};

struct TtcResult
{
  double ttc{std::numeric_limits<double>::infinity()};  ///< 충돌 시각 [s], 없으면 +inf
  /// 최근접 시각 [s] (격자 해상도; 충돌이 있으면 충돌 격자 직전까지만 탐색)
  double cpa_time{0.0};
  double cpa_distance{std::numeric_limits<double>::infinity()};  ///< 최근접 중심 거리 [m]
};

/// 충돌 여유 g(τ) (<= 0 이면 충돌)
double collisionClearance(
  const ObstacleMotion & obstacle, const RobotMotionModel & robot, const TtcParams & params,
  double tau);

TtcResult computeTimeToCollision(
  const ObstacleMotion & obstacle, const RobotMotionModel & robot, const TtcParams & params);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__TTC_HPP_
