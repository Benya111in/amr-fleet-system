// Pure Pursuit 경로 추종 — ROS 비의존 코어 (명세 4.5 "Pure
//   Pursuit 직접 구현, 속도 적응 look-ahead").
//
// 기하 (docs/algorithms/path_tracking.md 유도):
//   로봇 좌표계의 look-ahead 점 G = (x_g, y_g), ‖G‖ = L 을 지나는
//   원호의 곡률  κ = 2·y_g / L² = 2·sin α / L,
//   각속도 ω = v·κ. 직선 근방 선형화 → ë + (2v/L)ė + (2v²/L²)e = 0 (ζ = 1/√2, ω_n = √2·v/L).
// 속도 적응 look-ahead: L = clamp(k·|v|, L_min, L_max) — L ∝
//   v 이면 ω_n = √2/k 로 속도와 무관 (게인 스케줄).
// 속도 조절: v = min(v_des, 앞보기 곡률·목표 감속 상한(SpeedProfile),
//   √(a_lat/|κ|), ω_max/|κ|, 외부 제한)
//            × max(0, cos α) (look-ahead 방향 오차가 크면 감속).
// 모드: 경로 방향 오차가 크고 거의 정지 → 제자리 회전(rotate-to-path),
//       목표 근처(goal_align_distance) → 목표 방향 정렬(rotate-to-goal),
//       ω = sign·min(ω_max, √(2α|Δθ|)).
// 확장 지점: use_chord_correction — 기준 현(chord) 성분 y_g⁰ 를 빼고 경로 곡률을 피드포워드하는
//   CC-PP (연구 브리프 path-tracking §4.1). 직선·원에서는 기본
//   PP 와 같고 곡률이 바뀌는 구간에서만 다르다.
#ifndef AMR_NAVIGATION__CORE__PURE_PURSUIT_HPP_
#define AMR_NAVIGATION__CORE__PURE_PURSUIT_HPP_

#include <cstddef>
#include <limits>
#include <vector>

#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/speed_profile.hpp"

namespace amr_navigation
{
namespace core
{

struct PurePursuitConfig
{
  double desired_linear_vel{1.0};      // [m/s]
  double lookahead_time{0.8};          // [s] k: L = k·|v|
  double min_lookahead{0.4};           // [m]
  double max_lookahead{1.8};           // [m]
  double max_angular_vel{1.5};         // [rad/s]
  double max_angular_accel{2.0};       // [rad/s²]
  double max_linear_accel{1.0};        // [m/s²]
  double max_linear_decel{1.0};        // [m/s²]
  double max_lateral_accel{0.8};       // [m/s²]
  double max_linear_jerk{2.0};         // [m/s³] (주행 시간 예측)
  double min_approach_vel{0.05};       // [m/s]
  double approach_latency{0.3};        // [s] 목표 정지 접근의 명령 지연 (= stop_latency)
  double curvature_window{0.25};       // [m]
  bool use_rotate_to_heading{true};
  double rotate_to_heading_min_angle{0.785};   // [rad]
  double rotate_to_heading_max_v{0.1};         // [m/s] 이 속도 이하에서만 제자리 회전
  double goal_align_distance{0.08};    // [m] 목표 방향 정렬을 시작하는 거리
  double goal_yaw_tolerance{0.02};     // [rad] 정렬 완료 판정
  bool use_chord_correction{false};    // CC-PP 확장
  double chord_gain{1.0};              // K
};

enum class PurePursuitMode
{
  kTrack,
  kRotateToPath,
  kRotateToGoal,
  kGoalReached,
  kNoPath
};

struct PurePursuitOutput
{
  double v{0.0};
  double w{0.0};
  PurePursuitMode mode{PurePursuitMode::kNoPath};
  double lookahead{0.0};
  double curvature{0.0};
  Point2D carrot;               // 경로 프레임 look-ahead 점
  double carrot_angle{0.0};     // α [rad]
  double cte{0.0};              // [m] 좌측 +
  double d_goal{0.0};           // [m]
  double s{0.0};                // [m] 투영 호길이
};

class PurePursuit
{
public:
  explicit PurePursuit(const PurePursuitConfig & config = PurePursuitConfig());

  void setConfig(const PurePursuitConfig & config);
  const PurePursuitConfig & config() const {return config_;}

  /// 경로 설정 (경로와 자세는 같은 프레임). 속도 상한 프로파일을 다시 계산한다.
  void setPath(const std::vector<Pose2D> & path);
  const std::vector<Pose2D> & path() const {return path_;}
  const SpeedProfile & profile() const {return profile_;}

  /// 한 주기 명령. v_now: 현재(직전 명령) 선속도 — look-ahead 계산에 쓴다.
  PurePursuitOutput compute(
    const Pose2D & pose, double v_now,
    double speed_limit = std::numeric_limits<double>::infinity());

  /// L = clamp(k·|v|, L_min, L_max).
  static double lookaheadDistance(double v, const PurePursuitConfig & config);
  /// 로봇 좌표 점 (x, y) 로 가는 원호 곡률 2y/(x²+y²).
  static double curvatureToPoint(double x, double y);
  /// 경로 위 look-ahead 점: 투영 선분부터 시작해 로봇 중심 거리 L 인 첫 교점(선분-원). 없으면 끝점.
  static Point2D findLookahead(
    const std::vector<Pose2D> & path, const Point2D & center, double L, std::size_t start_segment);

  /// 제자리 회전 각속도: sign(θ)·min(ω_max, √(2α|θ|)) (정지 시점에 각도 0 에 닿는 감속 곡선).
  static double rotationCommand(double angle, double w_max, double alpha_max);

private:
  PurePursuitConfig config_;
  std::vector<Pose2D> path_;
  std::vector<double> cum_s_;
  SpeedProfile profile_;
  std::size_t hint_{0};
  bool fresh_{true};
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__PURE_PURSUIT_HPP_
