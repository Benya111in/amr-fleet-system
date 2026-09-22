// 마커 기반 정밀 도킹 제어기 (ROS 비의존, docs/algorithms/docking.md).
//
// 입력: 마커 자세 관측 (base_link 기준 위치 + 마커 면 바깥 법선의 방위), 제어 주기마다 update().
// 출력: 속도 지령 (v, ω) 과 단계
//       (search → align → approach → final → 성공 / backup → 재시도 / 실패).
//
// 좌표 (도크 프레임 D): 원점 = 도킹 완료 시 base_link 위치 (마커 면에서 법선 방향으로 standoff),
// x 축 = 로봇이 마커를 향해 진행하는 방향. 로봇 자세 (x_r, y_r, ψ) 에서
//   e_x = -x_r (남은 종방향 거리, +면 아직 덜 감), e_y = y_r (횡오차), ψ (방위 오차).
// 판정: √(e_x² + e_y²) ≤ position_tolerance 이고 |ψ| ≤ angle_tolerance 가 settle_frames
// 연속이면 성공.
//
// 제어 법칙 두 가지 (params.law):
//   kProportional — 비례 시각 서보: v = k_d·e_x, 전방 주시점 방향 ψ_d = atan2(-e_y, L) 로
//                   ω = k_h·(ψ_d − ψ)
//   kGraceful     — Park & Kuipers (2011) 부드러운 제어 법칙 (Nav2 graceful controller 와 같은 식)
// 관측 사이(검출 30 Hz, 제어 20 Hz)와 짧은 미검출 구간은 MarkerTracker 가 직전 지령으로
// 예측해 메운다.
#ifndef AMR_BEHAVIOR__DOCKING__DOCKING_CONTROLLER_HPP_
#define AMR_BEHAVIOR__DOCKING__DOCKING_CONTROLLER_HPP_

#include <limits>
#include <optional>
#include <string>

namespace amr_behavior
{
namespace docking
{

enum class Phase {kIdle, kSearch, kAlign, kApproach, kFinal, kBackup, kSucceeded, kFailed};

/// Dock.action feedback current_phase 문자열 (components.md: search / align / approach / final).
const char * phaseName(Phase phase);

enum class ControlLaw {kProportional, kGraceful};

/// "proportional" / "graceful" → ControlLaw. 모르는 값이면 nullopt.
std::optional<ControlLaw> parseControlLaw(const std::string & name);

/// 파라미터 (behavior.yaml docking_server_node 와 1:1). 단위: m, rad, s.
struct Params
{
  ControlLaw law{ControlLaw::kGraceful};
  double standoff{0.65};              ///< 도킹 완료 시 마커 면 ~ base_link 거리
  double position_tolerance{0.02};    ///< 명세 위치 오차 2 cm
  double angle_tolerance{0.01745};    ///< 명세 각도 오차 1°
  int settle_frames{10};              ///< 판정 유지 주기 수 (20 Hz × 10 = 0.5 s)
  double final_distance{0.15};        ///< e_x 가 이 이하면 final 단계 (저속)
  /// 종방향 도달 판정 [m]: |e_x| 가 이 안에 들어와야 전진을 멈추고 판정한다 (허용오차 경계가 아닌
  /// 목표점까지 들어가 추정 오차 여유를 둔다). 도달 해제는 stop_distance + 2·linear_deadband.
  double stop_distance{0.008};
  double align_threshold{0.35};       ///< 진행 방향 오차가 이보다 크면 제자리 정렬 (align)
  // 속도 상한
  double max_linear_speed{0.15};      ///< approach 상한 [m/s]
  double final_linear_speed{0.05};    ///< final 상한 [m/s]
  double min_linear_speed{0.01};      ///< graceful 최소 전진 속도 [m/s] (정지 특이점 회피)
  double max_angular_speed{0.4};      ///< [rad/s]
  double search_angular_speed{0.25};  ///< 마커 탐색 회전 [rad/s]
  double search_sweep{0.5};           ///< 탐색 회전 진폭 ±[rad] (삼각파)
  // 비례 법칙 이득
  double k_distance{0.8};             ///< v = k_distance·e_x [1/s]
  double k_heading{1.5};              ///< ω = k_heading·(ψ_d − ψ) [1/s]
  double lookahead{0.05};             ///< 전방 주시 거리 L 의 하한 [m] (L = max(e_x, lookahead))
  // graceful 법칙 이득 (Park & Kuipers)
  double k_phi{2.0};
  double k_delta{1.0};
  double beta{0.4};
  double lambda{2.0};
  double slowdown_radius{0.25};       ///< 이 거리 안에서 v ∝ r [m]
  // 정지 대역: 이 안이면 해당 축 지령 0 (미세 떨림 방지)
  double linear_deadband{0.004};
  double angular_deadband{0.004};
  // 시간·재시도
  double marker_timeout{2.0};         ///< approach/final 중 관측이 이만큼 끊기면 시도 실패 [s]
  double search_timeout{8.0};         ///< search 단계 상한 [s]
  double attempt_timeout{45.0};       ///< 시도 1회 상한 [s]
  double backup_distance{0.3};        ///< 재시도 전 후진 거리 [m]
  double backup_speed{0.1};           ///< [m/s]
  double max_overshoot{0.05};         ///< e_x < −max_overshoot 이면 시도 실패 [m]
  /// 관측 저역통과 계수 (1 = 필터 없음, opennav filter_coef 와 같은 의미)
  double filter_coef{1.0};
  double control_period{0.05};        ///< 제어 주기 [s] (예측·신선도 판정용)
};

/// base_link 기준 마커 관측: 마커 중심 (x, y) 와 마커 면 바깥 법선(로봇 쪽)의 방위 normal_yaw.
struct MarkerObservation
{
  double x{0.0};
  double y{0.0};
  double normal_yaw{0.0};
};

/// 도크 프레임 오차. 값이 없으면 NaN.
struct DockErrors
{
  double longitudinal{std::numeric_limits<double>::quiet_NaN()};   ///< e_x [m]
  double lateral{std::numeric_limits<double>::quiet_NaN()};        ///< e_y [m]
  double heading{std::numeric_limits<double>::quiet_NaN()};        ///< ψ [rad]
  double position() const;   ///< √(e_x² + e_y²)
  bool valid() const;
};

struct Command
{
  double linear{0.0};    ///< [m/s]
  double angular{0.0};   ///< [rad/s]
};

/// 관측 → 도크 프레임 오차.
DockErrors computeErrors(const MarkerObservation & marker, double standoff);

/// 마커 자세 추적: 관측이 오면 (저역통과로) 갱신, 없으면 직전 속도 지령으로 예측한다.
/// (확장점: 브리프 §3.4 의 상대자세 EKF 로 교체 — 같은 predict/correct 인터페이스)
class MarkerTracker
{
public:
  void reset();
  void correct(const MarkerObservation & obs, double now, double filter_coef);
  /// 로봇이 (v, ω) 로 dt 동안 움직였을 때 로봇 기준 마커 자세를 옮긴다.
  void predict(double linear, double angular, double dt);
  bool hasEstimate() const {return has_;}
  const MarkerObservation & estimate() const {return est_;}
  /// 마지막 관측 이후 경과 [s] (관측이 없었으면 +inf).
  double age(double now) const;

private:
  bool has_{false};
  MarkerObservation est_;
  double last_obs_{-1.0};
};

class DockingController
{
public:
  explicit DockingController(const Params & params);

  const Params & params() const {return params_;}

  /// 새 goal 시작. max_attempts = 이 goal 의 최대 접근 횟수 (1 미만이면 1).
  void start(double now, int max_attempts);
  /// 제어 주기마다 호출. obs = 이번 주기에 새로 받은 관측 (없으면 nullopt).
  Command update(double now, const std::optional<MarkerObservation> & obs);
  /// 취소: 정지 지령만 내고 실패로 끝낸다.
  void cancel();

  Phase phase() const {return phase_;}
  int attempt() const {return attempt_;}
  int maxAttempts() const {return max_attempts_;}
  bool finished() const {return phase_ == Phase::kSucceeded || phase_ == Phase::kFailed;}
  bool succeeded() const {return phase_ == Phase::kSucceeded;}
  /// 마지막으로 계산한 오차 (추적 추정 기준).
  const DockErrors & errors() const {return errors_;}
  /// feedback distance_remaining: 위치 오차 (추정이 없으면 NaN).
  double distanceRemaining() const {return errors_.position();}
  /// 마지막 시도 실패 사유 (marker_lost / search_timeout / attempt_timeout / overshoot / lateral /
  /// canceled).
  const std::string & failureReason() const {return failure_reason_;}

  /// 제어 법칙 (단계 판단 없이 오차 → 지령). 테스트·문서용으로 공개.
  Command proportional(const DockErrors & e, double speed_cap) const;
  Command graceful(const DockErrors & e, double speed_cap) const;

private:
  Command control(const DockErrors & e, double speed_cap) const;
  Command finalAlign(const DockErrors & e) const;
  bool inTolerance(const DockErrors & e) const;
  void failAttempt(double now, const std::string & reason);
  void enterPhase(Phase phase, double now);
  Command stop();

  Params params_;
  MarkerTracker tracker_;
  Phase phase_{Phase::kIdle};
  int attempt_{0};
  int max_attempts_{1};
  double attempt_start_{0.0};
  double phase_start_{0.0};
  double last_update_{-1.0};
  int settle_count_{0};
  int lateral_stall_count_{0};
  bool arrived_{false};   ///< 종방향 도달 (stop_distance, 히스테리시스)
  Command last_cmd_;
  DockErrors errors_;
  std::string failure_reason_;
};

}  // namespace docking
}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__DOCKING__DOCKING_CONTROLLER_HPP_
