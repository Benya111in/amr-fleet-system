// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 안전 게이트 순수 로직 (ROS 비의존) — safety_node 가 50 Hz + 이벤트마다 호출한다.
//
// 입력: 속도 명령(cmd_vel_smoothed), 측정 속도(wheel_odom), base_link 평면 LiDAR 빔(빔 순서),
//       전방 깊이 점군(지면 높이로 자른 base_link 평면 점), 추적기 최소 TTC, E-stop 입력,
//       센서 생존 신호, (선택) 도킹 예외 다각형.
// 출력: 제한된 속도 명령, 존(CLEAR/WARNING/CRITICAL/STOP), estop_active, 사유 목록.
//
// 규칙 (docs/algorithms/tracking.md §6):
//   접근 거리 s(p) = 풋프린트(0.60×0.40)가 현재 운동(명령·측정 속도의 등속 원호, 회전 포함)을
//     따라 움직일 때 점 p 에 닿기까지 접촉점이 이동하는 거리. 원호는 정지 포락선
//     d_stop + d_brake(u) 까지, 그 뒤는 끝 자세의 접선 직선으로
//     max(경고 거리, 포락선) + margin 까지.
//     영역 밖 점(측면 벽, 지나온 점)은 접근이 아니다.
//   D = 공간 일관 최소 접근 거리: LiDAR 는 물리 폭 4 cm(최소 5 빔) 창에서 60 % 순위 값의 최소,
//     깊이 점군은 cloud_min_points 번째로 작은 값. 점은 촬영 후 측정 운동만큼 옮겨 판정한다
//     (지연 보정). 필수 깊이 점군이 없거나 촬영 후 cloud_max_age 를 넘으면 전진을
//     degraded_mode_max_speed 로 묶는다.
//   D <= 0.30 → STOP (safety/zone=3, 래치). 확정 = 2 프레임 연속 또는 0.25 m 이하 한 프레임.
//     해제 = 현재 운동의 D > stop_release_distance — 운동이 바뀌어 멀어지면 바로 해제(갇힘 없음).
//   D <= 0.50 → CRITICAL |u| <= 0.2 / D <= 1.00 → WARNING |u| <= 0.5 (u = 풋프린트 최고 점 속도)
//   여유 거리 연속 제한 u <= -a·t + √((a·t)² + 2a(D - 0.30))
//   접촉 가드: 모든 방향, 풋프린트까지 거리의 창 중앙값 최소 <= contact_guard_distance → STOP.
//     그 점들에서 멀어지는(거리가 줄지 않는) 명령만 escape_max_speed 로 통과.
//   도킹 예외 다각형 안 점: 정지 거리 exclusion_stop_distance, CRITICAL 상한 (계약 C2).
//   TTC <= τ_crit(2.15 s) → v <= a·(TTC - t_react)
//   센서: 간격 > late_timeout → 지연 경고(진단만), > fault_timeout → 고장
//     (stop → 정지 + estop_active, degraded → degraded_mode_max_speed).
//   E-stop 입력(estop, /fleet/estop): true 수신 즉시 래치. 해제는 모든 입력이 명시적 false 인
//     상태에서 reset 요청이 올 때만 (거절된 reset 은 아무것도 바꾸지 않는다).
//   estop_active = E-stop 래치 || 정지형 센서 고장 (근접 정지는 E-stop 이 아니다 — 계약 C1).

#ifndef AMR_PERCEPTION__SAFETY_GATE_HPP_
#define AMR_PERCEPTION__SAFETY_GATE_HPP_

#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "amr_perception/geometry2d.hpp"

namespace amr_perception
{

enum class SafetyZone : uint8_t
{
  kClear = 0,
  kWarning = 1,
  kCritical = 2,
  kStop = 3,
};

enum class SensorFailureAction
{
  kStop,
  kDegraded,
};

struct SensorWatch
{
  std::string name;
  double late_timeout{0.3};   ///< 수신 간격이 이보다 길면 지연 경고 (진단만) [s]
  double fault_timeout{0.3};  ///< 수신 간격이 이보다 길면 고장 → action [s]
  SensorFailureAction action{SensorFailureAction::kStop};
};

struct SafetyParams
{
  double footprint_length{0.60};
  double footprint_width{0.40};
  double emergency_stop_distance{0.30};
  double critical_zone_distance{0.50};
  double warning_zone_distance{1.00};
  double warning_zone_max_speed{0.5};
  double critical_zone_max_speed{0.2};
  double stop_release_distance{0.50};   ///< 같은 운동에서 STOP 래치 해제 거리 (히스테리시스) [m]
  double zone_hysteresis{0.05};         ///< WARNING/CRITICAL 완화 방향 히스테리시스 [m]
  double reaction_latency{0.15};
  double max_deceleration{1.0};         ///< limits.max_linear_acceleration
  bool clearance_speed_limit_enabled{true};
  double max_linear_velocity{2.0};
  double min_linear_velocity{-0.5};
  double max_angular_velocity{1.5};
  bool ttc_limit_enabled{true};
  double ttc_critical{2.15};            ///< τ_crit [s]
  double ttc_max_age{0.5};              ///< 이보다 오래된 TTC 는 무시 [s]
  double degraded_mode_max_speed{0.2};
  double command_timeout{0.5};          ///< 입력 명령이 이보다 오래되면 0 [s]
  bool estop_release_requires_reset{true};
  // 접근 영역
  double swept_margin{0.0};             ///< 스윕 풋프린트 좌우 팽창 [m] (스침은 접촉 가드가 맡는다)
  double region_margin{0.10};           ///< 영역 길이 여유 [m]
  double motion_epsilon_v{0.01};        ///< 이보다 느리면 선운동 없음 [m/s]
  double motion_epsilon_w{0.05};        ///< 이보다 느리면 회전 없음 [rad/s]
  bool hold_motion_intent{true};        ///< 정지 중에는 마지막 명령 운동 방향으로 영역을 유지
  double measured_velocity_timeout{0.2};  ///< 측정 속도 유효 시간 [s]
  double measured_velocity_time_constant{0.1};  ///< 측정 속도 1차 저역 통과 시정수 [s]
  double self_filter_margin{0.02};      ///< 풋프린트 안쪽(축소) 점 = 자기 차체, 판정 제외 [m]
  // 잡음 강건성 (LiDAR σ 0.03 m): 빔 창 = 물리 폭 window_width 를 덮는 빔 수 (최소 min_window),
  // 창 값 중 support 비율 순위 → 창의 support 비율 이상이 가까워야 한다 (단일·소수 빔 잡음 무시)
  double beam_window_width{0.04};       ///< 접근 거리 창 물리 폭 [m] (3 cm 물체까지 검출)
  int beam_window{5};                   ///< 접근 거리 창 최소 빔 수
  double beam_support{0.6};             ///< 접근 거리 창 순위 비율 (5 빔 창 = 세 번째로 작은 값)
  double guard_window_width{0.06};      ///< 접촉 가드 창 물리 폭 [m]
  int guard_window{9};                  ///< 접촉 가드 창 최소 빔 수 (순위 비율 0.5 = 중앙값)
  double contact_guard_distance{0.02};  ///< 모든 방향 접촉 가드 [m]
  int stop_confirm_frames{2};           ///< STOP 확정 연속 프레임 수 (프레임 = 새 스캔/점군)
  double immediate_stop_margin{0.05};   ///< 정지 거리보다 이만큼 더 가까우면 한 프레임으로 확정 [m]
  int cloud_min_points{3};              ///< 깊이 점군 접근 거리 = 이 순위 값
  double cloud_timeout{0.3};            ///< 수신 후 이보다 오래된 점군은 무시 [s]
  /// 깊이 점군을 필수 입력으로 본다 (safety_node: depth_cloud.enabled). 촬영 후 cloud_max_age 를
  /// 넘었거나 수신이 끊기면 LiDAR 평면 아래를 볼 수 없으므로 전진을
  /// degraded_mode_max_speed 로 묶는다
  bool cloud_required{false};
  double cloud_max_age{0.4};            ///< 촬영 후 이 시간을 넘은 점군 = 없음 [s]
  bool latency_compensation{true};      ///< 촬영 시각 이후 측정 운동만큼 점을 옮겨 판정
  double max_compensation_age{0.5};     ///< 보정에 쓰는 최대 지연 [s]
  // 탈출 (접촉 가드)
  bool allow_escape{true};
  double escape_horizon{0.3};           ///< 탈출 판정 예측 시간 [s]
  double escape_max_speed{0.2};         ///< 탈출 명령 최고 점 속도 [m/s]
  // 도킹 예외 (계약 C2)
  double exclusion_stop_distance{0.10};  ///< 도킹 예외 다각형 안 점의 정지 거리 [m]
  double exclusion_timeout{0.3};        ///< 예외 다각형 유효 시간 [s]
};

struct SafetyCommand
{
  double linear{0.0};
  double angular{0.0};
};

struct SafetyStatus
{
  SafetyCommand command;
  SafetyZone zone{SafetyZone::kClear};
  bool estop_active{false};     ///< E-stop 래치 || 정지형 센서 고장 (계약 C1)
  bool estop_latched{false};    ///< E-stop 버튼 래치
  bool proximity_stop{false};   ///< STOP 래치 (접근·접촉 가드·도킹 예외) — zone == STOP
  bool sensor_stop{false};      ///< 정지형 센서 고장
  bool degraded{false};         ///< 저속형 센서 고장
  bool cloud_stale{false};      ///< 필수 깊이 점군이 없거나 늦다 → 전진 저속
  bool command_stale{false};
  bool escaping{false};         ///< 접촉 가드 중 멀어지는 명령 통과
  double speed_limit{0.0};      ///< 적용된 선속도 상한 [m/s]
  double angular_limit{0.0};    ///< 적용된 각속도 상한 [rad/s]
  double min_distance{std::numeric_limits<double>::infinity()};  ///< 존 판정 접근 거리 D [m]
  double lidar_distance{std::numeric_limits<double>::infinity()};  ///< LiDAR 접근 거리 [m]
  double cloud_distance{std::numeric_limits<double>::infinity()};  ///< 깊이 점군 접근 거리 [m]
  double contact_distance{std::numeric_limits<double>::infinity()};  ///< 접촉 가드 거리 [m]
  double exclusion_distance{std::numeric_limits<double>::infinity()};  ///< 예외 다각형 접근 [m]
  double min_ttc{std::numeric_limits<double>::infinity()};
  std::vector<std::string> failed_sensors;
  std::vector<std::string> late_sensors;
  std::vector<std::string> stop_causes;  ///< STOP 원인 (lidar, depth, contact, dock_exclusion)
  std::vector<std::string> reasons;
};

struct ResetResult
{
  bool success{false};
  std::string message;
};

/// 등속 원호 운동 (base_link 속도)
struct Motion
{
  double v{0.0};
  double w{0.0};
};

/// 축 정렬 직사각형 풋프린트([-a, a] × [-b, b], 좌우로 margin 팽창)가 운동 (v, w) 를 따라
/// 움직일 때의 접근 거리. 원호는 접촉점 이동 거리 arc_travel 까지, 그 뒤 끝 자세의 접선
/// 직선으로 total_travel 까지 본다 (회전만 있으면 원호로 total_travel 까지).
/// 단위는 접촉점의 이동 거리 [m].
/// 이미 좌우 여유 안(풋프린트 옆 margin 이내)에 있는 점은 팽창 없는 풋프린트로 판정한다 — 옆에 붙은
/// 벽 점이 "지금 닿음(0)" 이 되지 않게.
class SweptFootprint
{
public:
  SweptFootprint() = default;
  SweptFootprint(
    double half_length, double half_width, double margin, const Motion & motion,
    double arc_travel, double total_travel, double epsilon_v, double epsilon_w);

  bool empty() const {return empty_;}
  /// 점 p 에 닿기까지 접촉점 이동 거리. 풋프린트 안 = 0, 영역 밖 = +inf.
  double approach(const Vec2 & p) const;
  /// 풋프린트 점 중 최고 속력 (꼭짓점) [m/s]
  static double maxPointSpeed(double half_length, double half_width, const Motion & motion);

private:
  double straightApproach(const Vec2 & p, double b) const;
  double arcApproach(const Vec2 & p, double b) const;

  bool empty_{true};
  double a_{0.3};
  double b_{0.2};            ///< 팽창한 반폭
  double b_real_{0.2};       ///< 풋프린트 반폭
  Motion m_;
  bool straight_{true};
  double radius_{0.0};       ///< 순간 회전 중심 (0, R), R = v/w
  double arc_angle_{0.0};    ///< 원호 구간 회전각 한도 [rad]
  bool has_tail_{false};     ///< 원호 끝의 접선 직선 구간
  Pose2D tail_pose_;         ///< 원호 끝 자세 (base_link 기준)
  double tail_offset_{0.0};  ///< 직선 구간 시작까지의 이동 거리 [m]
  double total_{0.0};
};

/// 연속 window 개 값 중 support 번째로 작은 값들의 최소 (window 보다 짧으면 전체에서 support 번째).
/// circular 면 끝과 처음을 잇는다. 값 +inf 는 "해당 없음".
double windowedOrderStatistic(
  const std::vector<double> & values, int window, int support, bool circular);

/// 빔 i 중심 창(물리 폭 width 를 덮는 빔 수 = width / (range_i·Δφ), 최소 min_window, 홀수)에서
/// ceil(support_ratio × 창 길이) 번째로 작은 값들의 최소. 유한 값을 가진 빔만 창 중심이 된다.
/// ranges: 센서 거리 (무효 빔 NaN). 가까운 물체는 빔이 많으므로 창도 길어진다.
double physicalWindowStatistic(
  const std::vector<double> & values, const std::vector<double> & ranges, double angle_increment,
  double width, int min_window, double support_ratio, bool circular);

/// support 번째로 작은 값 (1 = 최솟값). 개수가 모자라면 +inf.
double kthSmallest(std::vector<double> values, int support);

class SafetyGate
{
public:
  SafetyGate(const SafetyParams & params, std::vector<SensorWatch> sensors, double start_time);

  /// base_link 평면 LiDAR 빔 (빔 순서, 무효 빔은 NaN). origin: base_link 의 LiDAR 원점,
  /// angle_increment: 빔 간 각 [rad], circular: 360° 스캔 (끝-처음 이웃), stamp: 촬영 시각
  void setScan(
    const std::vector<Vec2> & beams, const Vec2 & origin, double angle_increment, bool circular,
    double stamp);
  /// 전방 깊이 점군 (지면 높이로 자른 base_link 평면 점, 순서 무관).
  /// capture: 촬영 시각 (지연 보정), received: 수신 시각 (cloud_timeout 기준)
  void setCloud(const std::vector<Vec2> & points, double capture, double received);
  void setCloud(const std::vector<Vec2> & points, double stamp) {setCloud(points, stamp, stamp);}
  /// 센서 생존 신호
  void sensorHeartbeat(const std::string & name, double stamp);
  /// 입력 속도 명령
  void setCommand(double linear, double angular, double stamp);
  /// 측정 속도 (wheel_odom twist). 1차 저역 통과 후 운동 가설로 쓴다
  void setMeasuredVelocity(double linear, double angular, double stamp);
  /// 추적기 최소 TTC (없으면 +inf)
  void setMinTtc(double ttc, double stamp);
  /// E-stop 입력 (source: "estop", "fleet_estop" 등)
  void setEstopSource(const std::string & source, bool active);
  /// E-stop 래치 해제 요청 (safety/reset_estop).
  /// 입력이 하나라도 true 면 거절하고 아무 상태도 바꾸지 않는다.
  ResetResult requestReset();
  /// 도킹 예외 다각형 (base_link). 빈 벡터 = 해제. stamp = 수신 시각 (exclusion_timeout 기준)
  void setExclusionPolygon(const std::vector<Vec2> & polygon, double stamp);

  /// 현재 시각의 판정과 제한된 명령
  SafetyStatus evaluate(double now);

  /// 여유 거리 연속 속도 상한 v(D) (stop_distance 앞에서 멈출 수 있는 속도)
  static double clearanceSpeedLimit(
    double distance, double stop_distance, const SafetyParams & params);
  static double clearanceSpeedLimit(double distance, const SafetyParams & params)
  {
    return clearanceSpeedLimit(distance, params.emergency_stop_distance, params);
  }
  /// 제동 거리 d(u) = u·t_react + u²/(2a)
  static double brakingDistance(double speed, const SafetyParams & params);
  /// 히스테리시스 없는 존 분류
  static SafetyZone classifyZone(double distance, const SafetyParams & params);
  /// 점 → 풋프린트 모서리 거리
  double footprintDistance(const Vec2 & p) const;
  /// 풋프린트 외접원 반경
  double circumscribedRadius() const;
  /// 운동 (v, w) 의 접근 영역 (정지 포락선 + 경고 거리)
  SweptFootprint sweptRegion(const Motion & motion) const;

  const SafetyParams & params() const {return params_;}
  bool estopLatched() const {return estop_latched_;}
  SafetyZone zone() const {return zone_;}

private:
  struct Channel
  {
    double value{std::numeric_limits<double>::infinity()};
    int count{0};  ///< 연속 확정 프레임 수
  };

  bool hasCause(const std::string & cause) const;

  std::vector<Motion> motionHypotheses(const SafetyCommand & cmd, bool cmd_fresh, double now);
  /// 촬영 시각 stamp 이후 측정 운동의 역변환 (촬영 시 base_link 점 → 지금 base_link)
  Pose2D latencyCorrection(double stamp, double now) const;
  bool isSelf(const Vec2 & p) const;
  bool excluded(const Vec2 & p, double now) const;
  bool escapeAllowed(double v, double w, const Pose2D & scan_correction) const;
  bool allSourcesClear() const;
  static void updateChannel(Channel & ch, double value, double threshold, bool new_frame);
  bool confirmed(const Channel & ch, double threshold) const;

  SafetyParams params_;
  std::vector<SensorWatch> sensors_;
  std::map<std::string, double> last_seen_;
  double robustBeams(const std::vector<double> & values, bool guard) const;

  std::vector<Vec2> beams_;
  std::vector<double> ranges_;
  double angle_increment_{0.00872665};
  bool circular_{false};
  double scan_stamp_{-1.0};
  double counted_scan_stamp_{-1.0};
  std::vector<Vec2> cloud_;
  double cloud_stamp_{-1.0};
  double cloud_capture_{-1.0};
  double counted_cloud_stamp_{-1.0};
  SafetyCommand input_;
  double input_stamp_{-1.0};
  Motion measured_;
  double measured_stamp_{-1.0};
  Motion intent_;
  bool have_intent_{false};
  double min_ttc_{std::numeric_limits<double>::infinity()};
  double ttc_stamp_{-1.0};
  std::map<std::string, bool> estop_sources_;
  bool estop_latched_{false};
  std::vector<Vec2> exclusion_;
  double exclusion_stamp_{-1.0};
  bool proximity_stop_{false};
  std::vector<std::string> stop_causes_;
  SafetyZone zone_{SafetyZone::kClear};
  Channel lidar_main_;
  Channel lidar_excl_;
  Channel guard_;
  Channel cloud_main_;
  Channel cloud_excl_;
};

/// 존 이름 (진단 문자열)
const char * zoneName(SafetyZone zone);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__SAFETY_GATE_HPP_
