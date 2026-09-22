// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 안전 게이트 순수 로직 (ROS 비의존) — safety_node 가 50 Hz 로 호출한다.
//
// 입력: 속도 명령(cmd_vel_smoothed), base_link 평면 스캔 점, 추적기 최소 TTC, E-stop 입력들,
//       센서 생존 신호, (선택) 도킹 예외 다각형.
// 출력: 제한된 속도 명령, 존(CLEAR/WARNING/CRITICAL/STOP), estop_active, 사유 목록.
//
// 규칙 (config/robot_params.yaml safety.*, docs/architecture/sequences.md §2,
//       docs/algorithms/tracking.md §6):
//   D = 풋프린트(0.60×0.40) 모서리에서 장애물 점까지 최단 거리 (distance_reference: footprint_edge)
//   D <= 0.30 → 즉시 정지(래치, D > stop_release_distance 에서 자동 해제)
//   D <= 0.50 → CRITICAL, |v| <= 0.2 / D <= 1.00 → WARNING, |v| <= 0.5
//   여유 거리 연속 제한 v <= -a·t + √((a·t)² + 2a(D - 0.30))  (clearance_speed_limit_enabled)
//   TTC <= τ_crit(2.15 s) → v <= a·(TTC - t_react)
//   각속도: 풋프린트 꼭짓점 속도 |ω|·r_circ 도 거리 기반 상한을 넘지 않게 (곡률 유지 비례 축소)
//   센서 타임아웃: 동작 stop → 정지 + estop_active, degraded → degraded_mode_max_speed
//   E-stop 입력(estop, /fleet/estop): true 수신 즉시 래치. 해제는 모든 입력이 명시적 false 이고
//   (estop_release_requires_reset 이면) reset 요청까지 있어야 한다.

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

/// 존 판정에 쓰는 점의 범위
enum class ZoneRegion
{
  kOmni,    ///< 모든 방향 (robot_params.yaml distance_reference: footprint_edge 그대로)
  /// 진행 방향 보호 영역: 전방 점만 존 판정, 측/후방 점은 lateral_stop_distance 로만 정지
  kMotion,
};

struct SensorWatch
{
  std::string name;
  double timeout{0.3};
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
  double stop_release_distance{0.50};   ///< 정지 래치 자동 해제 거리 (히스테리시스)
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
  double reset_grace{1.0};              ///< reset 요청 후 false 도착을 기다리는 시간 [s]
  ZoneRegion zone_region{ZoneRegion::kOmni};
  double lateral_stop_distance{0.05};   ///< kMotion: 측/후방 점 정지 거리 [m]
  double motion_epsilon_v{0.01};        ///< [m/s]
  double motion_epsilon_w{0.05};        ///< [rad/s]
  double self_filter_margin{0.02};      ///< 풋프린트 안쪽(축소) 점 = 자기 차체, 무시 [m]
  bool allow_escape{true};              ///< 근접 정지 중 멀어지는 명령 허용 (kOmni)
  double escape_horizon{0.5};           ///< 탈출 판정 예측 시간 [s]
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
  bool estop_active{false};     ///< 정지 사유 (래치/근접/정지형 센서 고장) 중 하나라도
  bool estop_latched{false};    ///< E-stop 버튼 래치
  bool proximity_stop{false};   ///< 0.3 m 근접 정지
  bool sensor_stop{false};      ///< 정지형 센서 고장
  bool degraded{false};         ///< 저속형 센서 고장
  bool command_stale{false};
  bool escaping{false};         ///< 근접 정지 중 탈출 명령 통과
  double speed_limit{0.0};      ///< 적용된 선속도 상한 [m/s]
  double angular_limit{0.0};    ///< 적용된 각속도 상한 [rad/s]
  double min_distance{std::numeric_limits<double>::infinity()};  ///< 존 판정 거리 D [m]
  double min_ttc{std::numeric_limits<double>::infinity()};
  std::vector<std::string> failed_sensors;
  std::vector<std::string> reasons;
};

struct ResetResult
{
  bool success{false};
  std::string message;
};

class SafetyGate
{
public:
  SafetyGate(const SafetyParams & params, std::vector<SensorWatch> sensors, double start_time);

  /// base_link 평면 스캔 점 (필터된 LiDAR). 호출 시각 stamp [s]
  void setScanPoints(const std::vector<Vec2> & points, double stamp);
  /// 센서 생존 신호
  void sensorHeartbeat(const std::string & name, double stamp);
  /// 입력 속도 명령
  void setCommand(double linear, double angular, double stamp);
  /// 추적기 최소 TTC (없으면 +inf)
  void setMinTtc(double ttc, double stamp);
  /// E-stop 입력 (source: "estop", "fleet_estop" 등)
  void setEstopSource(const std::string & source, bool active, double now);
  /// E-stop 래치 해제 요청 (safety/reset_estop)
  ResetResult requestReset(double now);
  /// 도킹 예외 다각형 (base_link). 빈 벡터 = 해제
  void setExclusionPolygon(const std::vector<Vec2> & polygon, double stamp);

  /// 현재 시각의 판정과 제한된 명령
  SafetyStatus evaluate(double now);

  /// 여유 거리 연속 속도 상한 v(D)
  static double clearanceSpeedLimit(double distance, const SafetyParams & params);
  /// 히스테리시스 없는 존 분류
  static SafetyZone classifyZone(double distance, const SafetyParams & params);
  /// 점 → 풋프린트 모서리 거리
  double footprintDistance(const Vec2 & p) const;
  /// 풋프린트 외접원 반경
  double circumscribedRadius() const;

  const SafetyParams & params() const {return params_;}
  bool estopLatched() const {return estop_latched_;}
  SafetyZone zone() const {return zone_;}

private:
  struct Distances
  {
    double zone{std::numeric_limits<double>::infinity()};
    double lateral{std::numeric_limits<double>::infinity()};
    double exclusion{std::numeric_limits<double>::infinity()};
    bool exclusion_present{false};
  };

  Distances computeDistances(double now, double v, double w) const;
  double minDistanceAfterMotion(double v, double w, double horizon) const;
  bool allSourcesClear() const;
  void releaseIfArmed(double now);

  SafetyParams params_;
  std::vector<SensorWatch> sensors_;
  std::map<std::string, double> last_seen_;
  std::vector<Vec2> scan_points_;
  double scan_stamp_{-1.0};
  SafetyCommand input_;
  double input_stamp_{-1.0};
  double min_ttc_{std::numeric_limits<double>::infinity()};
  double ttc_stamp_{-1.0};
  std::map<std::string, bool> estop_sources_;
  bool estop_latched_{false};
  double reset_armed_until_{-1.0};
  std::vector<Vec2> exclusion_;
  double exclusion_stamp_{-1.0};
  bool proximity_stop_{false};
  SafetyZone zone_{SafetyZone::kClear};
};

/// 존 이름 (진단 문자열)
const char * zoneName(SafetyZone zone);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__SAFETY_GATE_HPP_
