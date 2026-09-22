// 작업 실행기의 공유 상태 (ROS 비의존 코어).
//
// - 구독 콜백이 입력 상태(배터리/E-stop/교통/위치 상실/인식/마커)를 갱신하고,
//   BT 조건 노드가 tick 마다 이 스냅샷을 읽는다.
// - 출력(단계·작업 상태·적재·충전)은 Hooks 로 내보낸다. 노드는 ROS 발행자에, 테스트는
//   기록기에 연결한다.
// - 시간원(Clock)은 주입한다: 노드 = steady clock, 테스트 = 가짜 시계.
#ifndef AMR_BEHAVIOR__EXECUTOR_CONTEXT_HPP_
#define AMR_BEHAVIOR__EXECUTOR_CONTEXT_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "amr_msgs/msg/detected_object_array.hpp"
#include "amr_msgs/msg/task.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace amr_behavior
{

/// executor/phase 문자열 (fleet_adapter 가 대소문자 무시로 RobotState.status 에 매핑).
namespace phase
{
constexpr const char * kIdle = "IDLE";
constexpr const char * kMoving = "MOVING";
constexpr const char * kDocking = "DOCKING";
constexpr const char * kLoading = "LOADING";
constexpr const char * kUnloading = "UNLOADING";
constexpr const char * kReturning = "RETURNING";
constexpr const char * kCharging = "CHARGING";
constexpr const char * kError = "ERROR";
constexpr const char * kRecovering = "RECOVERING";
/// 알려진 단계인지 (SetPhase 입력 검증용).
bool isKnown(const std::string & name);
}  // namespace phase

/// 적재 물품 표 (robot_params.yaml payload.<type>).
struct PayloadSpec
{
  double mass{0.0};       ///< [kg]
  double load_time{0.0};  ///< [s] 적재/하역 소요 시간
};

/// 도크 등록 정보 (behavior.yaml docks.<id>). (x, y, yaw) = staging 자세: 마커 정면, 도킹 시작
/// 위치.
struct DockSpec
{
  std::string id;
  std::string frame_id{"map"};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};   ///< [rad] 마커를 바라보는 방향
};

/// 인식 조건 질의 (IsObjectDetected 포트).
struct ObjectQuery
{
  std::string class_name{"box"};
  double max_distance{2.0};     ///< [m] base_link 기준 거리 상한
  double min_confidence{0.5};
  double max_age{1.0};          ///< [s] 이보다 오래된 인식 결과는 무시
};

/// assign_task 수락 정책.
struct AcceptPolicy
{
  double battery_low_percent{20.0};   ///< 이 미만이면 거절 (충전 우선)
  bool accept_while_returning{false};  ///< RETURNING 중에도 수락할지 (기본: IDLE 에서만)
};

/// 작업 수락 결과.
struct AcceptDecision
{
  bool accepted{false};
  /// "accepted" / "busy:<id>" / "estop" / "lost" / "battery_low" / "invalid:<이유>"
  std::string message;
};

/// 컨텍스트가 바깥으로 내보내는 부수효과. 비어 있으면 무시한다.
struct ExecutorHooks
{
  std::function<void(const std::string &)> publish_phase;
  std::function<void(const amr_msgs::msg::Task &)> publish_task_status;
  std::function<void(const std::string &)> publish_payload_attach;  ///< "" = 분리
  std::function<void(double)> publish_payload_mass;                 ///< [kg]
  std::function<void(bool)> publish_charging;
  std::function<void(const std::string &)> log_info;
  std::function<void(const std::string &)> log_warn;
};

/// 실행기 공유 상태. 모든 메서드는 스레드 안전(내부 뮤텍스).
class ExecutorContext
{
public:
  using Clock = std::function<double ()>;   ///< [s], 단조 증가

  explicit ExecutorContext(Clock clock, ExecutorHooks hooks = {});

  double now() const;
  void setHooks(ExecutorHooks hooks);

  // ---------------------------------------------------------------- 설정
  void setRobotId(const std::string & robot_id);
  std::string robotId() const;
  void setAcceptPolicy(const AcceptPolicy & policy);
  void setPayloadTable(const std::map<std::string, PayloadSpec> & table);
  std::optional<PayloadSpec> payloadSpec(const std::string & item_type) const;
  /// position_tolerance [m] 안의 가장 가까운 도크로 대응시킨다. yaw_tolerance [rad] ≥ π 면 방향은
  /// 보지 않는다.
  void setDocks(
    const std::vector<DockSpec> & docks, double position_tolerance,
    double yaw_tolerance);
  /// 작업 자세(map)에 대응하는 도크 id. 허용오차 밖이면 빈 문자열.
  std::string resolveDock(const geometry_msgs::msg::PoseStamped & pose) const;
  std::optional<DockSpec> dock(const std::string & id) const;

  // ---------------------------------------------------------------- 입력 상태
  void updateBattery(double percent);     ///< [%], NaN = 미측정
  std::optional<double> battery() const;  ///< 미수신/미측정이면 nullopt
  void updateEstop(bool active);
  bool estopActive() const;
  void updateTrafficHold(bool hold);
  bool trafficHold() const;
  void updateYieldPose(const geometry_msgs::msg::PoseStamped & pose);
  /// hold 가 걸린 뒤 받은 양보 자세. 없으면 nullopt.
  std::optional<geometry_msgs::msg::PoseStamped> yieldPose() const;
  void updateLocalizationLost(bool lost);
  bool localizationLost() const;
  void updateDetectedObjects(const amr_msgs::msg::DetectedObjectArray & objects);
  bool objectDetected(const ObjectQuery & query) const;
  void updateDockMarker(const geometry_msgs::msg::PoseStamped & pose);
  /// 마지막 마커 관측 이후 경과 [s]. 한 번도 없으면 +inf.
  double dockMarkerAge() const;

  // ---------------------------------------------------------------- 작업
  AcceptDecision evaluateTask(const amr_msgs::msg::Task & task) const;
  /// 정책을 통과하면 IN_PROGRESS 로 바꿔 보관하고 task_status 를 발행한다.
  AcceptDecision acceptTask(const amr_msgs::msg::Task & task);
  bool hasTask() const;
  std::optional<amr_msgs::msg::Task> currentTask() const;
  /// 상태 보고. COMPLETED/FAILED 는 현재 작업을 비운다. 작업이 없으면 false.
  bool reportStatus(uint8_t status, const std::string & reason);
  /// 마지막 종료 작업의 결과/사유 (로그·테스트용).
  std::string lastFailureReason() const;

  // ---------------------------------------------------------------- 출력
  void setPhase(const std::string & name);   ///< 바뀔 때만 발행
  std::string phase() const;
  void attachPayload(const std::string & item_type, double mass);
  void detachPayload();
  std::string attachedPayload() const;
  void setCharging(bool enable);
  bool charging() const;

  void logInfo(const std::string & msg) const;
  void logWarn(const std::string & msg) const;

private:
  static double yawOf(const geometry_msgs::msg::PoseStamped & pose);

  Clock clock_;
  ExecutorHooks hooks_;
  mutable std::mutex mutex_;

  std::string robot_id_;
  AcceptPolicy policy_;
  std::map<std::string, PayloadSpec> payloads_;
  std::vector<DockSpec> docks_;
  double dock_pos_tol_{0.5};
  double dock_yaw_tol_{0.35};

  std::optional<double> battery_;
  bool estop_{false};
  bool traffic_hold_{false};
  double hold_since_{0.0};
  std::optional<geometry_msgs::msg::PoseStamped> yield_pose_;
  double yield_stamp_{0.0};
  bool localization_lost_{false};
  amr_msgs::msg::DetectedObjectArray objects_;
  double objects_stamp_{-1.0};
  double marker_stamp_{-1.0};

  std::optional<amr_msgs::msg::Task> task_;
  std::string last_failure_reason_;
  std::string phase_;
  std::string payload_;
  bool charging_{false};
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__EXECUTOR_CONTEXT_HPP_
