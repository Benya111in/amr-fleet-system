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
constexpr const char * kPerceiving = "PERCEIVING";
constexpr const char * kDocking = "DOCKING";
constexpr const char * kLoading = "LOADING";
constexpr const char * kUnloading = "UNLOADING";
constexpr const char * kReturning = "RETURNING";
constexpr const char * kCharging = "CHARGING";
constexpr const char * kError = "ERROR";
constexpr const char * kRecovering = "RECOVERING";
constexpr const char * kUndocking = "UNDOCKING";
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
  /// [rad] staging 위치에서 물품이 카메라 시야에 드는 방위 (없으면 staging 방위 그대로 인식)
  std::optional<double> perceive_yaw;
};

/// 적재한 물품의 출처 (하역 실패 시 되돌려 놓을 곳).
struct PayloadOrigin
{
  std::string item_type;
  geometry_msgs::msg::PoseStamped goal;   ///< 적재 도크 staging 자세 (map)
  std::string dock_id;                    ///< 적재 도크 ("" = 도킹 없이 적재)
};

/// 다른 로봇의 충전소 점유 (/fleet/charger_claims 심장박동).
struct ChargerClaim
{
  std::string charger;
  double since{0.0};   ///< 점유 시작 [s] (같은 /clock)
  double heard{0.0};   ///< 마지막 수신 [s]
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
  /// false(기본): 적재·하역 자세가 등록 도크와 맞지 않으면 "invalid:no_dock:<pickup|dropoff>" 거절.
  /// true: 도킹 없는 작업으로 받아 Nav2 도착 자세에서 적재/하역한다 (경고 로그).
  bool allow_undocked_tasks{false};
};

/// 작업 수락 결과.
struct AcceptDecision
{
  bool accepted{false};
  /// "accepted" / "busy:<id>" / "estop" / "lost" / "battery_low" / "blocked:payload" /
  /// "invalid:<이유>"
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
  /// 자기 충전소 점유 (charger "" = 해제, since = 점유 시작 [s])
  std::function<void(const std::string &, double)> publish_charger_claim;
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

  // ---------------------------------------------------------------- 충전소 할당
  /// 후보 충전소 (docks 표의 id). robot_index 만큼 돌려 로봇마다 첫 선택이 다르게 한다.
  void setChargers(const std::vector<std::string> & ids, int robot_index, double claim_timeout);
  std::vector<std::string> chargerPreference() const;
  /// 다른 로봇의 점유 심장박동 (charger "" = 해제). 자기 robot_id 는 무시한다.
  void updateChargerClaim(const std::string & robot, const std::string & charger, double since);
  /// 충전소 선택: 자기 점유가 여전히 유효하면 그대로, 아니면 선호 순서의 첫 빈 충전소를 점유한다.
  /// 먼저 점유한(since 가 이른, 같으면 id 가 작은) 로봇이 이긴다. 빈 곳이 없으면 nullopt.
  /// keep_current = true 면 자기 점유를 무조건 유지한다 (이미 도킹한 뒤 — 충전 중에는 양보하지
  /// 않는다).
  /// 점유가 바뀔 때만 publish_charger_claim 을 부른다 (심장박동은 노드 타이머).
  std::optional<DockSpec> selectCharger(bool keep_current = false);
  void releaseCharger();
  std::string chargerClaim() const;
  double chargerClaimSince() const;

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
  /// 작업이 끝난 뒤 대기 구역 복귀가 남았는지 (COMPLETED/FAILED 보고 시 설정, 복귀 뒤 해제).
  /// E-stop 으로 복귀가 끊겨도 재개 후 다시 복귀한다.
  bool returnPending() const;
  void clearReturnPending();

  // ---------------------------------------------------------------- 출력
  void setPhase(const std::string & name);   ///< 바뀔 때만 발행
  std::string phase() const;
  void attachPayload(
    const std::string & item_type, double mass, const PayloadOrigin & origin = PayloadOrigin());
  void detachPayload();
  std::string attachedPayload() const;
  /// 작업 없이 실린 물품(하역 실패 뒤)이고 되돌려 놓기를 아직 시도하지 않았으면 그 출처.
  std::optional<PayloadOrigin> strandedPayload() const;
  /// 되돌려 놓기 시도 완료 (성공이면 detach 로 이미 비었고, 실패면 물품을 실은 채 작업을 막는다).
  void markPayloadReturnAttempted();
  void setCharging(bool enable);
  bool charging() const;

  void logInfo(const std::string & msg) const;
  void logWarn(const std::string & msg) const;

private:
  static double yawOf(const geometry_msgs::msg::PoseStamped & pose);
  std::string resolveDockLocked(const geometry_msgs::msg::PoseStamped & pose) const;

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
  bool return_pending_{false};
  std::string phase_;
  std::string payload_;
  PayloadOrigin payload_origin_;
  bool payload_return_attempted_{false};
  bool charging_{false};

  std::vector<std::string> charger_preference_;
  double claim_timeout_{3.0};
  std::map<std::string, ChargerClaim> claims_;   ///< 다른 로봇 → 점유
  std::string charger_claim_;                    ///< 자기 점유 ("" = 없음)
  double charger_claim_since_{0.0};
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__EXECUTOR_CONTEXT_HPP_
