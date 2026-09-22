// 작업 트리 생성: 설정 키(behavior.yaml) → 루트 블랙보드, XML 로드.
//
// BT.CPP v3.8 의 SubTreePlus 블랙보드는 기본적으로 명시 매핑한 키만 부모와 공유한다. 재시도
// 횟수·타임아웃 같은 설정 키를 중첩 서브트리마다 XML 에 되풀이하지 않도록 모든 SubTreePlus 호출에
// __autoremap="true" 를 두고, 트리 생성 전에 루트 블랙보드에 설정 키를 써 둔다 (항목 연결은 생성
// 시점에 일어난다).
#ifndef AMR_BEHAVIOR__TASK_TREE_HPP_
#define AMR_BEHAVIOR__TASK_TREE_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace amr_behavior
{

/// 트리 설정 (behavior.yaml task_executor_node 파라미터와 1:1, 단위는 키 이름 접미사).
struct TaskTreeConfig
{
  // 재시도 횟수 (RetryUntilSuccessful num_attempts: 첫 시도 포함)
  /// 주행 1 + 재시도 2 (sequences.md: RetryUntilSuccessful(2) 재시도)
  int nav_attempts{3};
  int perception_attempts{3};       ///< 인식 확인 최대 3회
  int dock_attempts{3};             ///< 명세: 도킹 최대 3회 실패 시 에러 보고
  unsigned dock_max_retries{1};     ///< goal 당 도킹 서버 접근 횟수 (단일 재시도 카운터 → 1)
  // 시간 [ms] (BT.CPP Timeout/Delay 는 벽시계)
  unsigned perception_timeout_ms{5000};
  unsigned relocalization_timeout_ms{30000};
  unsigned charge_timeout_ms{3600000};
  unsigned charge_retry_delay_ms{10000};
  /// 실패 보고 뒤 ERROR 유지 [ms]: fleet_adapter 가 2 Hz 로 샘플해도 RobotState ERROR 가 보이게
  unsigned error_hold_ms{1500};
  // 인식 조건
  std::string perception_class{"box"};
  double perception_max_distance{2.0};   ///< [m]
  // 배터리 [%]
  double battery_low_percent{20.0};
  double battery_resume_percent{80.0};
  // 복구 동작
  double recovery_spin_angle{1.57};      ///< [rad] 주행 복구 회전
  double recovery_wait_s{1.0};           ///< [s] 주행 복구 대기
  double recovery_backup_dist{0.3};      ///< [m] 인식 복구 후진
  double perception_spin_angle{0.52};    ///< [rad] 인식 복구 회전 (≈ 30°)
  double dock_backup_dist{0.3};          ///< [m] 도킹 재시도 전 후진
  double marker_max_age{1.0};            ///< [s] 재시도 전 마커 가시 판정
  double undock_dist{1.55};              ///< [m] 적재·하역·충전 뒤 이탈 후진 (도크 staging 까지)
  // 위치 (충전소는 SelectCharger 가 실행 중에 고른다: charger_goal / charger_dock_id)
  geometry_msgs::msg::PoseStamped waiting_pose;   ///< 복귀(대기 구역) 자세
};

/// 루트 블랙보드에 설정 키를 쓴다 (타입은 BT 포트 타입과 일치: int / unsigned / double / string).
void writeConfig(const TaskTreeConfig & config, BT::Blackboard & blackboard);

/// writeConfig 가 루트에 쓰는 설정 키 + 실행기 상태 키(task_step, charge_step, fail_reason,
/// charger_*, pickup_perceive_turn, last_dock_*).
const std::vector<std::string> & globalKeys();

/// xml_path(메인 트리, <include> 포함)로 트리를 만든다. 루트 블랙보드는 config 로 초기화된다.
BT::Tree createTaskTree(
  BT::BehaviorTreeFactory & factory, const std::string & xml_path,
  const TaskTreeConfig & config);

/// 새 작업 수락 시 진행 단계·실패 사유를 초기화한다.
void resetTaskProgress(BT::Blackboard & blackboard);

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__TASK_TREE_HPP_
