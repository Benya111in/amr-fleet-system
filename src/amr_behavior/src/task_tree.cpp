// 작업 트리 생성 (task_tree.hpp 설명 참고).
#include "amr_behavior/task_tree.hpp"

#include <string>
#include <vector>

namespace amr_behavior
{

void writeConfig(const TaskTreeConfig & c, BT::Blackboard & bb)
{
  bb.set<int>("nav_attempts", c.nav_attempts);
  bb.set<int>("perception_attempts", c.perception_attempts);
  bb.set<int>("dock_attempts", c.dock_attempts);
  bb.set<unsigned>("dock_max_retries", c.dock_max_retries);
  bb.set<unsigned>("perception_timeout_ms", c.perception_timeout_ms);
  bb.set<unsigned>("relocalization_timeout_ms", c.relocalization_timeout_ms);
  bb.set<unsigned>("charge_timeout_ms", c.charge_timeout_ms);
  bb.set<unsigned>("charge_retry_delay_ms", c.charge_retry_delay_ms);
  bb.set<std::string>("perception_class", c.perception_class);
  bb.set<double>("perception_max_distance", c.perception_max_distance);
  bb.set<double>("battery_low_percent", c.battery_low_percent);
  bb.set<double>("battery_resume_percent", c.battery_resume_percent);
  bb.set<double>("recovery_spin_angle", c.recovery_spin_angle);
  bb.set<double>("recovery_wait_s", c.recovery_wait_s);
  bb.set<double>("recovery_backup_dist", c.recovery_backup_dist);
  bb.set<double>("perception_spin_angle", c.perception_spin_angle);
  bb.set<double>("dock_backup_dist", c.dock_backup_dist);
  bb.set<double>("marker_max_age", c.marker_max_age);
  bb.set<double>("undock_dist", c.undock_dist);
  bb.set<geometry_msgs::msg::PoseStamped>("waiting_pose", c.waiting_pose);
  bb.set<geometry_msgs::msg::PoseStamped>("charger_goal", c.charger_goal);
  bb.set<std::string>("charger_dock_id", c.charger_dock_id);
  // 실행기 상태 키 (타입 고정용 초기값)
  bb.set<unsigned>("last_dock_attempts", 0U);
  bb.set<double>("last_dock_position_error", 0.0);
  bb.set<double>("last_dock_angle_error", 0.0);
  resetTaskProgress(bb);
}

const std::vector<std::string> & globalKeys()
{
  static const std::vector<std::string> kKeys = {
    "nav_attempts", "perception_attempts", "dock_attempts", "dock_max_retries",
    "perception_timeout_ms", "relocalization_timeout_ms", "charge_timeout_ms",
    "charge_retry_delay_ms", "perception_class", "perception_max_distance",
    "battery_low_percent", "battery_resume_percent", "recovery_spin_angle", "recovery_wait_s",
    "recovery_backup_dist", "perception_spin_angle", "dock_backup_dist", "marker_max_age",
    "undock_dist", "waiting_pose", "charger_goal", "charger_dock_id",
    "last_dock_attempts", "last_dock_position_error", "last_dock_angle_error",
    "task_step", "fail_reason"};
  return kKeys;
}

BT::Tree createTaskTree(
  BT::BehaviorTreeFactory & factory, const std::string & xml_path,
  const TaskTreeConfig & config)
{
  // 루트 블랙보드를 먼저 채운다: 서브트리의 __autoremap 항목은 트리 생성 시점에 부모 항목과
  // 연결되므로 같은 이름·타입의 루트 항목이 있어야 설정값을 공유한다.
  auto blackboard = BT::Blackboard::create();
  writeConfig(config, *blackboard);
  return factory.createTreeFromFile(xml_path, blackboard);
}

void resetTaskProgress(BT::Blackboard & bb)
{
  bb.set<int>("task_step", 0);
  bb.set<std::string>("fail_reason", "unknown");
}

}  // namespace amr_behavior
