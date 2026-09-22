// Condition IsTaskAssigned: 수락된 작업이 있으면 SUCCESS, 작업 필드를 블랙보드로 전개한다.
//
// BT.CPP v3 에는 구조체 필드 접근({task.pickup_pose})이 없어 서브트리가 쓰는 값을 평탄한 키로
// 내보낸다. 도크 id 는 Task.msg 에 없으므로 작업 자세를 behavior.yaml 도크 표의 staging 자세와
// 대조해 찾는다.
//   - 도크를 찾으면: 주행 목표 = 그 도크의 staging 자세 (마커 정면, 도킹 시작 위치),
//                    dock_id = 도크 id
//   - 못 찾으면:     주행 목표 = 작업 자세 그대로, dock_id = "" (DockAt 이 도킹을 생략)
#ifndef AMR_BEHAVIOR__BT_NODES__IS_TASK_ASSIGNED_HPP_
#define AMR_BEHAVIOR__BT_NODES__IS_TASK_ASSIGNED_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "amr_msgs/msg/task.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace amr_behavior
{

class IsTaskAssigned : public BT::ConditionNode
{
public:
  IsTaskAssigned(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("task_id", "작업 id"),
      BT::OutputPort<std::string>("item_type", "small / medium / large"),
      BT::OutputPort<double>("item_mass", "물품 질량 [kg] (0 = 물품 표 값)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pickup_goal", "적재 주행 목표 (map)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("dropoff_goal", "하역 주행 목표 (map)"),
      BT::OutputPort<std::string>("pickup_dock_id", "적재 도크 id (\"\" = 도킹 생략)"),
      BT::OutputPort<std::string>("dropoff_dock_id", "하역 도크 id (\"\" = 도킹 생략)"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto task = ctx_->currentTask();
    if (!task) {
      return BT::NodeStatus::FAILURE;
    }
    expand(task->pickup_pose, "pickup_goal", "pickup_dock_id");
    expand(task->dropoff_pose, "dropoff_goal", "dropoff_dock_id");
    setOutput("task_id", task->task_id);
    setOutput("item_type", task->item_type);
    setOutput("item_mass", static_cast<double>(task->item_mass));
    return BT::NodeStatus::SUCCESS;
  }

private:
  void expand(
    const geometry_msgs::msg::PoseStamped & pose, const std::string & goal_key,
    const std::string & dock_key)
  {
    const std::string id = ctx_->resolveDock(pose);
    const auto dock = id.empty() ? std::nullopt : ctx_->dock(id);
    if (dock) {
      setOutput(goal_key, makePose(dock->frame_id, dock->x, dock->y, dock->yaw));
    } else {
      setOutput(goal_key, pose);
    }
    setOutput(dock_key, dock ? dock->id : std::string());
  }

  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__IS_TASK_ASSIGNED_HPP_
