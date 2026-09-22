// Nav2 BT 조건 플러그인 IsTTCBelowThreshold (BehaviorTree.CPP v3, nav2_behavior_tree 방식).
//
// bt_navigator 의 plugin_lib_names 에 amr_is_ttc_below_threshold_condition_bt_node 를
// 넣으면 로드된다. perception/tracked_obstacles 의 최소 TTC 가 threshold 미만이면 SUCCESS
// → 주행 BT 가 경로를 재계획한다. 예 (TTC 정상이면 통과, 위험하면 재계획):
//   <Fallback>
//     <Inverter><IsTTCBelowThreshold threshold="2.0" cooldown="1.0"/></Inverter>
//     <ComputePathToPose goal="{goal}" path="{path}" planner_id="AStar"/>
//   </Fallback>
// 블랙보드 "node"(rclcpp::Node::SharedPtr, bt_navigator 가 넣는다)로 구독을 만들고,
// 전용 콜백 그룹을 tick 마다 spin_some 한다 (nav2 IsBatteryLowCondition 과 같은 구조).
#ifndef AMR_BEHAVIOR__PLUGINS__IS_TTC_BELOW_THRESHOLD_CONDITION_HPP_
#define AMR_BEHAVIOR__PLUGINS__IS_TTC_BELOW_THRESHOLD_CONDITION_HPP_

#include <memory>
#include <string>

#include "amr_behavior/ttc/ttc_trigger.hpp"
#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace amr_behavior
{

class IsTTCBelowThreshold : public BT::ConditionNode
{
public:
  IsTTCBelowThreshold(const std::string & name, const BT::NodeConfiguration & config);
  IsTTCBelowThreshold() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("threshold", 2.0, "TTC 임계 [s] (이 미만이면 SUCCESS)"),
      BT::InputPort<std::string>(
        "tracked_obstacles_topic", std::string("perception/tracked_obstacles"),
        "추적 결과 토픽 (상대 이름)"),
      BT::InputPort<double>("max_age", 0.5, "메시지 유효 시간 [s]"),
      BT::InputPort<double>("cooldown", 0.0, "SUCCESS 후 재트리거 금지 시간 [s]"),
      BT::InputPort<bool>("only_dynamic", true, "동적 트랙만 고려"),
      BT::InputPort<double>("min_confidence", 0.0, "트랙 신뢰도 하한"),
      BT::OutputPort<double>("min_ttc", "최소 TTC [s] (없으면 inf)"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void onObstacles(const amr_msgs::msg::TrackedObstacleArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<amr_msgs::msg::TrackedObstacleArray>::SharedPtr sub_;
  TtcTrigger trigger_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__PLUGINS__IS_TTC_BELOW_THRESHOLD_CONDITION_HPP_
