// Action NavigateToPose: Nav2 bt_navigator 의 navigate_to_pose 액션 클라이언트.
//
// goal 자세의 frame_id 가 비어 있으면 skip_if_empty=true 일 때 서버에 보내지 않고 SUCCESS
// (Yield 서브트리: 양보 자세가 없으면 제자리 대기), false 면 FAILURE.
#ifndef AMR_BEHAVIOR__BT_NODES__NAVIGATE_TO_POSE_HPP_
#define AMR_BEHAVIOR__BT_NODES__NAVIGATE_TO_POSE_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
#include "amr_behavior/bt_nodes/ros_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace amr_behavior
{

class NavigateToPoseAction : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToPoseAction(
    const std::string & name, const BT::NodeConfiguration & config, RosNodeParams params)
  : RosActionNode(name, config, std::move(params), "navigate_to_pose") {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "목표 자세 (map)"),
        BT::InputPort<std::string>("behavior_tree", "", "Nav2 BT XML 경로 (\"\" = 서버 기본)"),
        BT::InputPort<bool>("skip_if_empty", false, "goal frame_id 가 비면 SUCCESS"),
        BT::OutputPort<double>("distance_remaining", "피드백: 남은 거리 [m]"),
      });
  }

protected:
  bool skipGoal() override
  {
    geometry_msgs::msg::PoseStamped goal;
    bool skip_if_empty = false;
    getInput("skip_if_empty", skip_if_empty);
    const bool has_goal = static_cast<bool>(getInput("goal", goal));
    return skip_if_empty && (!has_goal || goal.header.frame_id.empty());
  }

  bool setGoal(Goal & goal) override
  {
    if (!getInput("goal", goal.pose) || goal.pose.header.frame_id.empty()) {
      return false;
    }
    goal.behavior_tree.clear();
    getInput("behavior_tree", goal.behavior_tree);
    return true;
  }

  void onFeedback(const Feedback & feedback) override
  {
    setOutput("distance_remaining", static_cast<double>(feedback.distance_remaining));
  }
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__NAVIGATE_TO_POSE_HPP_
