// Action Spin: Nav2 behavior_server 의 spin 액션 (제자리 회전: 복구·재탐색, 물품 쪽으로 돌아보기).
// reverse=true 면 −spin_dist 로 돈다 (돌아본 뒤 원래 방위로). |spin_dist| < 1 mrad 이면 goal 없이
// SUCCESS (돌아볼 방위가 없는 도크).
#ifndef AMR_BEHAVIOR__BT_NODES__SPIN_HPP_
#define AMR_BEHAVIOR__BT_NODES__SPIN_HPP_

#include <cmath>
#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/ros_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "rclcpp/duration.hpp"

namespace amr_behavior
{

class SpinAction : public RosActionNode<nav2_msgs::action::Spin>
{
public:
  SpinAction(const std::string & name, const BT::NodeConfiguration & config, RosNodeParams params)
  : RosActionNode(name, config, std::move(params), "spin") {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("spin_dist", 1.57, "회전각 [rad] (+ = 반시계)"),
        BT::InputPort<double>("time_allowance", 10.0, "허용 시간 [s]"),
        BT::InputPort<bool>("reverse", false, "true 면 −spin_dist"),
      });
  }

protected:
  bool skipGoal() override
  {
    double dist = 1.57;
    getInput("spin_dist", dist);
    return std::isfinite(dist) && std::fabs(dist) < 1e-3;
  }

  bool setGoal(Goal & goal) override
  {
    double dist = 1.57;
    double allowance = 10.0;
    bool reverse = false;
    getInput("spin_dist", dist);
    getInput("time_allowance", allowance);
    getInput("reverse", reverse);
    if (!std::isfinite(dist) || !(allowance > 0.0)) {
      return false;
    }
    goal.target_yaw = static_cast<float>(reverse ? -dist : dist);
    goal.time_allowance = rclcpp::Duration::from_seconds(allowance);
    return true;
  }
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__SPIN_HPP_
