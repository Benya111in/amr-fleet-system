// Action Spin: Nav2 behavior_server 의 spin 액션 (제자리 회전, 복구·재탐색용).
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
      });
  }

protected:
  bool setGoal(Goal & goal) override
  {
    double dist = 1.57;
    double allowance = 10.0;
    getInput("spin_dist", dist);
    getInput("time_allowance", allowance);
    if (!std::isfinite(dist) || !(allowance > 0.0)) {
      return false;
    }
    goal.target_yaw = static_cast<float>(dist);
    goal.time_allowance = rclcpp::Duration::from_seconds(allowance);
    return true;
  }
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__SPIN_HPP_
