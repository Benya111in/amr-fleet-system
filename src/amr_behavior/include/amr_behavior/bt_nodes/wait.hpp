// Action Wait: Nav2 behavior_server 의 wait 액션 (복구 사이 대기).
#ifndef AMR_BEHAVIOR__BT_NODES__WAIT_HPP_
#define AMR_BEHAVIOR__BT_NODES__WAIT_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/ros_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "rclcpp/duration.hpp"

namespace amr_behavior
{

class WaitAction : public RosActionNode<nav2_msgs::action::Wait>
{
public:
  WaitAction(const std::string & name, const BT::NodeConfiguration & config, RosNodeParams params)
  : RosActionNode(name, config, std::move(params), "wait") {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<double>("wait_duration", 1.0, "대기 시간 [s]")});
  }

protected:
  bool setGoal(Goal & goal) override
  {
    double duration = 1.0;
    getInput("wait_duration", duration);
    if (!(duration >= 0.0)) {
      return false;
    }
    goal.time = rclcpp::Duration::from_seconds(duration);
    return true;
  }
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__WAIT_HPP_
