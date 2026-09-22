// Action BackUp: Nav2 behavior_server 의 backup 액션 (직선 후진, 복구·도킹 재시도용).
#ifndef AMR_BEHAVIOR__BT_NODES__BACK_UP_HPP_
#define AMR_BEHAVIOR__BT_NODES__BACK_UP_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/ros_action_node.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "rclcpp/duration.hpp"

namespace amr_behavior
{

class BackUpAction : public RosActionNode<nav2_msgs::action::BackUp>
{
public:
  BackUpAction(
    const std::string & name, const BT::NodeConfiguration & config,
    RosNodeParams params)
  : RosActionNode(name, config, std::move(params), "backup") {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("backup_dist", 0.3, "후진 거리 [m] (양수)"),
        BT::InputPort<double>("backup_speed", 0.1, "후진 속도 [m/s] (양수)"),
        BT::InputPort<double>("time_allowance", 10.0, "허용 시간 [s]"),
      });
  }

protected:
  bool setGoal(Goal & goal) override
  {
    double dist = 0.3;
    double speed = 0.1;
    double allowance = 10.0;
    getInput("backup_dist", dist);
    getInput("backup_speed", speed);
    getInput("time_allowance", allowance);
    if (!(dist > 0.0) || !(speed > 0.0) || !(allowance > 0.0)) {
      return false;
    }
    // Humble BackUp 은 target.x·speed 의 부호와 무관하게 후진한다 (|x|, -|speed|).
    goal.target.x = dist;
    goal.speed = static_cast<float>(speed);
    goal.time_allowance = rclcpp::Duration::from_seconds(allowance);
    return true;
  }
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__BACK_UP_HPP_
