// Condition IsTrafficHold: 교통 관리자가 traffic/hold 를 걸었으면 SUCCESS.
// hold 이후 받은 traffic/yield_pose 를 yield_pose 로 내보낸다 (없으면 frame_id 가 빈 자세).
#ifndef AMR_BEHAVIOR__BT_NODES__IS_TRAFFIC_HOLD_HPP_
#define AMR_BEHAVIOR__BT_NODES__IS_TRAFFIC_HOLD_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace amr_behavior
{

class IsTrafficHold : public BT::ConditionNode
{
public:
  IsTrafficHold(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "yield_pose", "양보 위치 (없으면 frame_id 가 빈 자세 → 제자리 대기)")};
  }

  BT::NodeStatus tick() override
  {
    if (!ctx_->trafficHold()) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("yield_pose", ctx_->yieldPose().value_or(geometry_msgs::msg::PoseStamped{}));
    return BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__IS_TRAFFIC_HOLD_HPP_
