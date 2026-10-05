// Condition IsDockMarkerVisible: perception/dock_marker_pose 를 max_age 초 안에 받았으면 SUCCESS.
#ifndef AMR_BEHAVIOR__BT_NODES__IS_DOCK_MARKER_VISIBLE_HPP_
#define AMR_BEHAVIOR__BT_NODES__IS_DOCK_MARKER_VISIBLE_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_behavior
{

class IsDockMarkerVisible : public BT::ConditionNode
{
public:
  IsDockMarkerVisible(
    const std::string & name, const BT::NodeConfiguration & config,
    ContextPtr ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_age", 1.0, "마커 관측 유효 시간 [s]")};
  }

  BT::NodeStatus tick() override
  {
    const double max_age = inputOr<double>(*this, "max_age", 1.0);
    return ctx_->dockMarkerAge() <= max_age ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__IS_DOCK_MARKER_VISIBLE_HPP_
