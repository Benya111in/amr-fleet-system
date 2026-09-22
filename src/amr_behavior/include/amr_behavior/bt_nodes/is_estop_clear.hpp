// Condition IsEstopClear: safety/estop_active 가 false 이면 SUCCESS.
#ifndef AMR_BEHAVIOR__BT_NODES__IS_ESTOP_CLEAR_HPP_
#define AMR_BEHAVIOR__BT_NODES__IS_ESTOP_CLEAR_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_behavior
{

class IsEstopClear : public BT::ConditionNode
{
public:
  IsEstopClear(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    return ctx_->estopActive() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__IS_ESTOP_CLEAR_HPP_
