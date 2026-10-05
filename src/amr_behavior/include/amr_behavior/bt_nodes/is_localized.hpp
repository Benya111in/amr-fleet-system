// Condition IsLocalized: localization/lost 가 false 이면 SUCCESS (kidnap_monitor_node 가 발행).
#ifndef AMR_BEHAVIOR__BT_NODES__IS_LOCALIZED_HPP_
#define AMR_BEHAVIOR__BT_NODES__IS_LOCALIZED_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_behavior
{

class IsLocalized : public BT::ConditionNode
{
public:
  IsLocalized(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    return ctx_->localizationLost() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__IS_LOCALIZED_HPP_
