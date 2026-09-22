// Action SetPhase: executor/phase 를 바꾼다 (같은 값이면 발행하지 않음). 항상 SUCCESS.
#ifndef AMR_BEHAVIOR__BT_NODES__SET_PHASE_HPP_
#define AMR_BEHAVIOR__BT_NODES__SET_PHASE_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace amr_behavior
{

class SetPhase : public BT::SyncActionNode
{
public:
  SetPhase(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>(
        "phase", "IDLE/MOVING/DOCKING/LOADING/UNLOADING/RETURNING/CHARGING/ERROR/RECOVERING")};
  }

  BT::NodeStatus tick() override
  {
    const std::string name = inputOr<std::string>(*this, "phase", "");
    if (!phase::isKnown(name)) {
      ctx_->logWarn("SetPhase: 알 수 없는 단계 '" + name + "' (그대로 발행)");
    }
    ctx_->setPhase(name);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__SET_PHASE_HPP_
