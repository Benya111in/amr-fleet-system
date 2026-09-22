// Action SetCharging: charging/enable 를 발행한다 (payload_manager_node 가 충전 이벤트로 처리).
// 항상 SUCCESS.
#ifndef AMR_BEHAVIOR__BT_NODES__SET_CHARGING_HPP_
#define AMR_BEHAVIOR__BT_NODES__SET_CHARGING_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace amr_behavior
{

class SetCharging : public BT::SyncActionNode
{
public:
  SetCharging(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("enable", true, "충전 시작(true) / 종료(false)")};
  }

  BT::NodeStatus tick() override
  {
    ctx_->setCharging(inputOr<bool>(*this, "enable", true));
    return BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__SET_CHARGING_HPP_
