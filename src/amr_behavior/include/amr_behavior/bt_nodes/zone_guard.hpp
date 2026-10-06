// Condition ZoneGuard: 사슬 자원이 모두 허가(held)돼 있으면 SUCCESS, 아니면 FAILURE.
//
// ReactiveSequence 안에서 NavigateToPose 앞에 두면 매 tick 허가를 다시 본다 — 허가가 없어지면
// (에이전트 상태가 오래됨 · 해제 · 재기동) 주행 goal 이 취소되고 멈춘다 (fail-closed 이중 안전).
#ifndef AMR_BEHAVIOR__BT_NODES__ZONE_GUARD_HPP_
#define AMR_BEHAVIOR__BT_NODES__ZONE_GUARD_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/zone_agent_link.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_behavior
{

class ZoneGuard : public BT::ConditionNode
{
public:
  ZoneGuard(const std::string & name, const BT::NodeConfiguration & config, ZoneAgentLinkPtr link)
  : BT::ConditionNode(name, config), link_(std::move(link)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("zones", "허가가 있어야 하는 자원")};
  }

  BT::NodeStatus tick() override
  {
    std::string text;
    if (!getInput("zones", text)) {
      return BT::NodeStatus::FAILURE;
    }
    return link_->holdsAll(parseResources(text)) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ZoneAgentLinkPtr link_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ZONE_GUARD_HPP_
