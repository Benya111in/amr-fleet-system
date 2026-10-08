// Condition ZonePathClear: 에이전트 경로 게이트가 "지금 허가 없이 움직이면 안 되는 구역" 을
// 알리지 않으면 SUCCESS (분산 모드, 보너스 4 b4-wire — fleet-distributed/p3.md §10.1).
//
// 에이전트(fleet_agent, path_gate.enabled)는 자기 plan · 자세로 다음 구역 사슬을 미리 요청하고,
// 사슬 첫 입구가 결정점(요청 거리) 안인데 사슬 전부를 쥐지 못했으면 그 미보유 구역을 상태의
// requested 에 싣는다. 이 노드는 신선한 상태(ZoneAgentLink::freshState)에서 requested 의 zone:* 가
// 모두 held 일 때만 SUCCESS — 상태가 없거나 오래되면 FAILURE (에이전트가 죽으면 움직이지 않는다,
// fail-closed). TrafficGate 의 ReactiveSequence 맨 앞에 두면 매 tick 다시 보고, 막히면 주행 goal 이
// 취소된다.
#ifndef AMR_BEHAVIOR__BT_NODES__ZONE_PATH_CLEAR_HPP_
#define AMR_BEHAVIOR__BT_NODES__ZONE_PATH_CLEAR_HPP_

#include <algorithm>
#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/zone_agent_link.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_behavior
{

class ZonePathClear : public BT::ConditionNode
{
public:
  ZonePathClear(
    const std::string & name, const BT::NodeConfiguration & config, ZoneAgentLinkPtr link)
  : BT::ConditionNode(name, config), link_(std::move(link)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    auto s = link_->freshState();
    if (!s) {
      return BT::NodeStatus::FAILURE;
    }
    for (const auto & r : s->requested) {
      if (r.rfind("zone:", 0) == 0 &&
        std::find(s->held.begin(), s->held.end(), r) == s->held.end())
      {
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  ZoneAgentLinkPtr link_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ZONE_PATH_CLEAR_HPP_
