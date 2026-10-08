// Condition IsFleetServerAlive: 에이전트가 본 서버 하트비트가 ALIVE 면 SUCCESS (분산 모드,
// b4-wire).
//
// 분산 모드에서 중앙 교통 관리자(traffic_manager)의 traffic/hold 는 권고다 — 구역 안전은 정족수가
// 진다. traffic/hold 는 latched 라 서버가 죽으면 마지막 값이 남는다 (hold=true 로 죽으면 영영
// 선다).
// 그래서 분산 TrafficGate 는 이 노드가 SUCCESS 일 때만 hold 를 따른다. 상태가 없거나 오래되면
// FAILURE (서버 상태를 모름 — 구역 게이트 ZonePathClear 가 같은 이유로 이미 막는다).
#ifndef AMR_BEHAVIOR__BT_NODES__ZONE_SERVER_ALIVE_HPP_
#define AMR_BEHAVIOR__BT_NODES__ZONE_SERVER_ALIVE_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/zone_agent_link.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_behavior
{

class IsFleetServerAlive : public BT::ConditionNode
{
public:
  IsFleetServerAlive(
    const std::string & name, const BT::NodeConfiguration & config, ZoneAgentLinkPtr link)
  : BT::ConditionNode(name, config), link_(std::move(link)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    auto s = link_->freshState();
    return s && s->server_status == ZoneAgentLink::State::SERVER_ALIVE ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ZoneAgentLinkPtr link_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ZONE_SERVER_ALIVE_HPP_
