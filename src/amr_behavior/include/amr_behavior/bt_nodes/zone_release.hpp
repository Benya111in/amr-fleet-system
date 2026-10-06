// Action ReleaseZones: 지나온 구역의 해제를 에이전트에 청한다 (보내면 SUCCESS).
//
// 실제 RELEASE 는 에이전트가 몸체가 구역 밖인지 확인한 뒤 낸다 (release_gate). 에이전트
// 서비스가 없으면 FAILURE — 해제되지 않은 구역은 다른 로봇에게 막혀 있다 (fail-closed).
#ifndef AMR_BEHAVIOR__BT_NODES__ZONE_RELEASE_HPP_
#define AMR_BEHAVIOR__BT_NODES__ZONE_RELEASE_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/zone_agent_link.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace amr_behavior
{

class ReleaseZones : public BT::SyncActionNode
{
public:
  ReleaseZones(
    const std::string & name, const BT::NodeConfiguration & config, ZoneAgentLinkPtr link)
  : BT::SyncActionNode(name, config), link_(std::move(link)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("zones", "", "해제할 자원 (비우면 보유한 구역 전부)")};
  }

  BT::NodeStatus tick() override
  {
    std::string text;
    getInput("zones", text);
    const auto token = link_->send(
      ZoneAgentLink::Command::Request::OP_RELEASE, parseResources(text));
    return token != 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ZoneAgentLinkPtr link_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ZONE_RELEASE_HPP_
