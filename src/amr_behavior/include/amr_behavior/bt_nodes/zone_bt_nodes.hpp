// 분산 플릿 존 BT 노드 등록 (보너스 4 P3, 헤더 전용).
//
// 실행기에 붙이려면 registerRosNodes() 다음에 한 줄을 더하면 된다 (bt_registry.cpp 소유자 몫):
//   amr_behavior::registerZoneNodes(factory, params);
// 노드들은 에이전트 연결 (ZoneAgentLink) 하나를 나눠 쓴다.
#ifndef AMR_BEHAVIOR__BT_NODES__ZONE_BT_NODES_HPP_
#define AMR_BEHAVIOR__BT_NODES__ZONE_BT_NODES_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "amr_behavior/bt_nodes/zone_acquire_chain.hpp"
#include "amr_behavior/bt_nodes/zone_agent_link.hpp"
#include "amr_behavior/bt_nodes/zone_guard.hpp"
#include "amr_behavior/bt_nodes/zone_release.hpp"
#include "amr_behavior/bt_nodes/zone_task_commit.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace amr_behavior
{

inline std::vector<std::string> zoneNodeIds()
{
  return {"AcquireZoneChain", "CommitTaskLock", "ReleaseZones", "ZoneGuard"};
}

/// 연결(link)을 생성자 셋째 인자로 넘기는 빌더를 등록한다.
template<class T>
void registerZoneNode(
  BT::BehaviorTreeFactory & factory, const std::string & id, const ZoneAgentLinkPtr & link)
{
  factory.registerBuilder<T>(
    id, [link](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, link);
    });
}

/// 존 노드 넷을 등록하고 공유 연결을 돌려준다.
inline ZoneAgentLinkPtr registerZoneNodes(
  BT::BehaviorTreeFactory & factory, const RosNodeParams & params,
  std::chrono::milliseconds max_state_age = std::chrono::milliseconds(3000))
{
  auto link = std::make_shared<ZoneAgentLink>(params, max_state_age);
  registerZoneNode<AcquireZoneChain>(factory, "AcquireZoneChain", link);
  registerZoneNode<ZoneGuard>(factory, "ZoneGuard", link);
  registerZoneNode<ReleaseZones>(factory, "ReleaseZones", link);
  registerZoneNode<CommitTaskLock>(factory, "CommitTaskLock", link);
  return link;
}

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ZONE_BT_NODES_HPP_
