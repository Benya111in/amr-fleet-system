// 분산 플릿 존 BT 노드 등록 (보너스 4 P3 · b4-wire, 헤더 전용).
//
// 실행기는 분산 모드(task_executor_node fleet.distributed)일 때만 bt_registry 의
// registerFleetAgentNodes() 로 이 노드들을 등록한다 — 중앙 모드 실행기는 에이전트 연결
// (구독 · 클라이언트)을 만들지 않는다. 노드들은 에이전트 연결 (ZoneAgentLink) 하나를 나눠 쓴다.
#ifndef AMR_BEHAVIOR__BT_NODES__ZONE_BT_NODES_HPP_
#define AMR_BEHAVIOR__BT_NODES__ZONE_BT_NODES_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "amr_behavior/bt_nodes/zone_acquire_chain.hpp"
#include "amr_behavior/bt_nodes/zone_agent_link.hpp"
#include "amr_behavior/bt_nodes/zone_guard.hpp"
#include "amr_behavior/bt_nodes/zone_path_clear.hpp"
#include "amr_behavior/bt_nodes/zone_release.hpp"
#include "amr_behavior/bt_nodes/zone_server_alive.hpp"
#include "amr_behavior/bt_nodes/zone_task_commit.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace amr_behavior
{

/// 존 노드 표. visitor.visit<Class>(ID) 를 노드마다 한 번 (등록 · 모델 · 시험이 같은 표를 쓴다).
template<class Visitor>
void forEachZoneNode(Visitor && visitor)
{
  visitor.template visit<AcquireZoneChain>("AcquireZoneChain");
  visitor.template visit<CommitTaskLock>("CommitTaskLock");
  visitor.template visit<IsFleetServerAlive>("IsFleetServerAlive");
  visitor.template visit<ReleaseZones>("ReleaseZones");
  visitor.template visit<ZoneGuard>("ZoneGuard");
  visitor.template visit<ZonePathClear>("ZonePathClear");
}

namespace detail
{
struct ZoneIdCollector
{
  std::vector<std::string> & ids;

  template<class T>
  void visit(const std::string & id) {ids.push_back(id);}
};

struct ZoneRegistrar
{
  BT::BehaviorTreeFactory & factory;
  ZoneAgentLinkPtr link;

  template<class T>
  void visit(const std::string & id)
  {
    auto l = link;
    factory.registerBuilder<T>(
      id, [l](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<T>(name, config, l);
      });
  }
};
}  // namespace detail

inline std::vector<std::string> zoneNodeIds()
{
  std::vector<std::string> ids;
  forEachZoneNode(detail::ZoneIdCollector{ids});
  return ids;
}

/// 존 노드 전부를 등록하고 공유 연결을 돌려준다.
inline ZoneAgentLinkPtr registerZoneNodes(
  BT::BehaviorTreeFactory & factory, const RosNodeParams & params,
  std::chrono::milliseconds max_state_age = std::chrono::milliseconds(3000))
{
  auto link = std::make_shared<ZoneAgentLink>(params, max_state_age);
  forEachZoneNode(detail::ZoneRegistrar{factory, link});
  return link;
}

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__ZONE_BT_NODES_HPP_
