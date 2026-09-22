// 실린 물품 상태 노드 (하역 실패 뒤 복구, 명세 4.8 "3회 실패 시 에러 보고 및 대체 작업").
//
// Condition IsPayloadEmpty     물품이 실려 있지 않으면 SUCCESS.
// Condition IsPayloadStranded  작업이 끝났는데(FAILED) 물품이 실려 있고 되돌려 놓기를 아직 시도하지
//                              않았으면 SUCCESS + 출처(적재 도크 staging·도크 id·종류)를 내보낸다.
// Action MarkPayloadReturnAttempted  되돌려 놓기 시도를 끝냈다고 기록 (실패해도 다시 돌지 않게).
//                              물품이 남아 있으면 실행기는 새 작업을 "blocked:payload" 로 거절하고
//                              Idle 에서 ERROR 로 머문다 (clear_payload 서비스로 해제).
#ifndef AMR_BEHAVIOR__BT_NODES__PAYLOAD_STATE_HPP_
#define AMR_BEHAVIOR__BT_NODES__PAYLOAD_STATE_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_behavior
{

class IsPayloadEmpty : public BT::ConditionNode
{
public:
  IsPayloadEmpty(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    return ctx_->attachedPayload().empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ContextPtr ctx_;
};

class IsPayloadStranded : public BT::ConditionNode
{
public:
  IsPayloadStranded(
    const std::string & name, const BT::NodeConfiguration & config,
    ContextPtr ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("origin_goal", "적재 도크 staging 자세"),
      BT::OutputPort<std::string>("origin_dock_id", "적재 도크 id (\"\" = 도킹 없이 적재)"),
      BT::OutputPort<std::string>("item_type", "실린 물품 종류"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto origin = ctx_->strandedPayload();
    if (!origin) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("origin_goal", origin->goal);
    setOutput("origin_dock_id", origin->dock_id);
    setOutput("item_type", origin->item_type);
    return BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

class MarkPayloadReturnAttempted : public BT::SyncActionNode
{
public:
  MarkPayloadReturnAttempted(
    const std::string & name, const BT::NodeConfiguration & config,
    ContextPtr ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    ctx_->markPayloadReturnAttempted();
    if (!ctx_->attachedPayload().empty()) {
      ctx_->logWarn("물품을 되돌려 놓지 못했다 → 물품을 실은 채 작업 거절 (clear_payload 로 해제)");
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__PAYLOAD_STATE_HPP_
