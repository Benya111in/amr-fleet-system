// 대기 구역 복귀 대기 상태 (작업 종료 뒤 복귀를 E-stop 재개 뒤에도 잃지 않도록).
//
// Condition IsReturnPending     작업이 COMPLETED/FAILED 로 끝나고 아직 대기 구역 복귀를
//                               마치지 않았으면 SUCCESS (실행기 컨텍스트가 종료 보고 때 설정).
// Action ClearReturnPending     복귀 시도를 마쳤다 (복귀는 최선 노력: 실패해도 해제).
#ifndef AMR_BEHAVIOR__BT_NODES__RETURN_PENDING_HPP_
#define AMR_BEHAVIOR__BT_NODES__RETURN_PENDING_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_behavior
{

class IsReturnPending : public BT::ConditionNode
{
public:
  IsReturnPending(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    return ctx_->returnPending() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ContextPtr ctx_;
};

class ClearReturnPending : public BT::SyncActionNode
{
public:
  ClearReturnPending(
    const std::string & name, const BT::NodeConfiguration & config,
    ContextPtr ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    ctx_->clearReturnPending();
    return BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__RETURN_PENDING_HPP_
