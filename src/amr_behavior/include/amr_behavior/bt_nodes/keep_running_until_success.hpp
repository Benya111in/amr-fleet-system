// Decorator KeepRunningUntilSuccess: 자식이 SUCCESS 를 낼 때까지 RUNNING (FAILURE 는 다음 tick
// 재평가).
//
// 조건을 "기다리는" 용도: Timeout 과 겹쳐 "timeout 안에 인식되면 SUCCESS, 아니면 FAILURE" 를
// 만든다. 내장 KeepRunningUntilFailure 의 대칭 (BT.CPP v3 에는 없다).
#ifndef AMR_BEHAVIOR__BT_NODES__KEEP_RUNNING_UNTIL_SUCCESS_HPP_
#define AMR_BEHAVIOR__BT_NODES__KEEP_RUNNING_UNTIL_SUCCESS_HPP_

#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace amr_behavior
{

class KeepRunningUntilSuccess : public BT::DecoratorNode
{
public:
  KeepRunningUntilSuccess(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);
    const BT::NodeStatus child = child_node_->executeTick();
    if (child == BT::NodeStatus::SUCCESS) {
      haltChild();
      return BT::NodeStatus::SUCCESS;
    }
    if (child == BT::NodeStatus::FAILURE) {
      haltChild();   // 다음 tick 에 처음부터 다시 평가
    }
    return BT::NodeStatus::RUNNING;
  }
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__KEEP_RUNNING_UNTIL_SUCCESS_HPP_
