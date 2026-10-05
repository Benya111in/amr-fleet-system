// Decorator SetFailReason: 자식이 FAILURE 면 fail_reason 출력 포트에 reason 을 쓰고 FAILURE.
// 단계별 실패 사유(nav_failed / perception_failed / dock_failed …)를 ReportTaskStatus 로 넘긴다.
#ifndef AMR_BEHAVIOR__BT_NODES__SET_FAIL_REASON_HPP_
#define AMR_BEHAVIOR__BT_NODES__SET_FAIL_REASON_HPP_

#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace amr_behavior
{

class SetFailReason : public BT::DecoratorNode
{
public:
  SetFailReason(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("reason", "실패 사유 코드"),
      BT::OutputPort<std::string>("fail_reason", "사유를 쓸 블랙보드 키"),
    };
  }

private:
  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);
    const BT::NodeStatus child = child_node_->executeTick();
    if (child == BT::NodeStatus::FAILURE) {
      std::string reason = "failed";
      getInput("reason", reason);
      setOutput("fail_reason", reason);
    }
    return child;
  }
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__SET_FAIL_REASON_HPP_
