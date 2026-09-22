// Decorator TaskDeadline: 자식이 msec 안에 끝나지 않으면 halt 하고
// fail_reason = task_timeout 으로 FAILURE.
//
// BT.CPP v3 Timeout 과 달리 (1) 시각은 실행기 시계(sim time)를 쓰고 — RTF 0.3 에서
// 벽시계 15 분은 sim 4.5 분이다 — (2) 실패 사유를 스스로 써서 "어느 단계에서 멈췄는지"
// 가 아니라 "작업이 상한을 넘었다" 를 구분한다.
// 필요성 (통합 시나리오 11 실측): 위치 상실로 Nav2 가 취소되자 실행기가 MOVING 에
// 1340 s 동안 머물러 작업이 끝나지도 실패하지도 않았고, 플릿은 그 로봇을 계속 점유로 봤다.
#ifndef AMR_BEHAVIOR__BT_NODES__TASK_DEADLINE_HPP_
#define AMR_BEHAVIOR__BT_NODES__TASK_DEADLINE_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

namespace amr_behavior
{

class TaskDeadline : public BT::DecoratorNode
{
public:
  TaskDeadline(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::DecoratorNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<unsigned>("msec", 900000u, "작업 상한 [ms] (0 = 끔), 실행기 시계 기준"),
      BT::OutputPort<std::string>("fail_reason", "시간 초과 시 쓸 사유 키"),
    };
  }

private:
  BT::NodeStatus tick() override
  {
    if (status() == BT::NodeStatus::IDLE) {
      start_ = ctx_->now();
    }
    setStatus(BT::NodeStatus::RUNNING);
    const unsigned limit_ms = inputOr<unsigned>(*this, "msec", 900000u);
    if (limit_ms > 0 && ctx_->now() - start_ > static_cast<double>(limit_ms) / 1000.0) {
      haltChild();
      setOutput("fail_reason", std::string("task_timeout"));
      ctx_->logWarn(
        "작업이 상한 " + std::to_string(limit_ms / 1000) + " s 를 넘겼다 → 실패 처리");
      return BT::NodeStatus::FAILURE;
    }
    return child_node_->executeTick();
  }

  void halt() override
  {
    BT::DecoratorNode::halt();   // IDLE 로 돌아가면 다음 시작에서 시계를 다시 잡는다
  }

  ContextPtr ctx_;
  double start_{0.0};
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__TASK_DEADLINE_HPP_
