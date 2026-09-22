// Decorator ChargingSession: 자식이 실행되는 동안만 charging/enable = true.
//
// 자식이 끝나면(SUCCESS/FAILURE) 또는 halt 되면(E-stop·반응형 게이트) 곧바로 false 로
// 되돌린다 → 충전 중 E-stop 이 걸려도 충전 신호가 켜진 채 남지 않는다. 재개하면(Charge 의
// charge_step) 다시 켠다.
// 자식 결과를 그대로 돌려준다.
#ifndef AMR_BEHAVIOR__BT_NODES__CHARGING_SESSION_HPP_
#define AMR_BEHAVIOR__BT_NODES__CHARGING_SESSION_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

namespace amr_behavior
{

class ChargingSession : public BT::DecoratorNode
{
public:
  ChargingSession(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::DecoratorNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts() {return {};}

  void halt() override
  {
    stop();
    BT::DecoratorNode::halt();
  }

private:
  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);
    if (!active_) {
      active_ = true;
      ctx_->setCharging(true);
    }
    const BT::NodeStatus child = child_node_->executeTick();
    if (child != BT::NodeStatus::RUNNING) {
      stop();
    }
    return child;
  }

  void stop()
  {
    if (active_) {
      active_ = false;
      ctx_->setCharging(false);
    }
  }

  ContextPtr ctx_;
  bool active_{false};
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__CHARGING_SESSION_HPP_
