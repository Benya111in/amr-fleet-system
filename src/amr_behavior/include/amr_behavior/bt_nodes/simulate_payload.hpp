// Action SimulateLoad / SimulateUnload: 명세 7장 "가상 적재/하역 이벤트".
//
// 물품 표(robot_params.yaml payload.<type>)의 load_time 동안 RUNNING 으로 기다린 뒤
//   적재: payload/attach = item_type, payload/mass = 질량 [kg]
//   하역: payload/attach = "",        payload/mass = 0
// 을 발행하고 SUCCESS. payload_manager_node(amr_simulation)가 attach 를 받아 Gazebo 에 박스를 붙여
// 질량 변화를 동역학에 반영한다. 도중에 halt(E-stop 등) 되면 이벤트를 내지 않는다(재개 시
// 처음부터).
#ifndef AMR_BEHAVIOR__BT_NODES__SIMULATE_PAYLOAD_HPP_
#define AMR_BEHAVIOR__BT_NODES__SIMULATE_PAYLOAD_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace amr_behavior
{

/// 적재/하역 공통 구현. attach = true 면 적재.
class PayloadTransferAction : public BT::StatefulActionNode
{
public:
  PayloadTransferAction(
    const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx,
    bool attach)
  : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)), attach_(attach) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("item_type", "small / medium / large"),
      BT::InputPort<double>("item_mass", -1.0, "질량 [kg], 0 이하 = 물품 표 값"),
      BT::InputPort<double>("load_time", -1.0, "소요 시간 [s], 음수 = 물품 표 값"),
    };
  }

  double duration() const {return duration_;}

protected:
  BT::NodeStatus onStart() override
  {
    item_type_ = inputOr<std::string>(*this, "item_type", "");
    const auto spec = ctx_->payloadSpec(item_type_);
    if (!spec) {
      ctx_->logWarn(registrationName() + ": 물품 표에 없는 종류 '" + item_type_ + "' (소요 0 s)");
    }
    const double mass_in = inputOr<double>(*this, "item_mass", -1.0);
    const double time_in = inputOr<double>(*this, "load_time", -1.0);
    mass_ = mass_in > 0.0 ? mass_in : (spec ? spec->mass : 0.0);
    duration_ = time_in >= 0.0 ? time_in : (spec ? spec->load_time : 0.0);
    start_ = ctx_->now();
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    if (ctx_->now() - start_ < duration_) {
      return BT::NodeStatus::RUNNING;
    }
    if (attach_) {
      ctx_->attachPayload(item_type_, mass_);
    } else {
      ctx_->detachPayload();
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

  ContextPtr ctx_;
  bool attach_;
  std::string item_type_;
  double mass_{0.0};
  double duration_{0.0};
  double start_{0.0};
};

class SimulateLoad : public PayloadTransferAction
{
public:
  SimulateLoad(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : PayloadTransferAction(name, config, std::move(ctx), true) {}
};

class SimulateUnload : public PayloadTransferAction
{
public:
  SimulateUnload(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : PayloadTransferAction(name, config, std::move(ctx), false) {}
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__SIMULATE_PAYLOAD_HPP_
