// Action SimulateLoad / SimulateUnload: 명세 7장 "가상 적재/하역 이벤트".
//
// 물품 표(robot_params.yaml payload.<type>)의 load_time 동안 RUNNING 으로 기다린 뒤
//   적재: payload/attach = item_type, payload/mass = 질량 [kg]
//   하역: payload/attach = "",        payload/mass = 0
// 을 발행하고 SUCCESS (계약 C5, 둘 다 latched). payload/mass 는 amr_navigation 의
// velocity_profiler·DWA·pure pursuit 가 가속 한계 스케일에 쓴다. Gazebo 물리 질량 변화는
// amr_simulation payload_manager_node(C5, 이 패키지 밖)가 두 토픽을 받아 반영한다 — 이 노드는
// 이벤트만 낸다.
// 적재는 출처(origin_goal·origin_dock_id)를 함께 기록한다 → 하역 실패 시 되돌려 놓을 곳.
// 이미 물품이 실려 있으면 적재하지 않고 FAILURE (물품 위에 싣지 않는다).
// 도중에 halt(E-stop 등) 되면 이벤트를 내지 않는다(재개 시 처음부터).
#ifndef AMR_BEHAVIOR__BT_NODES__SIMULATE_PAYLOAD_HPP_
#define AMR_BEHAVIOR__BT_NODES__SIMULATE_PAYLOAD_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
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
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "origin_goal", "적재 도크 staging 자세 (되돌려 놓을 곳, 적재만)"),
      BT::InputPort<std::string>("origin_dock_id", "", "적재 도크 id (적재만)"),
    };
  }

  double duration() const {return duration_;}

protected:
  BT::NodeStatus onStart() override
  {
    item_type_ = inputOr<std::string>(*this, "item_type", "");
    if (attach_ && !ctx_->attachedPayload().empty()) {
      ctx_->logWarn(
        "SimulateLoad: 이미 '" + ctx_->attachedPayload() + "' 이 실려 있다 → 적재 거부");
      return BT::NodeStatus::FAILURE;
    }
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
      PayloadOrigin origin;
      getInput("origin_goal", origin.goal);
      origin.dock_id = inputOr<std::string>(*this, "origin_dock_id", "");
      ctx_->attachPayload(item_type_, mass_, origin);
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
