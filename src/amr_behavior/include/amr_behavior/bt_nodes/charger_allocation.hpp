// 충전소 할당 (로봇 5대 · 충전소 3개, warehouse.sdf C1~C3).
//
// Action SelectCharger   자기 점유가 유효하면 그대로, 아니면 선호 순서(로봇 번호만큼 돌린
//                        목록)의 첫 빈 충전소를 점유하고 staging 자세·도크 id 를 내보낸다.
//                        빈 곳이 없으면 FAILURE. 충전소가 바뀌면 progress(= charge_step) 를
//                        0 으로 되돌린다 (이전 충전소 기준 단계부터 재개하지 않도록).
//                        ReactiveSequence 첫 자식으로 매 tick 재평가한다: 동시에 같은 충전소를
//                        고른 로봇 중 늦은 쪽은 점유를 잃으면 다른 충전소를 골라 한 tick RUNNING
//                        (뒤의 충전 단계를 halt 해 새 목표로 다시 시작). progress ≥ lock_step
//                        (도킹 뒤)이면 점유를 잃지 않는다. 점유는 /fleet/charger_claims 로 알린다.
// Action ReleaseCharger  점유 해제. 항상 SUCCESS.
#ifndef AMR_BEHAVIOR__BT_NODES__CHARGER_ALLOCATION_HPP_
#define AMR_BEHAVIOR__BT_NODES__CHARGER_ALLOCATION_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace amr_behavior
{

class SelectCharger : public BT::ActionNodeBase
{
public:
  SelectCharger(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::ActionNodeBase(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("charger_goal", "충전소 staging 자세"),
      BT::OutputPort<std::string>("charger_dock_id", "충전소 도크 id"),
      BT::BidirectionalPort<int>("progress", "충전 단계 (charge_step). 충전소가 바뀌면 0 을 쓴다"),
      BT::InputPort<int>("lock_step", 2, "progress 가 이 이상이면 점유 유지 (도킹 뒤)"),
    };
  }

  BT::NodeStatus tick() override
  {
    const std::string before = ctx_->chargerClaim();
    const int progress = inputOr<int>(*this, "progress", 0);
    const bool lock = progress >= inputOr<int>(*this, "lock_step", 2);
    const auto dock = ctx_->selectCharger(lock);
    if (!dock) {
      ctx_->logWarn("SelectCharger: 빈 충전소 없음");
      return BT::NodeStatus::FAILURE;
    }
    setOutput("charger_goal", makePose(dock->frame_id, dock->x, dock->y, dock->yaw));
    setOutput("charger_dock_id", dock->id);
    if (dock->id == before) {
      return BT::NodeStatus::SUCCESS;
    }
    setOutput("progress", 0);
    if (before.empty()) {
      ctx_->logInfo("충전소 점유: " + dock->id);
      return BT::NodeStatus::SUCCESS;
    }
    // 먼저 점유한 로봇에게 양보: 새 충전소로 다시 시작하도록 이번 tick 은 RUNNING (뒤 형제 halt)
    ctx_->logWarn("충전소 " + before + " 를 먼저 점유한 로봇에게 양보 → " + dock->id);
    return BT::NodeStatus::RUNNING;
  }

  void halt() override {setStatus(BT::NodeStatus::IDLE);}

private:
  ContextPtr ctx_;
};

class ReleaseCharger : public BT::SyncActionNode
{
public:
  ReleaseCharger(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    ctx_->releaseCharger();
    return BT::NodeStatus::SUCCESS;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__CHARGER_ALLOCATION_HPP_
