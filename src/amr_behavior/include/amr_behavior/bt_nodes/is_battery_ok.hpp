// Condition IsBatteryOk: 잔량이 min_percent 이상이면 SUCCESS (battery_state 미수신/미측정이면
// SUCCESS).
#ifndef AMR_BEHAVIOR__BT_NODES__IS_BATTERY_OK_HPP_
#define AMR_BEHAVIOR__BT_NODES__IS_BATTERY_OK_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_behavior
{

class IsBatteryOk : public BT::ConditionNode
{
public:
  IsBatteryOk(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("min_percent", 20.0, "허용 최소 잔량 [%]")};
  }

  BT::NodeStatus tick() override
  {
    const double min_percent = inputOr<double>(*this, "min_percent", 20.0);
    const auto level = ctx_->battery();
    // 미측정은 "정상" 으로 본다: 배터리 모델이 없는 시뮬레이션에서 작업을 막지 않기 위해.
    return (!level || *level >= min_percent) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__IS_BATTERY_OK_HPP_
