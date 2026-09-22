// Condition IsObjectDetected: 최근 max_age 초 안의 perception/detected_objects 에
// class_name 이고 confidence ≥ min_confidence, 거리 ≤ max_distance 인 객체가 있으면 SUCCESS.
#ifndef AMR_BEHAVIOR__BT_NODES__IS_OBJECT_DETECTED_HPP_
#define AMR_BEHAVIOR__BT_NODES__IS_OBJECT_DETECTED_HPP_

#include <string>
#include <utility>

#include "amr_behavior/bt_nodes/bt_utils.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace amr_behavior
{

class IsObjectDetected : public BT::ConditionNode
{
public:
  IsObjectDetected(const std::string & name, const BT::NodeConfiguration & config, ContextPtr ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

  static BT::PortsList providedPorts()
  {
    const ObjectQuery d;
    return {
      BT::InputPort<std::string>("class_name", d.class_name, "YOLO 클래스 이름 (대소문자 무시)"),
      BT::InputPort<double>("max_distance", d.max_distance, "base_link 기준 거리 상한 [m]"),
      BT::InputPort<double>("min_confidence", d.min_confidence, "신뢰도 하한"),
      BT::InputPort<double>("max_age", d.max_age, "인식 결과 유효 시간 [s]"),
    };
  }

  BT::NodeStatus tick() override
  {
    const ObjectQuery d;
    ObjectQuery q;
    q.class_name = inputOr<std::string>(*this, "class_name", d.class_name);
    q.max_distance = inputOr<double>(*this, "max_distance", d.max_distance);
    q.min_confidence = inputOr<double>(*this, "min_confidence", d.min_confidence);
    q.max_age = inputOr<double>(*this, "max_age", d.max_age);
    return ctx_->objectDetected(q) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  ContextPtr ctx_;
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__BT_NODES__IS_OBJECT_DETECTED_HPP_
