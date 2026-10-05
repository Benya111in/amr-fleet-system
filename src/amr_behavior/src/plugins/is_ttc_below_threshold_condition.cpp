// Nav2 BT 조건 플러그인 IsTTCBelowThreshold 구현.
#include "amr_behavior/plugins/is_ttc_below_threshold_condition.hpp"

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace amr_behavior
{

IsTTCBelowThreshold::IsTTCBelowThreshold(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  std::string topic = "perception/tracked_obstacles";
  getInput("tracked_obstacles_topic", topic);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  // components.md §5.4: 10 Hz, reliable
  sub_ = node_->create_subscription<amr_msgs::msg::TrackedObstacleArray>(
    topic, rclcpp::QoS(10),
    [this](const amr_msgs::msg::TrackedObstacleArray::SharedPtr msg) {onObstacles(msg);},
    options);
}

void IsTTCBelowThreshold::onObstacles(const amr_msgs::msg::TrackedObstacleArray::SharedPtr msg)
{
  std::vector<TtcSample> samples;
  samples.reserve(msg->obstacles.size());
  for (const auto & o : msg->obstacles) {
    samples.push_back(
      TtcSample{static_cast<double>(o.time_to_collision), o.is_dynamic,
        static_cast<double>(o.confidence)});
  }
  trigger_.update(samples, node_->now().seconds());
}

BT::NodeStatus IsTTCBelowThreshold::tick()
{
  callback_group_executor_.spin_some();
  TtcTrigger::Config config;
  TtcFilter filter;
  getInput("threshold", config.threshold);
  getInput("max_age", config.max_age);
  getInput("cooldown", config.cooldown);
  getInput("only_dynamic", filter.only_dynamic);
  getInput("min_confidence", filter.min_confidence);
  const bool below = trigger_.evaluate(node_->now().seconds(), config, filter);
  setOutput("min_ttc", trigger_.lastMinTtc());
  return below ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace amr_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<amr_behavior::IsTTCBelowThreshold>("IsTTCBelowThreshold");
}
