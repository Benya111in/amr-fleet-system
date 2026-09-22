// TTC 트리거 로직 + Nav2 BT 조건 플러그인(IsTTCBelowThreshold) 시험.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "amr_behavior/plugins/is_ttc_below_threshold_condition.hpp"
#include "amr_behavior/ttc/ttc_trigger.hpp"
#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

using amr_behavior::minTimeToCollision;
using amr_behavior::TtcFilter;
using amr_behavior::TtcSample;
using amr_behavior::TtcTrigger;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
}

TEST(TtcTrigger, MinTtcAppliesFilters)
{
  const std::vector<TtcSample> samples = {
    {kInf, true, 0.9}, {3.0, true, 0.9}, {1.0, false, 0.9}, {0.5, true, 0.1},
    {-1.0, true, 0.9}, {std::nan(""), true, 0.9}};
  TtcFilter f;
  EXPECT_DOUBLE_EQ(minTimeToCollision(samples, f), 0.5);   // 동적만, 신뢰도 무시
  f.min_confidence = 0.5;
  EXPECT_DOUBLE_EQ(minTimeToCollision(samples, f), 3.0);
  f.only_dynamic = false;
  EXPECT_DOUBLE_EQ(minTimeToCollision(samples, f), 1.0);
  EXPECT_TRUE(std::isinf(minTimeToCollision({}, f)));
}

TEST(TtcTrigger, ThresholdStaleAndCooldown)
{
  TtcTrigger trigger;
  TtcTrigger::Config cfg;
  TtcFilter filter;
  EXPECT_FALSE(trigger.evaluate(0.0, cfg, filter));   // 메시지 없음
  trigger.update({{1.5, true, 1.0}}, 10.0);
  EXPECT_TRUE(trigger.evaluate(10.1, cfg, filter));
  EXPECT_DOUBLE_EQ(trigger.lastMinTtc(), 1.5);
  EXPECT_FALSE(trigger.evaluate(10.7, cfg, filter));   // max_age 0.5 초과
  EXPECT_TRUE(std::isinf(trigger.lastMinTtc()));
  trigger.update({{2.5, true, 1.0}}, 11.0);
  EXPECT_FALSE(trigger.evaluate(11.0, cfg, filter));   // 임계 이상

  cfg.cooldown = 1.0;
  trigger.update({{0.8, true, 1.0}}, 20.0);
  EXPECT_TRUE(trigger.evaluate(20.0, cfg, filter));
  trigger.update({{0.8, true, 1.0}}, 20.4);
  EXPECT_FALSE(trigger.evaluate(20.4, cfg, filter));   // cooldown 중
  trigger.update({{0.8, true, 1.0}}, 21.1);
  EXPECT_TRUE(trigger.evaluate(21.1, cfg, filter));
}

class TtcPluginTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("ttc_plugin_test");
    pub_ = node_->create_publisher<amr_msgs::msg::TrackedObstacleArray>(
      "perception/tracked_obstacles", rclcpp::QoS(10));
    blackboard_ = BT::Blackboard::create();
    blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
  }

  void publish(double ttc, bool dynamic)
  {
    amr_msgs::msg::TrackedObstacleArray msg;
    amr_msgs::msg::TrackedObstacle o;
    o.time_to_collision = static_cast<float>(ttc);
    o.is_dynamic = dynamic;
    o.confidence = 0.9F;
    msg.obstacles.push_back(o);
    pub_->publish(msg);
  }

  /// 메시지가 전달될 때까지 tick 을 반복한다 (DDS 전달 지연 흡수).
  BT::NodeStatus tickUntil(BT::Tree & tree, BT::NodeStatus want, int max_tries = 200)
  {
    BT::NodeStatus s = BT::NodeStatus::IDLE;
    for (int i = 0; i < max_tries; ++i) {
      s = tree.tickRoot();
      if (s == want) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return s;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<amr_msgs::msg::TrackedObstacleArray>::SharedPtr pub_;
  BT::Blackboard::Ptr blackboard_;
};

TEST_F(TtcPluginTest, SuccessWhenTtcBelowThreshold)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<amr_behavior::IsTTCBelowThreshold>("IsTTCBelowThreshold");
  const std::string xml =
    R"(<root main_tree_to_execute="T"><BehaviorTree ID="T">
         <IsTTCBelowThreshold threshold="2.0" max_age="5.0" min_ttc="{min_ttc}"/>
       </BehaviorTree></root>)";
  auto tree = factory.createTreeFromText(xml, blackboard_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);   // 수신 전
  publish(1.2, true);
  EXPECT_EQ(tickUntil(tree, BT::NodeStatus::SUCCESS), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(blackboard_->get<double>("min_ttc"), 1.2, 1e-6);
  publish(1.2, false);   // 정적 트랙 → 무시
  EXPECT_EQ(tickUntil(tree, BT::NodeStatus::FAILURE), BT::NodeStatus::FAILURE);
  publish(kInf, true);
  EXPECT_EQ(tickUntil(tree, BT::NodeStatus::FAILURE), BT::NodeStatus::FAILURE);
}

TEST_F(TtcPluginTest, LoadsAsNav2PluginLibrary)
{
  // bt_navigator 와 같은 방식 (plugin_lib_names → registerFromPlugin)
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(AMR_TTC_PLUGIN_PATH);
  EXPECT_EQ(factory.manifests().count("IsTTCBelowThreshold"), 1U);
  const std::string xml =
    R"(<root main_tree_to_execute="T"><BehaviorTree ID="T">
         <IsTTCBelowThreshold threshold="3.0" max_age="5.0" only_dynamic="false"/>
       </BehaviorTree></root>)";
  auto tree = factory.createTreeFromText(xml, blackboard_);
  publish(2.0, false);
  EXPECT_EQ(tickUntil(tree, BT::NodeStatus::SUCCESS), BT::NodeStatus::SUCCESS);
}
