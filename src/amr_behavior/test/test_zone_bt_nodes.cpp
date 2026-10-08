// 분산 플릿 존 BT 노드 시험 (보너스 4 P3): 프로세스 안 가짜 에이전트(상태 발행 + 명령 서비스)로
// AcquireZoneChain · ZoneGuard · ReleaseZones · CommitTaskLock 과 zone_gates.xml 을 본다.
// (P3 에서는 소유 경로 때문에 src/bt_nodes/ 에 있었다 — b4-wire 에서 test/ 로 옮겼다, p3.md §7)
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "amr_behavior/bt_nodes/zone_bt_nodes.hpp"
#include "amr_msgs/msg/agent_state.hpp"
#include "amr_msgs/srv/zone_command.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "spin_thread.hpp"

using namespace std::chrono_literals;
using BT::NodeStatus;
using amr_msgs::msg::AgentState;
using amr_msgs::srv::ZoneCommand;

namespace
{

/// 가짜 에이전트: 명령을 기록하고, 시험이 정한 상태를 발행한다.
class FakeAgent
{
public:
  explicit FakeAgent(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    pub_ = node->create_publisher<AgentState>(
      "fleet_agent/state", rclcpp::QoS(1).reliable().transient_local());
    srv_ = node->create_service<ZoneCommand>(
      "fleet_agent/zone_command",
      [this](const std::shared_ptr<ZoneCommand::Request> req,
      std::shared_ptr<ZoneCommand::Response> resp) {
        std::lock_guard<std::mutex> lock(mutex_);
        ops_.emplace_back(req->op, req->resources);
        resp->accepted = accept;
        resp->message = accept ? "ok" : "mode ISOLATED";
      });
  }

  void publish(
    const std::vector<std::string> & held, const std::vector<std::string> & denied = {},
    std::chrono::milliseconds age = 0ms)
  {
    publishAt(held, node_->now() - rclcpp::Duration(age), denied);
  }

  void publishAt(
    const std::vector<std::string> & held, const rclcpp::Time & stamp,
    const std::vector<std::string> & denied = {})
  {
    AgentState s;
    s.stamp = stamp;
    s.robot_id = "amr_t";
    s.held = held;
    s.denied = denied;
    pub_->publish(s);
  }

  std::vector<std::pair<uint8_t, std::vector<std::string>>> ops()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return ops_;
  }

  bool sawOp(uint8_t op)
  {
    for (const auto & o : ops()) {
      if (o.first == op) {
        return true;
      }
    }
    return false;
  }

  std::atomic<bool> accept{true};

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<AgentState>::SharedPtr pub_;
  rclcpp::Service<ZoneCommand>::SharedPtr srv_;
  std::mutex mutex_;
  std::vector<std::pair<uint8_t, std::vector<std::string>>> ops_;
};

class ZoneNodesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  /// 노드 옵션 (sim 시계 시험은 use_sim_time 을 켠다)
  virtual rclcpp::NodeOptions options() const {return rclcpp::NodeOptions();}

  void SetUp() override
  {
    bt_node_ = std::make_shared<rclcpp::Node>("bt_exec", "/amr_t", options());
    agent_node_ = std::make_shared<rclcpp::Node>("fleet_agent", "/amr_t", options());
    agent_ = std::make_unique<FakeAgent>(agent_node_);
    exec_.add_node(bt_node_);
    exec_.add_node(agent_node_);
    spin_ = std::make_unique<amr_behavior_test::SpinThread>(exec_);
    params_ = amr_behavior::RosNodeParams::make(bt_node_);
    link_ = amr_behavior::registerZoneNodes(factory_, params_, 3000ms);
    factory_.registerSimpleAction(
      "NavigateToPose", [this](BT::TreeNode &) {
        ++nav_ticks_;
        return NodeStatus::SUCCESS;
      }, {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")});
    waitFor([this]() {return link_->serviceReady();}, 5s);
  }

  void TearDown() override
  {
    spin_.reset();
  }

  template<class Pred>
  bool waitFor(Pred pred, std::chrono::milliseconds limit)
  {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return pred();
  }

  NodeStatus tickUntilDone(BT::Tree & tree, std::chrono::milliseconds limit)
  {
    NodeStatus st = tree.tickRoot();
    const auto end = std::chrono::steady_clock::now() + limit;
    while (st == NodeStatus::RUNNING && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(20ms);
      st = tree.tickRoot();
    }
    return st;
  }

  BT::Tree tree(const std::string & body)
  {
    return factory_.createTreeFromText(
      "<root main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + body +
      "</BehaviorTree></root>");
  }

  rclcpp::Node::SharedPtr bt_node_;
  rclcpp::Node::SharedPtr agent_node_;
  std::unique_ptr<FakeAgent> agent_;
  rclcpp::executors::MultiThreadedExecutor exec_;
  std::unique_ptr<amr_behavior_test::SpinThread> spin_;
  amr_behavior::RosNodeParams params_;
  amr_behavior::ZoneAgentLinkPtr link_;
  BT::BehaviorTreeFactory factory_;
  std::atomic<int> nav_ticks_{0};
};

}  // namespace

TEST(ZoneParse, PrefixesAndDeduplicates)
{
  const auto r = amr_behavior::parseResources(" x_ab_1; zone:y ,task:t1;x_ab_1;");
  EXPECT_EQ(r, (std::vector<std::string>{"zone:x_ab_1", "zone:y", "task:t1"}));
  EXPECT_TRUE(amr_behavior::parseResources("").empty());
}

TEST_F(ZoneNodesTest, AcquireWaitsForWholeChainThenSucceeds)
{
  auto t = tree("<AcquireZoneChain zones=\"a;b\" priority=\"2\" timeout_ms=\"5000\"/>");
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor([this]() {return agent_->sawOp(ZoneCommand::Request::OP_ACQUIRE);}, 3s));
  EXPECT_EQ(agent_->ops().front().second, (std::vector<std::string>{"zone:a", "zone:b"}));
  agent_->publish({"zone:a"});                      // 사슬 일부만 — 진입하지 않는다
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);
  agent_->publish({"zone:a", "zone:b", "task:x"});
  EXPECT_EQ(tickUntilDone(t, 3s), NodeStatus::SUCCESS);
}

TEST_F(ZoneNodesTest, AcquireFailsWhenAgentRejects)
{
  agent_->accept = false;
  auto t = tree("<AcquireZoneChain zones=\"a\"/>");
  EXPECT_EQ(tickUntilDone(t, 3s), NodeStatus::FAILURE);
}

TEST_F(ZoneNodesTest, AcquireTimeoutAndHaltSendCancel)
{
  auto t = tree("<AcquireZoneChain zones=\"a\" timeout_ms=\"300\"/>");
  EXPECT_EQ(tickUntilDone(t, 3s), NodeStatus::FAILURE);
  ASSERT_TRUE(waitFor([this]() {return agent_->sawOp(ZoneCommand::Request::OP_CANCEL);}, 3s));

  auto t2 = tree("<AcquireZoneChain zones=\"c\"/>");
  const auto before = agent_->ops().size();
  EXPECT_EQ(t2.tickRoot(), NodeStatus::RUNNING);
  t2.haltTree();
  ASSERT_TRUE(waitFor([&]() {return agent_->ops().size() >= before + 2;}, 3s));
  EXPECT_EQ(agent_->ops().back().first, ZoneCommand::Request::OP_CANCEL);
}

TEST_F(ZoneNodesTest, GuardFollowsFreshHeldOnly)
{
  auto t = tree("<ZoneGuard zones=\"a;b\"/>");
  EXPECT_EQ(t.tickRoot(), NodeStatus::FAILURE);     // 상태 없음
  agent_->publish({"zone:a", "zone:b"});
  ASSERT_TRUE(waitFor([&]() {return t.tickRoot() == NodeStatus::SUCCESS;}, 3s));
  agent_->publish({"zone:a"});
  ASSERT_TRUE(waitFor([&]() {return t.tickRoot() == NodeStatus::FAILURE;}, 3s));
  agent_->publish({"zone:a", "zone:b"}, {}, 10s);   // 오래된 상태 = 허가 없음 (fail-closed)
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(t.tickRoot(), NodeStatus::FAILURE);
}

TEST_F(ZoneNodesTest, CommitTaskLockSucceedsOnHeldAndFailsOnDenied)
{
  auto t = tree("<CommitTaskLock task_id=\"t1\" timeout_ms=\"5000\"/>");
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor([this]() {return agent_->sawOp(ZoneCommand::Request::OP_COMMIT_TASK);}, 3s));
  EXPECT_EQ(agent_->ops().back().second, (std::vector<std::string>{"task:t1"}));
  agent_->publish({"task:t1"});
  EXPECT_EQ(tickUntilDone(t, 3s), NodeStatus::SUCCESS);

  auto t2 = tree("<CommitTaskLock task_id=\"t2\"/>");
  EXPECT_EQ(t2.tickRoot(), NodeStatus::RUNNING);
  agent_->publish({"task:t1"}, {"task:t2"});
  EXPECT_EQ(tickUntilDone(t2, 3s), NodeStatus::FAILURE);
}

TEST_F(ZoneNodesTest, ReleaseSendsCommand)
{
  auto t = tree("<ReleaseZones zones=\"a\"/>");
  EXPECT_EQ(t.tickRoot(), NodeStatus::SUCCESS);
  ASSERT_TRUE(waitFor([this]() {return agent_->sawOp(ZoneCommand::Request::OP_RELEASE);}, 3s));
  EXPECT_EQ(agent_->ops().back().second, (std::vector<std::string>{"zone:a"}));
}

TEST_F(ZoneNodesTest, ZoneGatesSubtreeStopsWithoutPermissionAndPassesWithIt)
{
  const std::string xml = AMR_BEHAVIOR_SOURCE_DIR "/behavior_trees/subtrees/zone_gates.xml";
  factory_.registerBehaviorTreeFromFile(xml);
  auto bb = BT::Blackboard::create();
  bb->set("zones", std::string("x_ab_1"));
  bb->set("priority", 0);
  bb->set("zone_timeout_ms", 2000U);
  bb->set("goal", geometry_msgs::msg::PoseStamped{});
  bb->set("decision_goal", geometry_msgs::msg::PoseStamped{});
  auto t = factory_.createTree("ZoneMoveTo", bb);
  EXPECT_EQ(tickUntilDone(t, 4s), NodeStatus::FAILURE);   // 허가가 오지 않음 → 진입 안 함
  EXPECT_EQ(nav_ticks_.load(), 1);                         // 결정점까지만 갔다
  agent_->publish({"zone:x_ab_1"});
  auto t2 = factory_.createTree("ZoneMoveTo", bb);
  ASSERT_TRUE(waitFor([this]() {return link_->holdsAll({"zone:x_ab_1"});}, 3s));
  EXPECT_EQ(tickUntilDone(t2, 4s), NodeStatus::SUCCESS);
  EXPECT_EQ(nav_ticks_.load(), 3);
  EXPECT_TRUE(waitFor([this]() {return agent_->sawOp(ZoneCommand::Request::OP_RELEASE);}, 3s));
}

TEST_F(ZoneNodesTest, FutureStampBeyondToleranceIsNotFresh)
{
  // P3 리뷰 §8-13: 미래 stamp(sim 리셋 · 시계 혼용)는 age < 0 이라 예전에는 "언제나 신선"
  // 이었다 (fail-open)
  auto t = tree("<ZoneGuard zones=\"a\"/>");
  agent_->publishAt({"zone:a"}, agent_node_->now() + rclcpp::Duration(10s));
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(t.tickRoot(), NodeStatus::FAILURE);
  EXPECT_FALSE(link_->freshState());
  // 허용치(0.5 s) 안의 작은 미래는 정상 (프로세스마다 /clock 받는 시각이 조금 다르다)
  agent_->publishAt({"zone:a"}, agent_node_->now() + rclcpp::Duration(200ms));
  ASSERT_TRUE(waitFor([&]() {return t.tickRoot() == NodeStatus::SUCCESS;}, 3s));
  agent_->publishAt({"zone:a"}, agent_node_->now() + rclcpp::Duration(900ms));
  ASSERT_TRUE(waitFor([&]() {return t.tickRoot() == NodeStatus::FAILURE;}, 3s));
}

namespace
{

/// use_sim_time 노드 + 시험이 손으로 내는 /clock (sim 이 멈춘 동안 벽시계만 흐르게 한다)
class ZoneNodesSimTest : public ZoneNodesTest
{
protected:
  rclcpp::NodeOptions options() const override
  {
    return rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  }

  void SetUp() override
  {
    clock_node_ = std::make_shared<rclcpp::Node>("sim_clock_src");
    clock_pub_ = clock_node_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(10).reliable());
    ZoneNodesTest::SetUp();
  }

  /// /clock 을 t [s] 로 두고 실행기 노드 시계가 따라올 때까지 기다린다
  void setSim(double t)
  {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
    const auto want = static_cast<int64_t>(t * 1e9);
    ASSERT_TRUE(
      waitFor(
        [&]() {
          clock_pub_->publish(msg);
          return bt_node_->now().nanoseconds() == want &&
          agent_node_->now().nanoseconds() == want;
        }, 5s)) << "sim 시계가 " << t << " 에 오지 않음";
  }

  /// sim 이 멈춘 채 벽시계 wall 동안 tick — 줄곧 RUNNING 이어야 한다
  void expectRunningWhileSimFrozen(BT::Tree & tree, std::chrono::milliseconds wall)
  {
    const auto end = std::chrono::steady_clock::now() + wall;
    while (std::chrono::steady_clock::now() < end) {
      ASSERT_EQ(tree.tickRoot(), NodeStatus::RUNNING);
      std::this_thread::sleep_for(50ms);
    }
  }

  rclcpp::Node::SharedPtr clock_node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
};

}  // namespace

TEST_F(ZoneNodesSimTest, CommitTimeoutFollowsSimClockNotWallClock)
{
  // P3 리뷰 §8-14: RTF 0.25 면 벽시계 시간 초과는 sim 기준 4 배 빨리 끝난다 — 노드 시계로 잰다.
  // /clock 전(시각 0)에 시작해도 첫 sim 시각부터 잰다 (0 → 100 s 도약을 경과로 세지 않는다)
  auto t = tree("<CommitTaskLock task_id=\"t9\" timeout_ms=\"500\"/>");
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);
  setSim(100.0);
  expectRunningWhileSimFrozen(t, 1200ms);       // 벽시계 1.2 s > 0.5 s, sim 0 s
  setSim(100.3);
  expectRunningWhileSimFrozen(t, 300ms);        // sim 0.3 s < 0.5 s
  setSim(100.6);
  ASSERT_TRUE(waitFor([&]() {return t.tickRoot() == NodeStatus::FAILURE;}, 3s));
}

TEST_F(ZoneNodesSimTest, AcquireTimeoutFollowsSimClockAndCancels)
{
  setSim(50.0);
  auto t = tree("<AcquireZoneChain zones=\"a\" timeout_ms=\"400\"/>");
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);
  ASSERT_TRUE(waitFor([this]() {return agent_->sawOp(ZoneCommand::Request::OP_ACQUIRE);}, 3s));
  expectRunningWhileSimFrozen(t, 1000ms);
  EXPECT_FALSE(agent_->sawOp(ZoneCommand::Request::OP_CANCEL));
  setSim(50.5);
  ASSERT_TRUE(waitFor([&]() {return t.tickRoot() == NodeStatus::FAILURE;}, 3s));
  EXPECT_TRUE(waitFor([this]() {return agent_->sawOp(ZoneCommand::Request::OP_CANCEL);}, 3s));
}

TEST_F(ZoneNodesSimTest, SimClockGoingBackRestartsTheStopwatch)
{
  amr_behavior::SimStopwatch w;
  const auto at = [](double s) {return rclcpp::Time(static_cast<int64_t>(s * 1e9), RCL_ROS_TIME);};
  w.start(at(0.0));                               // /clock 전
  EXPECT_DOUBLE_EQ(w.elapsedS(at(0.0)), 0.0);
  EXPECT_DOUBLE_EQ(w.elapsedS(at(100.0)), 0.0);   // 첫 sim 시각부터
  EXPECT_NEAR(w.elapsedS(at(101.5)), 1.5, 1e-9);
  EXPECT_DOUBLE_EQ(w.elapsedS(at(10.0)), 0.0);    // sim 리셋 — 다시 잰다
  EXPECT_NEAR(w.elapsedS(at(10.25)), 0.25, 1e-9);
}
