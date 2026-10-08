// 분산 플릿 모드 작업 트리 (task_executor_distributed.xml, 보너스 4 b4-wire) 통합 시험.
// ROS 액션/서비스 노드와 존 노드를 스텁으로 주입한다 (실제 에이전트 연결은 test_zone_bt_nodes ·
// launch 시험이 본다).
// - 정상: 중앙 트리와 같은 단계 순서, 적재 도킹 뒤 Load 앞에서 작업 ID 잠금 확정 한 번.
// - 잠금 실패: 적재하지 않고 task_lock_failed 로 실패 보고 → 대기 구역 복귀.
// - 구역 게이트(ZonePathClear)가 막으면 주행 goal 취소 · 재시작 안 함, 풀리면 다시 간다.
//   도킹 · 양보 이동도 막는다.
// - 중앙 traffic/hold 는 서버가 살아 있을 때만 따른다 (IsFleetServerAlive).
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
#include "amr_behavior/bt_nodes/zone_bt_nodes.hpp"
#include "amr_behavior/bt_registry.hpp"
#include "amr_behavior/executor_context.hpp"
#include "amr_behavior/task_tree.hpp"
#include "bt_test_utils.hpp"

using amr_behavior::DockSpec;
using amr_behavior::ExecutorContext;
using amr_behavior::ExecutorHooks;
using amr_behavior::makePose;
using amr_behavior_test::StubScript;
using amr_msgs::msg::Task;
using BT::NodeStatus;

namespace
{

const char kDistXml[] = AMR_BEHAVIOR_SOURCE_DIR "/behavior_trees/task_executor_distributed.xml";

/// CommitTaskLock 스텁: task_id 를 기록하고 대본대로 끝낸다.
struct CommitScript
{
  std::vector<std::string> task_ids;
  std::vector<size_t> at_phase;          ///< 시작할 때까지 나온 단계 수
  NodeStatus outcome{NodeStatus::SUCCESS};
  int running_ticks{2};
};

class StubCommit : public BT::StatefulActionNode
{
public:
  StubCommit(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<CommitScript> s, std::function<size_t()> phases)
  : BT::StatefulActionNode(name, config), s_(std::move(s)), phases_(std::move(phases)) {}

  static BT::PortsList providedPorts() {return amr_behavior::CommitTaskLock::providedPorts();}

  NodeStatus onStart() override
  {
    std::string id;
    getInput("task_id", id);
    s_->task_ids.push_back(id);
    s_->at_phase.push_back(phases_());
    left_ = s_->running_ticks;
    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() override {return --left_ > 0 ? NodeStatus::RUNNING : s_->outcome;}
  void onHalted() override {}

private:
  std::shared_ptr<CommitScript> s_;
  std::function<size_t()> phases_;
  int left_{0};
};

class DistributedTreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ExecutorHooks hooks;
    hooks.publish_phase = [this](const std::string & p) {phases_.push_back(p);};
    hooks.publish_task_status = [this](const Task & t) {statuses_.push_back(t.status);};
    hooks.publish_payload_attach = [this](const std::string & a) {attach_.push_back(a);};
    ctx_ = std::make_shared<ExecutorContext>([this]() {return now_;}, hooks);
    ctx_->setPayloadTable({{"small", {2.0, 0.5}}, {"medium", {10.0, 0.5}}});
    ctx_->setDocks(
      {DockSpec{"dock_1", "map", -28.29, 17.0, M_PI, 1.9599},
        DockSpec{"dock_a", "map", 28.29, 17.0, 0.0, std::nullopt},
        DockSpec{"charger_c1", "map", -26.0, -16.64, -M_PI_2, std::nullopt}}, 1.8, M_PI);
    ctx_->setChargers({"charger_c1"}, 0, 3.0);
    amr_behavior::registerCoreNodes(factory_, ctx_);
    scripts_ = amr_behavior_test::registerStubRosNodes(factory_);
    // 존 노드 스텁 (실제 노드와 같은 ID · 포트)
    commit_ = std::make_shared<CommitScript>();
    factory_.registerBuilder<StubCommit>(
      "CommitTaskLock", [this](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<StubCommit>(
          name, config, commit_, [this]() {return phases_.size();});
      });
    factory_.registerSimpleCondition(
      "ZonePathClear", [this](BT::TreeNode &) {
        ++zone_checks_;
        return zone_clear_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
      });
    factory_.registerSimpleCondition(
      "IsFleetServerAlive", [this](BT::TreeNode &) {
        return server_alive_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
      });
    for (const char * id : {"AcquireZoneChain", "ReleaseZones", "ZoneGuard"}) {
      factory_.registerSimpleAction(
        id, [](BT::TreeNode &) {return NodeStatus::SUCCESS;}, {
          BT::InputPort<std::string>("zones"), BT::InputPort<int>("priority"),
          BT::InputPort<unsigned>("timeout_ms")});
    }
    config_.perception_timeout_ms = 40;
    config_.relocalization_timeout_ms = 150;
    config_.charge_retry_delay_ms = 20;
    config_.error_hold_ms = 60;
    config_.waiting_pose = makePose("map", 22.0, -16.0, 0.0);
    tree_ = std::make_unique<BT::Tree>(
      amr_behavior::createTaskTree(factory_, kDistXml, config_));
  }

  void TearDown() override {tree_->haltTree();}

  StubScript & stub(const std::string & id) {return *scripts_.at(id);}

  void tick()
  {
    now_ += 0.1;
    amr_msgs::msg::DetectedObjectArray arr;
    amr_msgs::msg::DetectedObject o;
    o.class_name = "box";
    o.confidence = 0.9F;
    o.distance = 1.1F;
    arr.objects.push_back(o);
    ctx_->updateDetectedObjects(arr);
    ASSERT_EQ(tree_->tickRoot(), NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  bool runUntil(const std::function<bool()> & done, int max_ticks = 3000)
  {
    for (int i = 0; i < max_ticks; ++i) {
      tick();
      if (done()) {
        return true;
      }
    }
    return false;
  }

  bool finishedTask() const
  {
    return !statuses_.empty() &&
           (statuses_.back() == Task::STATUS_COMPLETED ||
           statuses_.back() == Task::STATUS_FAILED) && ctx_->phase() == "IDLE";
  }

  void assign(const std::string & id = "T1")
  {
    Task t;
    t.task_id = id;
    t.item_type = "medium";
    t.pickup_pose = makePose("map", -27.5, 17.0, 0.0);
    t.dropoff_pose = makePose("map", 27.5, 17.0, 0.0);
    ASSERT_TRUE(ctx_->acceptTask(t).accepted);
    amr_behavior::resetTaskProgress(*tree_->rootBlackboard());
  }

  double now_{0.0};
  bool zone_clear_{true};
  bool server_alive_{true};
  int zone_checks_{0};
  std::vector<std::string> phases_;
  std::vector<uint8_t> statuses_;
  std::vector<std::string> attach_;
  std::shared_ptr<ExecutorContext> ctx_;
  BT::BehaviorTreeFactory factory_;
  std::map<std::string, std::shared_ptr<StubScript>> scripts_;
  std::shared_ptr<CommitScript> commit_;
  amr_behavior::TaskTreeConfig config_;
  std::unique_ptr<BT::Tree> tree_;
};

}  // namespace

TEST_F(DistributedTreeTest, NominalCommitsTheTaskLockRightBeforeLoading)
{
  tick();
  assign("T7");
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  const std::vector<std::string> expected = {
    "IDLE", "MOVING", "PERCEIVING", "DOCKING", "LOADING", "UNDOCKING", "MOVING", "DOCKING",
    "UNLOADING", "UNDOCKING", "RETURNING", "IDLE"};
  EXPECT_EQ(phases_, expected);                           // 중앙 트리와 같은 단계 순서
  EXPECT_EQ(statuses_, (std::vector<uint8_t>{Task::STATUS_IN_PROGRESS, Task::STATUS_COMPLETED}));
  ASSERT_EQ(commit_->task_ids, (std::vector<std::string>{"T7"}));
  // IDLE MOVING PERCEIVING DOCKING 뒤, LOADING 전
  EXPECT_EQ(commit_->at_phase.front(), 4U);
  EXPECT_EQ(attach_, (std::vector<std::string>{"medium", ""}));
  EXPECT_GT(zone_checks_, 0);
}

TEST_F(DistributedTreeTest, LockFailureSkipsLoadingAndReturns)
{
  commit_->outcome = NodeStatus::FAILURE;                 // 다른 로봇이 확정 · 시간 초과
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_FAILED);
  EXPECT_EQ(ctx_->lastFailureReason(), "task_lock_failed");
  EXPECT_TRUE(attach_.empty());                           // 적재하지 않았다
  EXPECT_EQ(std::count(phases_.begin(), phases_.end(), "LOADING"), 0);
  EXPECT_EQ(stub("NavigateToPose").starts.back(), "goal=map:22.000000,-16.000000");   // 대기 구역
  EXPECT_EQ(stub("Dock").calls, 1);                       // 하역 도킹 안 함
}

TEST_F(DistributedTreeTest, ZoneGateStopsMotionUntilPermission)
{
  stub("NavigateToPose").running_ticks = 30;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return stub("NavigateToPose").calls == 1;}));
  zone_clear_ = false;                                    // 결정점 안, 사슬 허가 없음
  tick();
  EXPECT_EQ(stub("NavigateToPose").halts, 1);             // goal 취소 = 정지
  for (int i = 0; i < 40; ++i) {
    tick();
  }
  EXPECT_EQ(stub("NavigateToPose").calls, 1);             // 막힌 동안 다시 보내지 않는다
  EXPECT_EQ(ctx_->phase(), "MOVING");
  zone_clear_ = true;
  stub("NavigateToPose").running_ticks = 1;
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
  // 같은 목표 재전송
  EXPECT_EQ(stub("NavigateToPose").starts[1], "goal=map:-28.290000,17.000000");
}

TEST_F(DistributedTreeTest, ZoneGateAlsoGatesDockingAndUndocking)
{
  stub("Dock").running_ticks = 30;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return stub("Dock").calls == 1;}));
  zone_clear_ = false;
  tick();
  EXPECT_EQ(stub("Dock").halts, 1);                       // 도킹도 움직이는 단계 — 게이트 뒤
  zone_clear_ = true;
  stub("Dock").running_ticks = 1;
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
}

TEST_F(DistributedTreeTest, CentralHoldIsFollowedOnlyWhileTheServerIsAlive)
{
  stub("NavigateToPose").running_ticks = 30;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return stub("NavigateToPose").calls == 1;}));
  ctx_->updateYieldPose(makePose("map", 3.0, 4.0, 0.0));
  ctx_->updateTrafficHold(true);
  tick();
  EXPECT_EQ(stub("NavigateToPose").halts, 1);             // 서버가 살아 있으면 hold 를 따른다
  ASSERT_TRUE(runUntil([this]() {return stub("NavigateToPose").calls == 2;}, 5));
  EXPECT_EQ(stub("NavigateToPose").starts[1], "goal=map:3.000000,4.000000");   // 양보 위치
  server_alive_ = false;                                  // 서버 사망 — latched hold 는 남아 있다
  stub("NavigateToPose").running_ticks = 1;
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);    // hold=true 그대로인데 작업을 끝냈다
  EXPECT_TRUE(ctx_->trafficHold());
}

TEST_F(DistributedTreeTest, ZoneGateIsCheckedBeforeYieldMotion)
{
  stub("NavigateToPose").running_ticks = 30;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return stub("NavigateToPose").calls == 1;}));
  zone_clear_ = false;
  ctx_->updateYieldPose(makePose("map", 3.0, 4.0, 0.0));
  ctx_->updateTrafficHold(true);
  for (int i = 0; i < 20; ++i) {
    tick();
  }
  EXPECT_EQ(stub("NavigateToPose").calls, 1);             // 양보 위치로도 안 간다 (구역 허가 먼저)
  zone_clear_ = true;
  ASSERT_TRUE(runUntil([this]() {return stub("NavigateToPose").calls == 2;}, 10));
  EXPECT_EQ(stub("NavigateToPose").starts[1], "goal=map:3.000000,4.000000");
}

TEST_F(DistributedTreeTest, EveryNodeInTheDistributedTreeIsRegisteredAndModelled)
{
  for (const auto & id : amr_behavior::nodeIdsUsedInFile(kDistXml)) {
    EXPECT_NE(factory_.manifests().find(id), factory_.manifests().end()) << id;
  }
  // bt_tool 경로: 존 노드 모델만으로도 분산 트리를 합쳐 읽을 수 있다
  BT::BehaviorTreeFactory model;
  auto ctx = std::make_shared<ExecutorContext>([]() {return 0.0;});
  amr_behavior::registerCoreNodes(model, ctx);
  amr_behavior::registerRosNodeManifests(model);
  amr_behavior::registerFleetAgentNodeManifests(model);
  const std::string flat = amr_behavior::flattenTreeXml(kDistXml, model);
  for (const auto & id : amr_behavior::zoneNodeIds()) {
    EXPECT_NE(flat.find("ID=\"" + id + "\""), std::string::npos) << id;
  }
  EXPECT_NE(flat.find("<BehaviorTree ID=\"PickupLocked\""), std::string::npos);
}
