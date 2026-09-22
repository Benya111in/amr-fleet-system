// 작업 트리(behavior_trees/*.xml) 통합 시험: ROS 액션/서비스 노드를 스텁으로 주입하고 정상
// 시나리오와 복구 경로(주행 불가, 인식 실패, 도킹 3회 실패, E-stop, 교통 hold, 위치 상실, 충전)를
// 검증한다.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
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

const char kMainXml[] = AMR_BEHAVIOR_SOURCE_DIR "/behavior_trees/task_executor.xml";

class TaskTreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ExecutorHooks hooks;
    hooks.publish_phase = [this](const std::string & p) {phases_.push_back(p);};
    hooks.publish_task_status = [this](const Task & t) {statuses_.push_back(t.status);};
    hooks.publish_payload_attach = [this](const std::string & a) {attach_.push_back(a);};
    hooks.publish_charging = [this](bool c) {charging_.push_back(c);};
    ctx_ = std::make_shared<ExecutorContext>([this]() {return now_;}, hooks);
    ctx_->setPayloadTable({{"small", {2.0, 0.5}}, {"medium", {10.0, 0.5}}});
    ctx_->setDocks(
      {DockSpec{"dock_1", "map", -28.29, 17.0, M_PI}, DockSpec{"dock_a", "map", 28.29, 17.0, 0.0},
        DockSpec{"charger_c1", "map", -26.0, -16.64, -M_PI_2}}, 1.6, M_PI);
    amr_behavior::registerCoreNodes(factory_, ctx_);
    scripts_ = amr_behavior_test::registerStubRosNodes(factory_);

    config_.perception_timeout_ms = 40;
    config_.relocalization_timeout_ms = 150;
    config_.charge_retry_delay_ms = 20;
    config_.waiting_pose = makePose("map", 22.0, -16.0, 0.0);
    config_.charger_goal = makePose("map", -26.0, -16.64, -M_PI_2);
    config_.charger_dock_id = "charger_c1";
    tree_ = std::make_unique<BT::Tree>(
      amr_behavior::createTaskTree(factory_, kMainXml, config_));
  }

  void TearDown() override
  {
    tree_->haltTree();
  }

  StubScript & stub(const std::string & id) {return *scripts_.at(id);}

  void publishBox()
  {
    amr_msgs::msg::DetectedObjectArray arr;
    amr_msgs::msg::DetectedObject o;
    o.class_name = "box";
    o.confidence = 0.9F;
    o.distance = 1.1F;
    arr.objects.push_back(o);
    ctx_->updateDetectedObjects(arr);
  }

  void tick()
  {
    now_ += 0.1;
    if (perception_ok_) {
      publishBox();
    }
    if (on_tick_) {
      on_tick_();
    }
    ASSERT_EQ(tree_->tickRoot(), NodeStatus::RUNNING);   // 메인 루프는 끝나지 않는다
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
    t.pickup_pose = makePose("map", -27.5, 17.0, 0.0);    // dock_1 패드
    t.dropoff_pose = makePose("map", 27.5, 17.0, 0.0);    // dock_a 패드
    ASSERT_TRUE(ctx_->acceptTask(t).accepted);
    amr_behavior::resetTaskProgress(*tree_->rootBlackboard());
  }

  int count(const std::string & phase) const
  {
    return static_cast<int>(std::count(phases_.begin(), phases_.end(), phase));
  }

  double now_{0.0};
  bool perception_ok_{true};
  std::function<void()> on_tick_;
  std::vector<std::string> phases_;
  std::vector<uint8_t> statuses_;
  std::vector<std::string> attach_;
  std::vector<bool> charging_;
  std::shared_ptr<ExecutorContext> ctx_;
  BT::BehaviorTreeFactory factory_;
  std::map<std::string, std::shared_ptr<StubScript>> scripts_;
  amr_behavior::TaskTreeConfig config_;
  std::unique_ptr<BT::Tree> tree_;
};

}  // namespace

TEST_F(TaskTreeTest, TreeUsesAtLeastFifteenRegisteredNodeTypes)
{
  const auto used = amr_behavior::nodeIdsUsedInFile(kMainXml);
  std::map<BT::NodeType, int> per_type;
  int distinct = 0;
  for (const auto & id : used) {
    const auto it = factory_.manifests().find(id);
    ASSERT_NE(it, factory_.manifests().end()) << "등록되지 않은 노드: " << id;
    if (it->second.type == BT::NodeType::SUBTREE) {
      continue;
    }
    ++per_type[it->second.type];
    ++distinct;
  }
  EXPECT_GE(distinct, 15);
  EXPECT_GE(per_type[BT::NodeType::ACTION], 4);
  EXPECT_GE(per_type[BT::NodeType::CONDITION], 4);
  EXPECT_GE(per_type[BT::NodeType::CONTROL], 4);
  EXPECT_GE(per_type[BT::NodeType::DECORATOR], 4);
  // components.md §3.5 노드 목록이 모두 트리에 쓰인다
  const std::set<std::string> used_set(used.begin(), used.end());
  for (const char * required : {
    "Sequence", "Fallback", "ReactiveSequence", "Parallel", "RetryUntilSuccessful", "Timeout",
    "Inverter", "KeepRunningUntilFailure", "IsTaskAssigned", "IsBatteryOk", "IsEstopClear",
    "IsTrafficHold", "IsLocalized", "IsObjectDetected", "IsDockMarkerVisible", "NavigateToPose",
    "Dock", "SimulateLoad", "SimulateUnload", "ReportTaskStatus", "Spin", "BackUp", "Wait",
    "ClearCostmap"})
  {
    EXPECT_EQ(used_set.count(required), 1U) << required;
  }
  EXPECT_GE(amr_behavior::customNodeIds(factory_).size(), 15U);
  RecordProperty("distinct_node_types", distinct);
}

TEST_F(TaskTreeTest, SubtreesAreDefinedAndFlattenedTreeLoads)
{
  const std::string flat = amr_behavior::flattenTreeXml(kMainXml, factory_);
  for (const char * id : {"TaskExecutor", "MoveTo", "Perceive", "DockAt", "Load", "Unload",
      "RecoverNavigation", "RecoverPerception", "RecoverDocking", "Charge", "Yield"})
  {
    EXPECT_NE(flat.find(std::string("<BehaviorTree ID=\"") + id + "\""), std::string::npos) << id;
  }
  EXPECT_NE(flat.find("<TreeNodesModel>"), std::string::npos);
  EXPECT_NE(flat.find("ID=\"SimulateLoad\""), std::string::npos);
  auto bb = BT::Blackboard::create();
  amr_behavior::writeConfig(config_, *bb);
  EXPECT_NO_THROW(factory_.createTreeFromText(flat, bb));
  EXPECT_NE(
    amr_behavior::treeNodesModelXml(factory_).find("ID=\"IsTaskAssigned\""), std::string::npos);
  EXPECT_EQ(amr_behavior::globalKeys().size(), 27U);
  for (const auto & key : amr_behavior::globalKeys()) {
    EXPECT_NE(tree_->rootBlackboard()->getAny(key), nullptr) << key;
  }
  // 리터럴 서브트리 인자(phase)는 부모로 새지 않는다 (__autoremap 이 마지막 속성)
  EXPECT_EQ(tree_->rootBlackboard()->getAny("phase"), nullptr);
}

TEST_F(TaskTreeTest, NominalScenarioPhaseSequence)
{
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  const std::vector<std::string> expected = {
    "IDLE", "MOVING", "DOCKING", "LOADING", "MOVING", "DOCKING", "UNLOADING", "RETURNING", "IDLE"};
  EXPECT_EQ(phases_, expected);
  EXPECT_EQ(statuses_, (std::vector<uint8_t>{Task::STATUS_IN_PROGRESS, Task::STATUS_COMPLETED}));
  EXPECT_EQ(attach_, (std::vector<std::string>{"medium", ""}));
  ASSERT_EQ(stub("NavigateToPose").calls, 3);
  EXPECT_EQ(stub("NavigateToPose").starts[0], "goal=map:-28.290000,17.000000");   // staging
  EXPECT_EQ(stub("NavigateToPose").starts[2], "goal=map:22.000000,-16.000000");   // 대기 구역
  ASSERT_EQ(stub("Dock").calls, 2);
  EXPECT_EQ(stub("Dock").starts[0], "dock_id=dock_1");
  EXPECT_EQ(stub("Dock").starts[1], "dock_id=dock_a");
  EXPECT_EQ(stub("Spin").calls, 0);
  EXPECT_EQ(stub("BackUp").calls, 0);
  EXPECT_FALSE(ctx_->hasTask());
}

TEST_F(TaskTreeTest, NavigationFailureRecoversWithClearSpinRetry)
{
  stub("NavigateToPose").script({NodeStatus::FAILURE, NodeStatus::FAILURE});
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
  EXPECT_EQ(stub("ClearCostmap").calls, 4);   // 전역 + 지역 × 2 회 (Parallel)
  EXPECT_EQ(stub("Spin").calls, 2);
  EXPECT_EQ(stub("Wait").calls, 2);
  EXPECT_EQ(stub("NavigateToPose").calls, 5);   // 픽업 3 + 하역 1 + 복귀 1
  EXPECT_EQ(count("RECOVERING"), 2);
  EXPECT_NE(stub("ClearCostmap").starts[1].find("local_costmap"), std::string::npos);
}

TEST_F(TaskTreeTest, NavigationPermanentFailureReportsAndReturns)
{
  stub("NavigateToPose").fallback = NodeStatus::FAILURE;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_FAILED);
  EXPECT_EQ(ctx_->lastFailureReason(), "nav_failed");
  EXPECT_EQ(stub("NavigateToPose").calls, 6);   // 픽업 3 회 + 복귀(대체 작업) 3 회
  EXPECT_EQ(stub("Dock").calls, 0);
  const auto err = std::find(phases_.begin(), phases_.end(), "ERROR");
  ASSERT_NE(err, phases_.end());
  EXPECT_NE(std::find(err, phases_.end(), "RETURNING"), phases_.end());
  EXPECT_EQ(phases_.back(), "IDLE");
}

TEST_F(TaskTreeTest, PerceptionFailureRecoversThenFails)
{
  perception_ok_ = false;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_FAILED);
  EXPECT_EQ(ctx_->lastFailureReason(), "perception_failed");
  EXPECT_EQ(stub("BackUp").calls, 3);
  EXPECT_EQ(stub("Spin").calls, 3);
  EXPECT_EQ(stub("Spin").starts[0], "spin=0.520000");
  EXPECT_EQ(stub("Dock").calls, 0);
  EXPECT_TRUE(attach_.empty());
}

TEST_F(TaskTreeTest, PerceptionSucceedsAfterRotation)
{
  perception_ok_ = false;
  on_tick_ = [this]() {perception_ok_ = stub("Spin").calls >= 1;};   // 회전 뒤 물품이 보인다
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
  EXPECT_EQ(stub("Spin").calls, 1);
  EXPECT_EQ(stub("BackUp").calls, 1);
}

TEST_F(TaskTreeTest, DockingFailsThreeTimesReportsErrorAndReturns)
{
  stub("Dock").fallback = NodeStatus::FAILURE;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_FAILED);
  EXPECT_EQ(ctx_->lastFailureReason(), "dock_failed");
  EXPECT_EQ(stub("Dock").calls, 3);   // 명세: 최대 3 회
  EXPECT_EQ(stub("BackUp").calls, 3);   // 시도마다 0.3 m 후진
  EXPECT_EQ(stub("BackUp").starts[0], "backup=0.300000");
  // 마커가 안 보이므로 staging 재접근(3) + 픽업(1) + 대기 구역 복귀(1)
  EXPECT_EQ(stub("NavigateToPose").calls, 5);
  EXPECT_EQ(stub("NavigateToPose").starts.back(), "goal=map:22.000000,-16.000000");
  EXPECT_TRUE(attach_.empty());
  EXPECT_EQ(count("DOCKING"), 3);
}

TEST_F(TaskTreeTest, DockingSucceedsOnThirdAttempt)
{
  stub("Dock").script({NodeStatus::FAILURE, NodeStatus::FAILURE});
  ctx_->updateDockMarker(makePose("base_link", 0.8, 0.0, M_PI));
  on_tick_ = [this]() {ctx_->updateDockMarker(makePose("base_link", 0.8, 0.0, M_PI));};
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
  EXPECT_EQ(stub("Dock").calls, 4);
  EXPECT_EQ(stub("BackUp").calls, 2);
  EXPECT_EQ(stub("NavigateToPose").calls, 3);   // 마커가 보이면 staging 재접근 생략
}

TEST_F(TaskTreeTest, EstopPausesAndResumesWithoutRepeatingSteps)
{
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return ctx_->phase() == "LOADING";}));
  const int nav_calls = stub("NavigateToPose").calls;
  const int dock_calls = stub("Dock").calls;
  ctx_->updateEstop(true);
  for (int i = 0; i < 20; ++i) {
    tick();
  }
  EXPECT_TRUE(attach_.empty());   // 적재 중단 (이벤트 없음)
  EXPECT_EQ(ctx_->phase(), "LOADING");
  ctx_->updateEstop(false);
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
  // 재개는 중단된 적재 단계부터: 픽업 주행·도킹은 반복하지 않는다
  EXPECT_EQ(stub("NavigateToPose").calls, nav_calls + 2);
  EXPECT_EQ(stub("Dock").calls, dock_calls + 1);
  EXPECT_EQ(attach_, (std::vector<std::string>{"medium", ""}));

  // 주행 중 E-stop: goal 이 halt(취소)된다
  stub("NavigateToPose").running_ticks = 50;
  assign("T2");
  ASSERT_TRUE(
    runUntil([this, nav_calls]() {return stub("NavigateToPose").calls == nav_calls + 3;}));
  const int halts = stub("NavigateToPose").halts;
  ctx_->updateEstop(true);
  tick();
  EXPECT_EQ(stub("NavigateToPose").halts, halts + 1);
  ctx_->updateEstop(false);
  stub("NavigateToPose").running_ticks = 1;
  ASSERT_TRUE(runUntil([this]() {return finishedTask() && statuses_.size() == 4U;}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
}

TEST_F(TaskTreeTest, TrafficHoldYieldsThenResumes)
{
  stub("NavigateToPose").running_ticks = 30;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return stub("NavigateToPose").calls == 1;}));
  ctx_->updateYieldPose(makePose("map", 3.0, 4.0, 0.0));
  ctx_->updateTrafficHold(true);
  tick();
  EXPECT_EQ(stub("NavigateToPose").halts, 1);   // 주행 goal 취소
  ASSERT_TRUE(runUntil([this]() {return stub("NavigateToPose").calls == 2;}, 5));
  EXPECT_EQ(stub("NavigateToPose").starts[1], "goal=map:3.000000,4.000000");   // 양보 위치로
  for (int i = 0; i < 40; ++i) {
    tick();   // 양보 위치 도착 후 hold 동안 대기
  }
  EXPECT_EQ(stub("NavigateToPose").calls, 2);
  ctx_->updateTrafficHold(false);
  stub("NavigateToPose").running_ticks = 1;
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
  // 원래 목표 재전송
  EXPECT_EQ(stub("NavigateToPose").starts[2], "goal=map:-28.290000,17.000000");
}

TEST_F(TaskTreeTest, LocalizationLostWaitsThenRecovers)
{
  stub("NavigateToPose").running_ticks = 30;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return stub("NavigateToPose").calls == 1;}));
  ctx_->updateLocalizationLost(true);
  tick();
  EXPECT_EQ(stub("NavigateToPose").halts, 1);
  // relocalization_timeout_ms(150 ms) 경과 → 시도 실패 → 주행 복구(회전)
  ASSERT_TRUE(runUntil([this]() {return stub("Spin").calls >= 1;}, 500));
  ctx_->updateLocalizationLost(false);
  stub("NavigateToPose").running_ticks = 1;
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
}

TEST_F(TaskTreeTest, LowBatteryWhenIdleCharges)
{
  ctx_->updateBattery(10.0);
  ASSERT_TRUE(runUntil([this]() {return ctx_->phase() == "CHARGING";}));
  EXPECT_EQ(stub("NavigateToPose").starts[0], "goal=map:-26.000000,-16.640000");
  EXPECT_EQ(stub("Dock").starts[0], "dock_id=charger_c1");
  EXPECT_EQ(charging_, (std::vector<bool>{true}));
  EXPECT_EQ(ctx_->evaluateTask(Task{}).message, "busy:charging");
  ctx_->updateBattery(85.0);
  ASSERT_TRUE(runUntil([this]() {return ctx_->phase() == "IDLE";}));
  EXPECT_EQ(charging_, (std::vector<bool>{true, false}));
  EXPECT_EQ(stub("BackUp").calls, 1);   // 충전소 이탈
}

TEST_F(TaskTreeTest, ChargeFailureCoolsDownInsteadOfSpinning)
{
  ctx_->updateBattery(10.0);
  stub("NavigateToPose").fallback = NodeStatus::FAILURE;
  ASSERT_TRUE(runUntil([this]() {return count("ERROR") >= 1;}));
  const int calls = stub("NavigateToPose").calls;
  EXPECT_EQ(calls, 3);
  EXPECT_TRUE(charging_.empty());
}
