// 작업 트리(behavior_trees/*.xml) 통합 시험: ROS 액션/서비스 노드를 스텁으로 주입하고 정상
// 시나리오와 복구 경로(주행 불가, 인식 실패, 도킹 3회 실패, E-stop, 교통 hold, 위치 상실, 충전)를
// 검증한다. 리뷰 회귀: 물품 쪽 돌아보기, 하역 실패 뒤 물품 되돌려 놓기·작업 차단, halt 정리(충전
// 끄기·복귀 유지·도킹/복구 중 교통 hold), 실패 시 ERROR 유지.
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
constexpr unsigned kErrorHoldMs = 60;
const char kPerceiveTurn[] = "spin=-1.181693";   // dock_1 perceive_yaw 1.9599 − staging π

class TaskTreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ExecutorHooks hooks;
    hooks.publish_phase = [this](const std::string & p) {
        phases_.push_back(p);
        phase_times_.push_back(std::chrono::steady_clock::now());
      };
    hooks.publish_task_status = [this](const Task & t) {statuses_.push_back(t.status);};
    hooks.publish_payload_attach = [this](const std::string & a) {attach_.push_back(a);};
    hooks.publish_charging = [this](bool c) {charging_.push_back(c);};
    ctx_ = std::make_shared<ExecutorContext>([this]() {return now_;}, hooks);
    ctx_->setPayloadTable({{"small", {2.0, 0.5}}, {"medium", {10.0, 0.5}}});
    ctx_->setDocks(
      {DockSpec{"dock_1", "map", -28.29, 17.0, M_PI, 1.9599},
        DockSpec{"dock_a", "map", 28.29, 17.0, 0.0, std::nullopt},
        DockSpec{"charger_c1", "map", -26.0, -16.64, -M_PI_2, std::nullopt}}, 1.8, M_PI);
    ctx_->setChargers({"charger_c1"}, 0, 3.0);
    amr_behavior::registerCoreNodes(factory_, ctx_);
    scripts_ = amr_behavior_test::registerStubRosNodes(factory_);

    config_.perception_timeout_ms = 40;
    config_.relocalization_timeout_ms = 150;
    config_.charge_retry_delay_ms = 20;
    config_.error_hold_ms = kErrorHoldMs;
    config_.waiting_pose = makePose("map", 22.0, -16.0, 0.0);
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
  std::vector<std::chrono::steady_clock::time_point> phase_times_;
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
  for (const char * id : {"TaskExecutor", "MoveTo", "Perceive", "DockAt", "Undock", "Load",
      "Unload", "ReturnPayload", "RecoverNavigation", "RecoverPerception", "RecoverDocking",
      "Charge", "TrafficGate", "Yield"})
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
  EXPECT_EQ(amr_behavior::globalKeys().size(), 31U);   // + task_timeout_ms
  for (const auto & key : amr_behavior::globalKeys()) {
    EXPECT_NE(tree_->rootBlackboard()->getAny(key), nullptr) << key;
  }
  // 리터럴 서브트리 인자(phase)는 부모로 새지 않는다 (__autoremap 이 마지막 속성)
  EXPECT_EQ(tree_->rootBlackboard()->getAny("phase"), nullptr);
}

TEST_F(TaskTreeTest, TaskDeadlineFailsTheTaskInsteadOfHangingForever)
{
  // 회귀 (통합 시나리오 11): 위치 상실로 Nav2 가 취소되자 실행기가 MOVING 에 1340 s 머물러
  // 작업이 끝나지도 실패하지도 않았다. 상한을 넘으면 task_timeout 으로 실패 보고 후 복귀한다.
  config_.task_timeout_ms = 2000;                 // tick 0.1 s → 20 tick
  tree_ = std::make_unique<BT::Tree>(amr_behavior::createTaskTree(factory_, kMainXml, config_));
  stub("NavigateToPose").running_ticks = 100000;  // 끝나지 않는 주행
  tick();
  assign();
  ASSERT_TRUE(
    runUntil(
      [this]() {
        return !statuses_.empty() && statuses_.back() == Task::STATUS_FAILED;
      }));
  EXPECT_EQ(ctx_->lastFailureReason(), "task_timeout");
  EXPECT_GE(stub("NavigateToPose").halts, 1);     // 주행을 실제로 멈췄다
  stub("NavigateToPose").running_ticks = 1;       // 복귀 주행은 정상
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  const auto err = std::find(phases_.begin(), phases_.end(), "ERROR");
  ASSERT_NE(err, phases_.end());
  EXPECT_EQ(phases_.back(), "IDLE");
}

TEST_F(TaskTreeTest, NominalScenarioPhaseSequence)
{
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  const std::vector<std::string> expected = {
    "IDLE", "MOVING", "PERCEIVING", "DOCKING", "LOADING", "UNDOCKING", "MOVING", "DOCKING",
    "UNLOADING", "UNDOCKING", "RETURNING", "IDLE"};
  EXPECT_EQ(phases_, expected);
  EXPECT_EQ(statuses_, (std::vector<uint8_t>{Task::STATUS_IN_PROGRESS, Task::STATUS_COMPLETED}));
  EXPECT_EQ(attach_, (std::vector<std::string>{"medium", ""}));
  ASSERT_EQ(stub("NavigateToPose").calls, 3);
  EXPECT_EQ(stub("NavigateToPose").starts[0], "goal=map:-28.290000,17.000000");   // staging
  EXPECT_EQ(stub("NavigateToPose").starts[2], "goal=map:22.000000,-16.000000");   // 대기 구역
  ASSERT_EQ(stub("Dock").calls, 2);
  EXPECT_EQ(stub("Dock").starts[0], "dock_id=dock_1");
  EXPECT_EQ(stub("Dock").starts[1], "dock_id=dock_a");
  // 인식: 물품 쪽으로 돌고(−67.7°) 확인 뒤 되돌아온다 (reverse)
  ASSERT_EQ(stub("Spin").calls, 2);
  EXPECT_EQ(stub("Spin").starts[0], kPerceiveTurn);
  // 적재·하역 뒤 판 앞에서 후진 이탈
  ASSERT_EQ(stub("BackUp").calls, 2);
  EXPECT_EQ(stub("BackUp").starts[0], "backup=1.550000");
  EXPECT_FALSE(ctx_->hasTask());
  EXPECT_FALSE(ctx_->returnPending());
}

TEST_F(TaskTreeTest, PerceptionNeedsTheTurnTowardTheItems)
{
  // 리뷰 회귀: staging 방위에서는 도크 물품(±83°)이 시야 밖 → 돌아보기 전에는 인식이 안 된다.
  // 물품 쪽으로 돈 뒤 보이면 인식 복구(후진·회전) 없이 첫 시도에 확인해야 한다.
  perception_ok_ = false;
  on_tick_ = [this]() {perception_ok_ = stub("Spin").calls >= 1;};
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
  EXPECT_EQ(count("RECOVERING"), 0);
  EXPECT_EQ(stub("BackUp").calls, 2);   // 이탈 2 회뿐 (인식 복구 후진 없음)
  EXPECT_EQ(stub("Spin").calls, 2);
}

TEST_F(TaskTreeTest, NavigationFailureRecoversWithClearSpinRetry)
{
  stub("NavigateToPose").script({NodeStatus::FAILURE, NodeStatus::FAILURE});
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
  EXPECT_EQ(stub("ClearCostmap").calls, 4);   // 전역 + 지역 × 2 회 (Parallel)
  EXPECT_EQ(stub("Spin").calls, 4);   // 복구 2 + 물품 돌아보기 2
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
  EXPECT_EQ(stub("Spin").calls, 4);   // 돌아보기 1 + 복구 3 (확인 실패 → 되돌아오기 없음)
  EXPECT_EQ(stub("Spin").starts[0], kPerceiveTurn);
  EXPECT_EQ(stub("Spin").starts[1], "spin=0.520000");
  EXPECT_EQ(stub("Dock").calls, 0);
  EXPECT_TRUE(attach_.empty());
}

TEST_F(TaskTreeTest, PerceptionSucceedsAfterRecoveryRotation)
{
  perception_ok_ = false;
  on_tick_ = [this]() {perception_ok_ = stub("Spin").calls >= 2;};   // 복구 회전 뒤 물품이 보인다
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
  EXPECT_EQ(stub("Spin").calls, 3);     // 돌아보기 + 복구 1 + 되돌아오기
  EXPECT_EQ(stub("BackUp").calls, 3);   // 복구 1 + 이탈 2
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
  // 플릿(2 Hz 샘플)이 볼 수 있도록 ERROR 를 error_hold_ms 이상 유지한 뒤 복귀한다
  const auto err = std::find(phases_.begin(), phases_.end(), "ERROR");
  ASSERT_NE(err, phases_.end());
  const size_t i = static_cast<size_t>(err - phases_.begin());
  ASSERT_LT(i + 1, phases_.size());
  EXPECT_EQ(phases_[i + 1], "RETURNING");
  EXPECT_GE(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      phase_times_[i + 1] - phase_times_[i]).count(), static_cast<int64_t>(kErrorHoldMs));
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
  EXPECT_EQ(stub("BackUp").calls, 4);   // 재시도 전 후진 2 + 이탈 2
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
  EXPECT_TRUE(ctx_->chargerClaim().empty());   // 점유 해제
}

TEST_F(TaskTreeTest, EstopWhileChargingTurnsChargingOffAndFinishesWithUndock)
{
  // 리뷰 회귀: 충전 중 E-stop → charging/enable 이 켜진 채 남고, 해제 뒤 이탈 없이 IDLE 로
  // 작업을 받았다
  ctx_->updateBattery(10.0);
  ASSERT_TRUE(runUntil([this]() {return ctx_->phase() == "CHARGING";}));
  ASSERT_TRUE(ctx_->charging());
  EXPECT_EQ(ctx_->chargerClaim(), "charger_c1");
  ctx_->updateEstop(true);
  tick();
  EXPECT_FALSE(ctx_->charging());   // halt 즉시 충전 끔
  ctx_->updateBattery(50.0);        // 저전량(20 %)은 벗어났지만 재개 기준(80 %) 전
  for (int i = 0; i < 10; ++i) {
    tick();
  }
  ctx_->updateEstop(false);
  tick();
  // 잔량 50 % 여도 끊긴 충전을 이어 한다 (IDLE 로 가서 작업을 받지 않는다)
  EXPECT_EQ(ctx_->phase(), "CHARGING");
  EXPECT_TRUE(ctx_->charging());
  EXPECT_EQ(ctx_->evaluateTask(Task{}).message, "busy:charging");
  EXPECT_EQ(stub("Dock").calls, 1);   // 도킹을 반복하지 않는다
  ctx_->updateBattery(90.0);
  ASSERT_TRUE(runUntil([this]() {return ctx_->phase() == "IDLE";}));
  EXPECT_EQ(charging_, (std::vector<bool>{true, false, true, false}));
  EXPECT_EQ(stub("BackUp").calls, 1);   // 이탈 후진
  EXPECT_EQ(stub("NavigateToPose").calls, 1);
}

TEST_F(TaskTreeTest, EstopWhileChargingAboveResumeStillUndocks)
{
  ctx_->updateBattery(10.0);
  ASSERT_TRUE(runUntil([this]() {return ctx_->phase() == "CHARGING";}));
  ctx_->updateEstop(true);
  tick();
  ctx_->updateBattery(95.0);
  ctx_->updateEstop(false);
  ASSERT_TRUE(runUntil([this]() {return ctx_->phase() == "IDLE";}));
  EXPECT_FALSE(ctx_->charging());
  EXPECT_EQ(stub("BackUp").calls, 1);
  EXPECT_TRUE(ctx_->chargerClaim().empty());
}

TEST_F(TaskTreeTest, EstopDuringReturnKeepsThePendingReturn)
{
  // 리뷰 회귀: 복귀(RETURNING) 중 E-stop → 작업이 이미 끝나 복귀가 사라지고 그 자리에서
  // IDLE 이 됐다
  stub("NavigateToPose").running_ticks = 20;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return ctx_->phase() == "RETURNING";}));
  const int calls = stub("NavigateToPose").calls;
  const int halts = stub("NavigateToPose").halts;
  ctx_->updateEstop(true);
  tick();
  EXPECT_EQ(stub("NavigateToPose").halts, halts + 1);
  ctx_->updateEstop(false);
  ASSERT_TRUE(runUntil([this]() {return ctx_->phase() == "IDLE";}));
  EXPECT_EQ(stub("NavigateToPose").calls, calls + 1);   // 복귀 재전송
  EXPECT_EQ(stub("NavigateToPose").starts.back(), "goal=map:22.000000,-16.000000");
  EXPECT_FALSE(ctx_->returnPending());
}

TEST_F(TaskTreeTest, TrafficHoldPausesDockingWithoutSpendingAnAttempt)
{
  // 리뷰 회귀: traffic/hold 는 MoveTo 안에서만 보아 도킹 중 hold 를 무시했다
  stub("Dock").running_ticks = 30;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return stub("Dock").calls == 1;}));
  ctx_->updateTrafficHold(true);
  tick();
  EXPECT_EQ(stub("Dock").halts, 1);   // 도킹 goal 취소
  for (int i = 0; i < 40; ++i) {
    tick();
  }
  EXPECT_EQ(stub("Dock").calls, 1);   // hold 동안 다시 보내지 않는다
  ctx_->updateTrafficHold(false);
  stub("Dock").running_ticks = 1;
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
  EXPECT_EQ(stub("Dock").calls, 3);   // 재전송 1 + 하역 1
  EXPECT_EQ(count("RECOVERING"), 0);  // 시도로 세지 않았다 (도킹 복구 없음)
}

TEST_F(TaskTreeTest, TrafficHoldPausesRecoveryMotion)
{
  stub("NavigateToPose").script({NodeStatus::FAILURE});
  stub("Spin").running_ticks = 30;
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return stub("Spin").calls == 1;}));   // 주행 복구 회전 중
  ctx_->updateTrafficHold(true);
  tick();
  EXPECT_EQ(stub("Spin").halts, 1);
  for (int i = 0; i < 40; ++i) {
    tick();
  }
  EXPECT_EQ(stub("Spin").calls, 1);
  ctx_->updateTrafficHold(false);
  stub("Spin").running_ticks = 1;
  ASSERT_TRUE(runUntil([this]() {return finishedTask();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_COMPLETED);
}

TEST_F(TaskTreeTest, DropoffDockingFailureReturnsThePayloadToThePickupDock)
{
  // 리뷰 회귀: 하역 도킹 3회 실패 → 물품이 실린 채 FAILED, 새 작업을 받아 그 위에 적재했다
  stub("Dock").script(
    {NodeStatus::SUCCESS, NodeStatus::FAILURE, NodeStatus::FAILURE, NodeStatus::FAILURE});
  tick();
  assign();
  ASSERT_TRUE(runUntil([this]() {return finishedTask() && ctx_->attachedPayload().empty();}));
  EXPECT_EQ(statuses_.back(), Task::STATUS_FAILED);
  EXPECT_EQ(ctx_->lastFailureReason(), "dock_failed");
  // 에러 보고 → 대체 작업: 적재 도크(dock_1)로 가서 하역 → 이탈 → 대기 구역
  ASSERT_EQ(stub("Dock").calls, 5);
  EXPECT_EQ(stub("Dock").starts[4], "dock_id=dock_1");
  EXPECT_EQ(attach_, (std::vector<std::string>{"medium", ""}));
  const auto err = std::find(phases_.begin(), phases_.end(), "ERROR");
  ASSERT_NE(err, phases_.end());
  const std::vector<std::string> after(err, phases_.end());
  const std::vector<std::string> expected = {
    "ERROR", "MOVING", "DOCKING", "UNLOADING", "UNDOCKING", "RETURNING", "IDLE"};
  EXPECT_EQ(after, expected);
  EXPECT_EQ(stub("NavigateToPose").starts.back(), "goal=map:22.000000,-16.000000");
  // 다시 작업을 받을 수 있다 (Task{} 는 id 가 비어 invalid 로 거절 = 차단 사유가 아님)
  EXPECT_EQ(ctx_->evaluateTask(Task{}).message, "invalid:task_id");
}

TEST_F(TaskTreeTest, PayloadThatCannotBeReturnedBlocksNewTasks)
{
  stub("Dock").script({NodeStatus::SUCCESS});
  stub("Dock").fallback = NodeStatus::FAILURE;   // 하역·되돌려 놓기 모두 실패
  tick();
  assign();
  ASSERT_TRUE(
    runUntil(
      [this]() {
        return !statuses_.empty() && statuses_.back() == Task::STATUS_FAILED &&
        !ctx_->returnPending() && ctx_->phase() == "ERROR";
      }));
  EXPECT_EQ(stub("Dock").calls, 7);   // 적재 1 + 하역 3 + 되돌려 놓기 3
  EXPECT_EQ(ctx_->attachedPayload(), "medium");
  for (int i = 0; i < 20; ++i) {
    tick();
  }
  EXPECT_EQ(ctx_->phase(), "ERROR");   // IDLE 로 가지 않는다 (플릿이 배정하지 않도록)
  EXPECT_EQ(stub("Dock").calls, 7);    // 되돌려 놓기를 반복하지 않는다
  Task next;
  next.task_id = "T2";
  next.item_type = "small";
  next.pickup_pose = makePose("map", -27.5, 17.0, 0.0);
  next.dropoff_pose = makePose("map", 27.5, 17.0, 0.0);
  EXPECT_EQ(ctx_->acceptTask(next).message, "blocked:payload");
  EXPECT_EQ(attach_, (std::vector<std::string>{"medium"}));   // 위에 싣지 않았다
  ctx_->detachPayload();   // clear_payload: 작업자가 내림
  ASSERT_TRUE(runUntil([this]() {return ctx_->phase() == "IDLE";}));
  EXPECT_TRUE(ctx_->acceptTask(next).accepted);
}


TEST_F(TaskTreeTest, ChargeFailureCoolsDownInsteadOfSpinning)
{
  ctx_->updateBattery(10.0);
  stub("NavigateToPose").fallback = NodeStatus::FAILURE;
  ASSERT_TRUE(runUntil([this]() {return count("ERROR") >= 1;}));
  const int calls = stub("NavigateToPose").calls;
  EXPECT_EQ(calls, 3);
  EXPECT_TRUE(charging_.empty());
  EXPECT_TRUE(ctx_->chargerClaim().empty());   // 실패하면 점유를 푼다
}

TEST_F(TaskTreeTest, AllChargersClaimedCoolsDown)
{
  ctx_->updateChargerClaim("amr_09", "charger_c1", -1.0);
  ctx_->updateBattery(10.0);
  ASSERT_TRUE(runUntil([this]() {return count("ERROR") >= 1;}));
  EXPECT_EQ(stub("NavigateToPose").calls, 0);
}
