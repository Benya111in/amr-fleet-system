// 주행 BT 재계획 주기 회귀 시험: 배포 XML 4 종을 실제 BehaviorTree.CPP v3 + Nav2 제어 노드
// (PipelineSequence, RateController, RecoveryNode, RoundRobin 플러그인)로 돌리고, ROS 액션·조건은
// 호출 수를 세는 대역으로 바꿔 10 ms tick (bt_navigator bt_loop_duration) 으로 실시간 구동한다.
//
// 회귀 대상 (통합 시험 발견): ReactiveFallback 아래의 RateController 는 자식 SUCCESS 때마다
// ReactiveFallback 이 halt → IDLE → 다음 tick first_time 으로 즉시 다시 실행 → tick 마다 재계획
// (/plan 22 Hz, FollowPath 선점 수백 회). 배포 XML 은 1 Hz 에 가깝게 유효성 검사·재계획해야 한다.
#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace
{

// 대역 노드의 호출 기록과 결과 (id 별)
struct Stub
{
  std::map<std::string, int> starts;     // StatefulAction onStart / 조건 tick 수
  std::map<std::string, int> halts;      // 실행 중 halt 수 (FollowPath 선점)
  std::map<std::string, int> run_ticks;  // 액션이 RUNNING 으로 머무는 tick 수 (<0: 끝나지 않음)
  std::map<std::string, bool> cond;      // 조건 결과
  bool ttc_once{false};                  // IsTTCBelowThreshold 한 번만 SUCCESS
  bool empty_path_invalid{true};         // 계획 전(빈 경로) IsPathValid 는 FAILURE (planner_server)
};

class StubAction : public BT::StatefulActionNode
{
public:
  StubAction(
    const std::string & name, const BT::NodeConfiguration & cfg, Stub * s, std::string id)
  : BT::StatefulActionNode(name, cfg), s_(s), id_(std::move(id)) {}

  BT::NodeStatus onStart() override
  {
    ++s_->starts[id_];
    left_ = s_->run_ticks.count(id_) ? s_->run_ticks[id_] : 0;
    return left_ == 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    if (left_ < 0) {
      return BT::NodeStatus::RUNNING;
    }
    return --left_ > 0 ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {++s_->halts[id_];}

private:
  Stub * s_;
  std::string id_;
  int left_{0};
};

class StubCondition : public BT::ConditionNode
{
public:
  StubCondition(
    const std::string & name, const BT::NodeConfiguration & cfg, Stub * s, std::string id)
  : BT::ConditionNode(name, cfg), s_(s), id_(std::move(id)) {}

  BT::NodeStatus tick() override
  {
    ++s_->starts[id_];
    bool ok = s_->cond[id_];
    if (id_ == "IsPathValid" && s_->empty_path_invalid) {
      ok = ok && s_->starts["ComputePathToPose"] + s_->starts["ComputePathThroughPoses"] > 0;
    }
    if (id_ == "IsTTCBelowThreshold") {
      ok = s_->ttc_once;
      s_->ttc_once = false;
    }
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  Stub * s_;
  std::string id_;
};

void addStub(
  BT::BehaviorTreeFactory & f, Stub * s, const std::string & id, BT::NodeType type,
  const std::vector<std::string> & ports)
{
  BT::TreeNodeManifest m;
  m.type = type;
  m.registration_ID = id;
  for (const auto & p : ports) {
    m.ports.insert(BT::BidirectionalPort<std::string>(p));
  }
  if (type == BT::NodeType::CONDITION) {
    f.registerBuilder(
      m, [s, id](const std::string & name, const BT::NodeConfiguration & cfg) {
        return std::make_unique<StubCondition>(name, cfg, s, id);
      });
  } else {
    f.registerBuilder(
      m, [s, id](const std::string & name, const BT::NodeConfiguration & cfg) {
        return std::make_unique<StubAction>(name, cfg, s, id);
      });
  }
}

// 실제 Nav2 제어 노드 + 대역 액션·조건 (포트 이름은 배포 XML 이 쓰는 것)
void makeFactory(BT::BehaviorTreeFactory & f, Stub * s)
{
  BT::SharedLibrary lib;
  for (const char * p : {"nav2_pipeline_sequence_bt_node", "nav2_rate_controller_bt_node",
      "nav2_recovery_node_bt_node", "nav2_round_robin_node_bt_node"})
  {
    f.registerFromPlugin(lib.getOSName(p));
  }
  using T = BT::NodeType;
  addStub(
    f, s, "PlannerSelector", T::ACTION,
    {"selected_planner", "default_planner", "topic_name"});
  addStub(
    f, s, "ControllerSelector", T::ACTION,
    {"selected_controller", "default_controller", "topic_name"});
  addStub(f, s, "ComputePathToPose", T::ACTION, {"goal", "path", "planner_id"});
  addStub(f, s, "ComputePathThroughPoses", T::ACTION, {"goals", "path", "planner_id"});
  addStub(f, s, "RemovePassedGoals", T::ACTION, {"input_goals", "output_goals", "radius"});
  addStub(f, s, "FollowPath", T::ACTION, {"path", "controller_id"});
  addStub(f, s, "ClearEntireCostmap", T::ACTION, {"service_name"});
  addStub(f, s, "Spin", T::ACTION, {"spin_dist"});
  addStub(f, s, "Wait", T::ACTION, {"wait_duration"});
  addStub(f, s, "BackUp", T::ACTION, {"backup_dist", "backup_speed"});
  addStub(f, s, "IsPathValid", T::CONDITION, {"path"});
  addStub(f, s, "GlobalUpdatedGoal", T::CONDITION, {});
  addStub(f, s, "GoalUpdated", T::CONDITION, {});
  addStub(f, s, "IsTTCBelowThreshold", T::CONDITION, {"threshold", "cooldown", "only_dynamic"});
  s->run_ticks["ComputePathToPose"] = 3;        // 계획 30 ms
  s->run_ticks["ComputePathThroughPoses"] = 3;
  s->run_ticks["FollowPath"] = -1;              // 주행은 시험 동안 끝나지 않는다
  s->cond["IsPathValid"] = true;
  s->cond["GlobalUpdatedGoal"] = false;
  s->cond["GoalUpdated"] = false;
}

// 10 ms 주기로 sec 동안 tick
void tickFor(BT::Tree & tree, double sec)
{
  const auto period = std::chrono::milliseconds(10);
  auto next = std::chrono::steady_clock::now();
  const auto end = next + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(sec));
  while (next < end) {
    tree.tickRoot();
    next += period;
    std::this_thread::sleep_until(next);
  }
}

int plans(Stub & s)
{
  return s.starts["ComputePathToPose"] + s.starts["ComputePathThroughPoses"];
}

class BtReplan : public ::testing::TestWithParam<std::string> {};

TEST_P(BtReplan, ChecksAndReplansAtOneHzWithoutPreemptingFollowPath)
{
  Stub s;
  BT::BehaviorTreeFactory f;
  makeFactory(f, &s);
  auto tree = f.createTreeFromFile(std::string(BT_DIR) + "/" + GetParam());
  const bool ttc = GetParam().find("_no_ttc") == std::string::npos;

  // 1) 경로 유효·TTC 없음 3.25 s: 처음 계획 1 회, 유효성 검사 = 첫 tick (빈 경로) + 1 Hz 3 회,
  //    FollowPath 선점 없음
  tickFor(tree, 3.25);
  EXPECT_EQ(plans(s), 1);
  EXPECT_GE(s.starts["IsPathValid"], 3);
  EXPECT_LE(s.starts["IsPathValid"], 4);
  EXPECT_EQ(s.starts["FollowPath"], 1);
  EXPECT_EQ(s.halts["FollowPath"], 0);

  // 2) 경로가 막히면 다음 주기에 한 번 재계획 (tick 마다가 아니라)
  s.cond["IsPathValid"] = false;
  tickFor(tree, 1.05);
  s.cond["IsPathValid"] = true;
  EXPECT_EQ(plans(s), 2);
  tickFor(tree, 1.0);
  EXPECT_EQ(plans(s), 2);
  EXPECT_EQ(s.halts["FollowPath"], 0);

  // 3) TTC 사건: 주기를 기다리지 않고 곧바로 재계획, 1 Hz 검사 주기는 그대로
  if (ttc) {
    const int checks = s.starts["IsPathValid"];
    s.ttc_once = true;
    tickFor(tree, 0.1);
    EXPECT_EQ(plans(s), 3);
    tickFor(tree, 2.0);
    EXPECT_EQ(plans(s), 3);
    EXPECT_LE(s.starts["IsPathValid"] - checks, 3);
    EXPECT_EQ(s.halts["FollowPath"], 0);
    EXPECT_GE(s.starts["IsTTCBelowThreshold"], 200);   // TTC 조건은 tick 마다 본다
  }
  EXPECT_EQ(s.starts["FollowPath"], 1);
  tree.haltTree();
}

INSTANTIATE_TEST_SUITE_P(
  ShippedTrees, BtReplan,
  ::testing::Values(
    "navigate_to_pose.xml", "navigate_through_poses.xml", "navigate_to_pose_no_ttc.xml",
    "navigate_through_poses_no_ttc.xml"));

// 결함 기전 확인: ReactiveFallback 아래 RateController 는 주기를 잃고 tick 마다 자식을 실행한다
// (이 구조로 되돌리면 위 시험이 실패하는 이유)
TEST(BtReplanMechanism, RateControllerUnderReactiveFallbackRunsEveryTick)
{
  static const char * kXml =
    R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence>
      <ReactiveFallback>
        <Sequence>
          <IsTTCBelowThreshold threshold="2.0"/>
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="AStar"/>
        </Sequence>
        <RateController hz="1.0">
          <IsPathValid path="{path}"/>
        </RateController>
      </ReactiveFallback>
      <FollowPath path="{path}" controller_id="DWA"/>
    </PipelineSequence>
  </BehaviorTree>
</root>)";
  Stub s;
  BT::BehaviorTreeFactory f;
  makeFactory(f, &s);
  s.empty_path_invalid = false;   // 경로 유효 → RateController 자식 SUCCESS (실제 결함 조건)
  auto tree = f.createTreeFromText(kXml);
  tickFor(tree, 1.0);
  EXPECT_GT(s.starts["IsPathValid"], 50);   // 1 Hz 라면 1–2 회
  tree.haltTree();
}

}  // namespace
