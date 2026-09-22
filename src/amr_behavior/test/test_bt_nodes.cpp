// 자체 BT 노드 단위 시험: 조건 노드가 구독 상태를 반영하는지, 동작·제어·데코레이터 노드 의미.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
#include "amr_behavior/bt_nodes/report_task_status.hpp"
#include "amr_behavior/bt_nodes/resumable_sequence.hpp"
#include "amr_behavior/bt_registry.hpp"
#include "amr_behavior/executor_context.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

using amr_behavior::ExecutorContext;
using amr_behavior::ExecutorHooks;
using amr_behavior::makePose;
using amr_msgs::msg::Task;
using BT::NodeStatus;

namespace
{

class BtNodesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ExecutorHooks hooks;
    hooks.publish_phase = [this](const std::string & p) {phases_.push_back(p);};
    hooks.publish_task_status = [this](const Task & t) {statuses_.push_back(t.status);};
    hooks.publish_payload_attach = [this](const std::string & a) {attach_.push_back(a);};
    hooks.publish_payload_mass = [this](double m) {mass_.push_back(m);};
    hooks.publish_charging = [this](bool c) {charging_.push_back(c);};
    ctx_ = std::make_shared<ExecutorContext>([this]() {return now_;}, hooks);
    ctx_->setPayloadTable({{"small", {2.0, 5.0}}, {"large", {25.0, 15.0}}});
    ctx_->setDocks({amr_behavior::DockSpec{"dock_1", "map", -28.29, 17.0, M_PI}}, 1.6, M_PI);
    amr_behavior::registerCoreNodes(factory_, ctx_);
    blackboard_ = BT::Blackboard::create();
  }

  BT::Tree tree(const std::string & body)
  {
    const std::string xml = "<root main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + body +
      "</BehaviorTree></root>";
    return factory_.createTreeFromText(xml, blackboard_);
  }

  NodeStatus once(const std::string & body)
  {
    auto t = tree(body);
    return t.tickRoot();
  }

  Task task(const std::string & id)
  {
    Task t;
    t.task_id = id;
    t.item_type = "large";
    t.item_mass = 0.0F;
    t.pickup_pose = makePose("map", -27.6, 17.1, 0.0);   // dock_1 근처 → staging 으로 바뀜
    t.dropoff_pose = makePose("map", 5.0, 5.0, 1.0);     // 등록 도크 없음 → 그대로, 도킹 생략
    return t;
  }

  double now_{0.0};
  std::vector<std::string> phases_;
  std::vector<uint8_t> statuses_;
  std::vector<std::string> attach_;
  std::vector<double> mass_;
  std::vector<bool> charging_;
  std::shared_ptr<ExecutorContext> ctx_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
};

}  // namespace

TEST_F(BtNodesTest, ConditionsReflectSubscribedState)
{
  EXPECT_EQ(once("<IsEstopClear/>"), NodeStatus::SUCCESS);
  ctx_->updateEstop(true);
  EXPECT_EQ(once("<IsEstopClear/>"), NodeStatus::FAILURE);

  EXPECT_EQ(once("<IsLocalized/>"), NodeStatus::SUCCESS);
  ctx_->updateLocalizationLost(true);
  EXPECT_EQ(once("<IsLocalized/>"), NodeStatus::FAILURE);

  EXPECT_EQ(once("<IsBatteryOk min_percent=\"20\"/>"), NodeStatus::SUCCESS);   // 미수신 = 정상
  ctx_->updateBattery(15.0);
  EXPECT_EQ(once("<IsBatteryOk min_percent=\"20\"/>"), NodeStatus::FAILURE);
  EXPECT_EQ(once("<IsBatteryOk min_percent=\"10\"/>"), NodeStatus::SUCCESS);

  EXPECT_EQ(once("<IsTrafficHold yield_pose=\"{yp}\"/>"), NodeStatus::FAILURE);
  ctx_->updateTrafficHold(true);
  EXPECT_EQ(once("<IsTrafficHold yield_pose=\"{yp}\"/>"), NodeStatus::SUCCESS);
  EXPECT_TRUE(blackboard_->get<geometry_msgs::msg::PoseStamped>("yp").header.frame_id.empty());

  EXPECT_EQ(once("<IsDockMarkerVisible max_age=\"0.5\"/>"), NodeStatus::FAILURE);
  ctx_->updateDockMarker(makePose("base_link", 0.7, 0.0, M_PI));
  EXPECT_EQ(once("<IsDockMarkerVisible max_age=\"0.5\"/>"), NodeStatus::SUCCESS);
  now_ += 1.0;
  EXPECT_EQ(once("<IsDockMarkerVisible max_age=\"0.5\"/>"), NodeStatus::FAILURE);

  const std::string detect = "<IsObjectDetected class_name=\"box\" max_distance=\"2.0\"/>";
  EXPECT_EQ(once(detect), NodeStatus::FAILURE);
  amr_msgs::msg::DetectedObjectArray arr;
  amr_msgs::msg::DetectedObject o;
  o.class_name = "box";
  o.confidence = 0.9F;
  o.distance = 1.2F;
  arr.objects.push_back(o);
  ctx_->updateDetectedObjects(arr);
  EXPECT_EQ(once(detect), NodeStatus::SUCCESS);
  EXPECT_EQ(once("<IsObjectDetected class_name=\"person\"/>"), NodeStatus::FAILURE);
}

TEST_F(BtNodesTest, IsTaskAssignedExpandsTask)
{
  const std::string node =
    "<IsTaskAssigned task_id=\"{id}\" item_type=\"{it}\" item_mass=\"{im}\""
    " pickup_goal=\"{pg}\" dropoff_goal=\"{dg}\" pickup_dock_id=\"{pd}\""
    " dropoff_dock_id=\"{dd}\"/>";
  EXPECT_EQ(once(node), NodeStatus::FAILURE);
  ASSERT_TRUE(ctx_->acceptTask(task("T9")).accepted);
  EXPECT_EQ(once(node), NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard_->get<std::string>("id"), "T9");
  EXPECT_EQ(blackboard_->get<std::string>("pd"), "dock_1");
  EXPECT_EQ(blackboard_->get<std::string>("dd"), "");
  const auto pg = blackboard_->get<geometry_msgs::msg::PoseStamped>("pg");
  EXPECT_DOUBLE_EQ(pg.pose.position.x, -28.29);   // staging 자세로 교체
  EXPECT_NEAR(amr_behavior::yawOf(pg), M_PI, 1e-9);
  const auto dg = blackboard_->get<geometry_msgs::msg::PoseStamped>("dg");
  EXPECT_DOUBLE_EQ(dg.pose.position.x, 5.0);
  EXPECT_EQ(blackboard_->get<std::string>("it"), "large");
}

TEST_F(BtNodesTest, PhaseStatusAndCharging)
{
  EXPECT_EQ(once("<SetPhase phase=\"MOVING\"/>"), NodeStatus::SUCCESS);
  EXPECT_EQ(once("<SetPhase phase=\"MOVING\"/>"), NodeStatus::SUCCESS);
  EXPECT_EQ(once("<SetPhase phase=\"FLYING\"/>"), NodeStatus::SUCCESS);   // 경고 후 그대로
  EXPECT_EQ(phases_, (std::vector<std::string>{"MOVING", "FLYING"}));

  EXPECT_EQ(once("<ReportTaskStatus status=\"COMPLETED\"/>"), NodeStatus::FAILURE);   // 작업 없음
  EXPECT_FALSE(ctx_->acceptTask(task("T0")).accepted);   // IDLE 이 아님 (FLYING)
  ctx_->setPhase("IDLE");
  ASSERT_TRUE(ctx_->acceptTask(task("T1")).accepted);
  EXPECT_EQ(once("<ReportTaskStatus status=\"BOGUS\"/>"), NodeStatus::FAILURE);
  EXPECT_EQ(once("<ReportTaskStatus status=\"PENDING\"/>"), NodeStatus::SUCCESS);
  EXPECT_EQ(once("<ReportTaskStatus status=\"IN_PROGRESS\"/>"), NodeStatus::SUCCESS);
  EXPECT_EQ(once("<ReportTaskStatus status=\"FAILED\" reason=\"x\"/>"), NodeStatus::SUCCESS);
  EXPECT_EQ(ctx_->lastFailureReason(), "x");
  EXPECT_EQ(
    statuses_, (std::vector<uint8_t>{Task::STATUS_IN_PROGRESS, Task::STATUS_PENDING,
      Task::STATUS_IN_PROGRESS, Task::STATUS_FAILED}));
  uint8_t s = 0;
  EXPECT_TRUE(amr_behavior::ReportTaskStatus::parseStatus("COMPLETED", s));
  EXPECT_EQ(s, Task::STATUS_COMPLETED);

  EXPECT_EQ(once("<SetCharging enable=\"true\"/>"), NodeStatus::SUCCESS);
  EXPECT_EQ(once("<SetCharging enable=\"false\"/>"), NodeStatus::SUCCESS);
  EXPECT_EQ(charging_, (std::vector<bool>{true, false}));
}

TEST_F(BtNodesTest, SimulateLoadWaitsLoadTimeThenPublishes)
{
  auto t = tree(
    "<Sequence><SimulateLoad item_type=\"large\" item_mass=\"0\"/>"
    "<SimulateUnload item_type=\"large\" load_time=\"1.0\"/></Sequence>");
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);
  now_ += 14.9;
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);
  EXPECT_TRUE(attach_.empty());   // 적재 시간(15 s) 전에는 이벤트 없음
  now_ += 0.2;
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);   // 적재 완료, 하역 시작
  EXPECT_EQ(attach_, (std::vector<std::string>{"large"}));
  EXPECT_EQ(mass_, (std::vector<double>{25.0}));
  now_ += 1.1;
  EXPECT_EQ(t.tickRoot(), NodeStatus::SUCCESS);
  EXPECT_EQ(attach_, (std::vector<std::string>{"large", ""}));
  EXPECT_EQ(mass_, (std::vector<double>{25.0, 0.0}));
  EXPECT_EQ(ctx_->attachedPayload(), "");

  // 모르는 종류: 소요 0 s, 질량 지정값
  auto u = tree("<SimulateLoad item_type=\"weird\" item_mass=\"3.5\"/>");
  EXPECT_EQ(u.tickRoot(), NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(mass_.back(), 3.5);

  // halt 되면 이벤트 없음
  auto h = tree("<SimulateLoad item_type=\"small\"/>");
  EXPECT_EQ(h.tickRoot(), NodeStatus::RUNNING);
  h.haltTree();
  EXPECT_EQ(attach_.size(), 3U);
}

TEST_F(BtNodesTest, ResumableSequenceKeepsProgressAcrossHalt)
{
  auto t = tree(
    "<ResumableSequence progress_key=\"step\">"
    "<SetPhase phase=\"MOVING\"/>"
    "<SimulateLoad item_type=\"small\"/>"
    "<SetPhase phase=\"IDLE\"/>"
    "</ResumableSequence>");
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);
  EXPECT_EQ(blackboard_->get<int>("step"), 1);
  t.haltTree();   // E-stop 등으로 중단
  EXPECT_EQ(blackboard_->get<int>("step"), 1);
  phases_.clear();
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);   // 1 단계부터 재개 → MOVING 재발행 없음
  EXPECT_TRUE(phases_.empty());
  now_ += 6.0;
  EXPECT_EQ(t.tickRoot(), NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard_->get<int>("step"), 0);
  EXPECT_EQ(phases_, (std::vector<std::string>{"IDLE"}));

  // 자식 실패 → 진행 위치 초기화
  blackboard_->set<int>("step2", 0);
  auto f = tree(
    "<ResumableSequence progress_key=\"step2\"><AlwaysSuccess/><AlwaysFailure/>"
    "</ResumableSequence>");
  EXPECT_EQ(f.tickRoot(), NodeStatus::FAILURE);
  EXPECT_EQ(blackboard_->get<int>("step2"), 0);

  // 문자열/범위 밖 진행 값도 안전하게 처리
  blackboard_->set<std::string>("step3", "1");
  EXPECT_EQ(amr_behavior::ResumableSequence::readProgress(*blackboard_, "step3"), 1);
  blackboard_->set<std::string>("step4", "abc");
  EXPECT_EQ(amr_behavior::ResumableSequence::readProgress(*blackboard_, "step4"), 0);
  EXPECT_EQ(amr_behavior::ResumableSequence::readProgress(*blackboard_, "missing"), 0);
  blackboard_->set<int>("step5", 99);
  auto r = tree(
    "<ResumableSequence progress_key=\"step5\"><AlwaysSuccess/></ResumableSequence>");
  EXPECT_EQ(r.tickRoot(), NodeStatus::SUCCESS);
}

TEST_F(BtNodesTest, KeepRunningUntilSuccessAndSetFailReason)
{
  auto t = tree(
    "<KeepRunningUntilSuccess><IsEstopClear/></KeepRunningUntilSuccess>");
  ctx_->updateEstop(true);
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);
  EXPECT_EQ(t.tickRoot(), NodeStatus::RUNNING);
  ctx_->updateEstop(false);
  EXPECT_EQ(t.tickRoot(), NodeStatus::SUCCESS);

  auto f = tree(
    "<SetFailReason reason=\"dock_failed\" fail_reason=\"{why}\"><AlwaysFailure/>"
    "</SetFailReason>");
  EXPECT_EQ(f.tickRoot(), NodeStatus::FAILURE);
  EXPECT_EQ(blackboard_->get<std::string>("why"), "dock_failed");
  auto s = tree(
    "<SetFailReason reason=\"other\" fail_reason=\"{why}\"><AlwaysSuccess/></SetFailReason>");
  EXPECT_EQ(s.tickRoot(), NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard_->get<std::string>("why"), "dock_failed");
}

TEST(BtConversions, PoseLiteral)
{
  const auto p = BT::convertFromString<geometry_msgs::msg::PoseStamped>("1.5;-2;0.5");
  EXPECT_EQ(p.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(p.pose.position.x, 1.5);
  EXPECT_NEAR(amr_behavior::yawOf(p), 0.5, 1e-12);
  const auto q = BT::convertFromString<geometry_msgs::msg::PoseStamped>("odom;1;2;3");
  EXPECT_EQ(q.header.frame_id, "odom");
  EXPECT_THROW(BT::convertFromString<geometry_msgs::msg::PoseStamped>("1;2"), BT::RuntimeError);
}
