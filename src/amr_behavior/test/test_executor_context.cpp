// ExecutorContext 시험: 작업 수락 정책, 상태 보고, 입력 상태, 도크 대응, 출력 훅.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "amr_behavior/bt_nodes/bt_conversions.hpp"
#include "amr_behavior/executor_context.hpp"

using amr_behavior::AcceptPolicy;
using amr_behavior::DockSpec;
using amr_behavior::ExecutorContext;
using amr_behavior::ExecutorHooks;
using amr_behavior::makePose;
using amr_behavior::ObjectQuery;
using amr_msgs::msg::Task;

namespace
{

struct Recorder
{
  std::vector<std::string> phases;
  std::vector<Task> statuses;
  std::vector<std::string> attach;
  std::vector<double> mass;
  std::vector<bool> charging;
  std::vector<std::string> claims;
  std::vector<std::string> logs;

  ExecutorHooks hooks()
  {
    ExecutorHooks h;
    h.publish_phase = [this](const std::string & p) {phases.push_back(p);};
    h.publish_task_status = [this](const Task & t) {statuses.push_back(t);};
    h.publish_payload_attach = [this](const std::string & a) {attach.push_back(a);};
    h.publish_payload_mass = [this](double m) {mass.push_back(m);};
    h.publish_charging = [this](bool c) {charging.push_back(c);};
    h.publish_charger_claim = [this](const std::string & c, double) {claims.push_back(c);};
    h.log_info = [this](const std::string & s) {logs.push_back("I:" + s);};
    h.log_warn = [this](const std::string & s) {logs.push_back("W:" + s);};
    return h;
  }
};

Task makeTask(const std::string & id, const std::string & item = "medium")
{
  Task t;
  t.task_id = id;
  t.item_type = item;
  t.pickup_pose = makePose("", -28.0, 17.0, 0.0);
  t.dropoff_pose = makePose("map", 28.3, 13.1, 0.0);
  return t;
}

class ContextTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctx_ = std::make_shared<ExecutorContext>([this]() {return now_;}, rec_.hooks());
    ctx_->setRobotId("amr_03");
    ctx_->setPayloadTable({{"small", {2.0, 5.0}}, {"medium", {10.0, 10.0}}});
    ctx_->setDocks(
      {DockSpec{"dock_1", "map", -28.29, 17.0, M_PI, std::nullopt},
        DockSpec{"dock_b", "map", 28.29, 13.0, 0.0, std::nullopt},
        DockSpec{"charger_c1", "map", -26.0, -16.64, -M_PI_2, std::nullopt},
        DockSpec{"charger_c2", "map", -22.0, -16.64, -M_PI_2, std::nullopt},
        DockSpec{"charger_c3", "map", -18.0, -16.64, -M_PI_2, std::nullopt}},
      1.8, M_PI);
    ctx_->setPhase(amr_behavior::phase::kIdle);
  }

  double now_{100.0};
  Recorder rec_;
  std::shared_ptr<ExecutorContext> ctx_;
};

}  // namespace

TEST(Phase, KnownNames)
{
  EXPECT_TRUE(amr_behavior::phase::isKnown("RECOVERING"));
  EXPECT_TRUE(amr_behavior::phase::isKnown("UNDOCKING"));   // fleet: undocking → DOCKING
  EXPECT_TRUE(amr_behavior::phase::isKnown("IDLE"));
  EXPECT_FALSE(amr_behavior::phase::isKnown("idle"));
}

TEST_F(ContextTest, AcceptsWhenIdleAndPublishesInProgress)
{
  EXPECT_FALSE(ctx_->hasTask());
  const auto d = ctx_->acceptTask(makeTask("T1"));
  EXPECT_TRUE(d.accepted);
  EXPECT_EQ(d.message, "accepted");
  ASSERT_EQ(rec_.statuses.size(), 1U);
  EXPECT_EQ(rec_.statuses[0].status, Task::STATUS_IN_PROGRESS);
  EXPECT_EQ(rec_.statuses[0].robot_id, "amr_03");
  EXPECT_EQ(rec_.statuses[0].pickup_pose.header.frame_id, "map");   // 빈 frame → map
  EXPECT_TRUE(ctx_->hasTask());
  EXPECT_EQ(ctx_->currentTask()->task_id, "T1");
}

TEST_F(ContextTest, RejectsWhenBusyOrUnsafe)
{
  ASSERT_TRUE(ctx_->acceptTask(makeTask("T1")).accepted);
  EXPECT_EQ(ctx_->acceptTask(makeTask("T2")).message, "busy:T1");
  ASSERT_TRUE(ctx_->reportStatus(Task::STATUS_COMPLETED, ""));

  ctx_->setPhase(amr_behavior::phase::kCharging);
  EXPECT_EQ(ctx_->evaluateTask(makeTask("T3")).message, "busy:charging");
  ctx_->setPhase(amr_behavior::phase::kReturning);
  EXPECT_FALSE(ctx_->evaluateTask(makeTask("T3")).accepted);
  AcceptPolicy policy;
  policy.accept_while_returning = true;
  ctx_->setAcceptPolicy(policy);
  EXPECT_TRUE(ctx_->evaluateTask(makeTask("T3")).accepted);
  ctx_->setPhase(amr_behavior::phase::kIdle);

  ctx_->updateEstop(true);
  EXPECT_EQ(ctx_->evaluateTask(makeTask("T3")).message, "estop");
  ctx_->updateEstop(false);
  ctx_->updateLocalizationLost(true);
  EXPECT_EQ(ctx_->evaluateTask(makeTask("T3")).message, "lost");
  ctx_->updateLocalizationLost(false);
  ctx_->updateBattery(10.0);
  EXPECT_EQ(ctx_->evaluateTask(makeTask("T3")).message, "battery_low");
  ctx_->updateBattery(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(ctx_->battery().has_value());
  EXPECT_EQ(ctx_->evaluateTask(makeTask("")).message, "invalid:task_id");
  Task bad = makeTask("T4");
  bad.pickup_pose.pose.position.x = std::nan("");
  EXPECT_EQ(ctx_->evaluateTask(bad).message, "invalid:pose");
  EXPECT_EQ(ctx_->evaluateTask(makeTask("T5", "huge")).message, "invalid:item_type");
  EXPECT_TRUE(ctx_->evaluateTask(makeTask("T6", "SMALL")).accepted);   // 대소문자 무시
}

TEST_F(ContextTest, ReportStatusTransitions)
{
  EXPECT_FALSE(ctx_->reportStatus(Task::STATUS_COMPLETED, ""));   // 작업 없음
  ASSERT_TRUE(ctx_->acceptTask(makeTask("T1")).accepted);
  EXPECT_TRUE(ctx_->reportStatus(Task::STATUS_IN_PROGRESS, ""));
  EXPECT_TRUE(ctx_->hasTask());
  EXPECT_TRUE(ctx_->reportStatus(Task::STATUS_FAILED, "dock_failed"));
  EXPECT_FALSE(ctx_->hasTask());
  EXPECT_EQ(ctx_->lastFailureReason(), "dock_failed");
  ASSERT_EQ(rec_.statuses.size(), 3U);
  EXPECT_EQ(rec_.statuses.back().status, Task::STATUS_FAILED);
  ASSERT_TRUE(ctx_->acceptTask(makeTask("T2")).accepted);
  EXPECT_TRUE(ctx_->lastFailureReason().empty());
  EXPECT_TRUE(ctx_->reportStatus(Task::STATUS_COMPLETED, ""));
  EXPECT_EQ(rec_.statuses.back().status, Task::STATUS_COMPLETED);
  bool warned = false;
  for (const auto & l : rec_.logs) {
    warned = warned || l.rfind("W:", 0) == 0;
  }
  EXPECT_TRUE(warned);
}

TEST_F(ContextTest, DockResolution)
{
  EXPECT_EQ(ctx_->resolveDock(makePose("map", -27.5, 17.0, 0.0)), "dock_1");   // 패드 중심
  EXPECT_EQ(ctx_->resolveDock(makePose("map", 28.0, 13.2, 1.0)), "dock_b");
  EXPECT_EQ(ctx_->resolveDock(makePose("map", 0.0, 0.0, 0.0)), "");
  ASSERT_TRUE(ctx_->dock("dock_1").has_value());
  EXPECT_FALSE(ctx_->dock("nope").has_value());
  // 방향 허용오차를 쓰면 반대 방향 자세는 대응하지 않는다
  ctx_->setDocks({DockSpec{"dock_1", "map", -28.29, 17.0, M_PI, std::nullopt}}, 1.6, 0.5);
  EXPECT_EQ(ctx_->resolveDock(makePose("map", -28.0, 17.0, 0.0)), "");
  EXPECT_EQ(ctx_->resolveDock(makePose("map", -28.0, 17.0, 3.0)), "dock_1");
}

TEST_F(ContextTest, ObjectDetectionFilters)
{
  ObjectQuery q;
  EXPECT_FALSE(ctx_->objectDetected(q));   // 수신 전
  amr_msgs::msg::DetectedObjectArray arr;
  amr_msgs::msg::DetectedObject o;
  o.class_name = "Box";
  o.confidence = 0.8F;
  o.distance = 1.5F;
  arr.objects.push_back(o);
  ctx_->updateDetectedObjects(arr);
  EXPECT_TRUE(ctx_->objectDetected(q));
  q.max_distance = 1.0;
  EXPECT_FALSE(ctx_->objectDetected(q));
  q.max_distance = 2.0;
  q.min_confidence = 0.9;
  EXPECT_FALSE(ctx_->objectDetected(q));
  q.min_confidence = 0.5;
  q.class_name = "person";
  EXPECT_FALSE(ctx_->objectDetected(q));
  q.class_name = "";   // 모든 클래스
  EXPECT_TRUE(ctx_->objectDetected(q));
  now_ += 2.0;   // max_age 1 s 초과
  EXPECT_FALSE(ctx_->objectDetected(q));
}

TEST_F(ContextTest, MarkerAgeAndYieldPose)
{
  EXPECT_TRUE(std::isinf(ctx_->dockMarkerAge()));
  ctx_->updateDockMarker(makePose("base_link", 0.6, 0.0, M_PI));
  now_ += 0.25;
  EXPECT_NEAR(ctx_->dockMarkerAge(), 0.25, 1e-9);
  geometry_msgs::msg::PoseStamped nan_pose = makePose("base_link", 0.0, 0.0, 0.0);
  nan_pose.pose.position.x = std::nan("");
  ctx_->updateDockMarker(nan_pose);   // 무시
  EXPECT_NEAR(ctx_->dockMarkerAge(), 0.25, 1e-9);

  EXPECT_FALSE(ctx_->yieldPose().has_value());
  ctx_->updateYieldPose(makePose("map", 1.0, 2.0, 0.0));   // hold 전에 도착 (유예 2 s 안)
  now_ += 0.5;
  ctx_->updateTrafficHold(true);
  EXPECT_TRUE(ctx_->trafficHold());
  ASSERT_TRUE(ctx_->yieldPose().has_value());
  EXPECT_DOUBLE_EQ(ctx_->yieldPose()->pose.position.y, 2.0);
  ctx_->updateYieldPose(makePose("", 5.0, 5.0, 0.0));   // frame 없음 → 무시
  EXPECT_DOUBLE_EQ(ctx_->yieldPose()->pose.position.x, 1.0);
  ctx_->updateTrafficHold(false);
  EXPECT_FALSE(ctx_->yieldPose().has_value());
  ctx_->updateYieldPose(makePose("map", 3.0, 3.0, 0.0));
  now_ += 5.0;
  ctx_->updateTrafficHold(true);   // 오래된 양보 자세는 무효
  EXPECT_FALSE(ctx_->yieldPose().has_value());
}

TEST_F(ContextTest, OutputsPublishOnChange)
{
  ASSERT_EQ(rec_.phases.size(), 1U);
  ctx_->setPhase(amr_behavior::phase::kIdle);   // 같은 값 → 발행 안 함
  ctx_->setPhase(amr_behavior::phase::kMoving);
  EXPECT_EQ(rec_.phases.size(), 2U);
  EXPECT_EQ(ctx_->phase(), "MOVING");
  ctx_->attachPayload("large", 25.0);
  EXPECT_EQ(ctx_->attachedPayload(), "large");
  ctx_->detachPayload();
  EXPECT_EQ(rec_.attach, (std::vector<std::string>{"large", ""}));
  EXPECT_EQ(rec_.mass, (std::vector<double>{25.0, 0.0}));
  ctx_->setCharging(true);
  ctx_->setCharging(true);
  ctx_->setCharging(false);
  EXPECT_EQ(rec_.charging, (std::vector<bool>{true, false}));
  EXPECT_FALSE(ctx_->charging());
  EXPECT_EQ(ctx_->robotId(), "amr_03");
  EXPECT_DOUBLE_EQ(ctx_->payloadSpec("MEDIUM")->mass, 10.0);
  EXPECT_FALSE(ctx_->payloadSpec("huge").has_value());
  ctx_->updateBattery(150.0);
  EXPECT_DOUBLE_EQ(*ctx_->battery(), 100.0);
}

TEST(ContextNoHooks, WorksWithoutHooksOrClock)
{
  ExecutorContext ctx(nullptr);
  EXPECT_DOUBLE_EQ(ctx.now(), 0.0);
  ctx.setPhase("IDLE");
  ctx.attachPayload("small", 2.0);
  ctx.setCharging(true);
  ctx.logInfo("x");
  ctx.logWarn("y");
  ctx.detachPayload();
  AcceptPolicy policy;
  policy.allow_undocked_tasks = true;   // 도크 표 없음 → 도킹 없는 작업을 명시적으로 허용
  ctx.setAcceptPolicy(policy);
  // 물품 표 없음 → 종류 검사 생략
  EXPECT_TRUE(ctx.acceptTask(makeTask("T1", "anything")).accepted);
  EXPECT_TRUE(ctx.reportStatus(Task::STATUS_COMPLETED, ""));
  ctx.setChargers({}, 0, 3.0);
  EXPECT_FALSE(ctx.selectCharger().has_value());
  ctx.releaseCharger();
}

TEST_F(ContextTest, TaskPosesMustMatchARegisteredDock)
{
  // 등록 도크와 맞지 않는 자세는 조용히 도킹 생략하지 않고 거절한다 (명시적 오류)
  Task t = makeTask("T1");
  t.pickup_pose = makePose("map", 12.5, 8.0, 1.57);   // amr_fleet 예시 작업 좌표 (도크 아님)
  EXPECT_EQ(ctx_->evaluateTask(t).message, "invalid:no_dock:pickup");
  t = makeTask("T2");
  t.dropoff_pose = makePose("map", 0.0, 0.0, 0.0);
  EXPECT_EQ(ctx_->evaluateTask(t).message, "invalid:no_dock:dropoff");
  // 마커 판 앞(staging 에서 1.69 m) 좌표도 도크로 대응한다
  t.dropoff_pose = makePose("map", -29.9, 17.0, 0.0);
  EXPECT_TRUE(ctx_->evaluateTask(t).accepted);
  // 도킹 없는 작업은 명시적으로 켰을 때만
  AcceptPolicy policy;
  policy.allow_undocked_tasks = true;
  ctx_->setAcceptPolicy(policy);
  t.dropoff_pose = makePose("map", 0.0, 0.0, 0.0);
  EXPECT_TRUE(ctx_->evaluateTask(t).accepted);
}

TEST_F(ContextTest, PayloadOnBoardBlocksNewTasksUntilReturnedOrCleared)
{
  ASSERT_TRUE(ctx_->acceptTask(makeTask("T1")).accepted);
  amr_behavior::PayloadOrigin origin;
  origin.goal = makePose("map", -28.29, 17.0, M_PI);
  origin.dock_id = "dock_1";
  ctx_->attachPayload("medium", 10.0, origin);
  EXPECT_FALSE(ctx_->strandedPayload().has_value());   // 작업 중에는 되돌려 놓기 대상 아님
  ASSERT_TRUE(ctx_->reportStatus(Task::STATUS_FAILED, "dock_failed"));   // 하역 도킹 3회 실패
  EXPECT_TRUE(ctx_->returnPending());
  const auto stranded = ctx_->strandedPayload();
  ASSERT_TRUE(stranded.has_value());
  EXPECT_EQ(stranded->dock_id, "dock_1");
  EXPECT_EQ(stranded->item_type, "medium");
  EXPECT_DOUBLE_EQ(stranded->goal.pose.position.x, -28.29);
  // 물품 위에 새 물품을 싣지 않는다
  EXPECT_EQ(ctx_->evaluateTask(makeTask("T2")).message, "blocked:payload");
  ctx_->markPayloadReturnAttempted();   // 되돌려 놓기도 실패
  EXPECT_FALSE(ctx_->strandedPayload().has_value());   // 다시 돌지 않는다
  EXPECT_EQ(ctx_->evaluateTask(makeTask("T2")).message, "blocked:payload");
  ctx_->detachPayload();   // 작업자가 내림 (clear_payload)
  EXPECT_TRUE(ctx_->evaluateTask(makeTask("T2")).accepted);
  ctx_->markPayloadReturnAttempted();   // 빈 상태에서는 표시하지 않는다
  ctx_->attachPayload("small", 2.0);
  ASSERT_TRUE(ctx_->strandedPayload().has_value());   // 새 물품은 되돌려 놓기 대상
  ctx_->clearReturnPending();
  EXPECT_FALSE(ctx_->returnPending());
}

TEST_F(ContextTest, ChargerAllocationRotatesAndHonoursOtherClaims)
{
  const std::vector<std::string> ids = {"charger_c1", "charger_c2", "charger_c3"};
  ctx_->setChargers(ids, 2, 3.0);   // amr_03 → c3 먼저
  EXPECT_EQ(
    ctx_->chargerPreference(),
    (std::vector<std::string>{"charger_c3", "charger_c1", "charger_c2"}));
  ctx_->updateChargerClaim("amr_01", "charger_c3", 50.0);   // 먼저 점유
  ctx_->updateChargerClaim("amr_03", "charger_c1", 10.0);   // 자기 id → 무시
  auto c = ctx_->selectCharger();
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->id, "charger_c1");
  EXPECT_EQ(ctx_->chargerClaim(), "charger_c1");
  EXPECT_DOUBLE_EQ(ctx_->chargerClaimSince(), now_);
  EXPECT_EQ(rec_.claims.back(), "charger_c1");
  // 같은 시각에 id 가 작은 로봇이 같은 충전소를 점유 → 양보하고 다음 빈 곳
  ctx_->updateChargerClaim("amr_02", "charger_c1", now_);
  c = ctx_->selectCharger();
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->id, "charger_c2");
  // 나중에 점유한 로봇은 이기지 못한다 (자기 점유 유지 — E-stop 재개 포함)
  now_ += 1.0;
  ctx_->updateChargerClaim("amr_04", "charger_c2", now_);
  EXPECT_EQ(ctx_->selectCharger()->id, "charger_c2");
  // 모두 점유 → 없음, 오래된 심장박동은 무효
  ctx_->releaseCharger();
  EXPECT_EQ(rec_.claims.back(), "");
  ctx_->updateChargerClaim("amr_05", "charger_c2", 0.0);
  EXPECT_FALSE(ctx_->selectCharger().has_value());
  now_ += 3.5;   // 모든 심장박동이 claim_timeout 3 s 를 넘김
  ASSERT_TRUE(ctx_->selectCharger().has_value());
  EXPECT_EQ(ctx_->chargerClaim(), "charger_c3");
  ctx_->updateChargerClaim("amr_01", "", 0.0);   // 해제 메시지
  ctx_->releaseCharger();
  ctx_->releaseCharger();   // 두 번째는 발행하지 않는다
  EXPECT_EQ(std::count(rec_.claims.begin(), rec_.claims.end(), ""), 2);
}
