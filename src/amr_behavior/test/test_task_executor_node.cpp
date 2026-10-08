// task_executor_node 통합 시험: 실제 트리 XML + 프로세스 안 모의 Nav2/도킹 서버로 assign_task →
// 대기-이동-인식-작업-복귀 → task_status COMPLETED, 실행 중 재할당 거절, Groot 퍼블리셔 기동
// (여는 데 실패해도 실행기는 계속), clear_payload, 도크 없는 작업 거절, 충전소 점유 메시지,
// traffic/hold latched 구독과 첫 값 전 규칙 (hold_failsafe.md), 첫 값 감시 — 매칭 수 기록 ·
// 구독 재생성 · 경보 (hold_failsafe.md §7, TrafficLinkWatch 단위 시험 포함).
#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "amr_behavior/task_executor_node.hpp"
#include "amr_msgs/action/dock.hpp"
#include "amr_msgs/msg/detected_object_array.hpp"
#include "amr_msgs/msg/task.hpp"
#include "amr_msgs/srv/assign_task.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "spin_thread.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using amr_msgs::msg::Task;

namespace
{

/// 즉시 성공하는 액션 서버 (goal 수 기록).
template<class ActionT>
class InstantServer
{
public:
  InstantServer(const rclcpp::Node::SharedPtr & node, const std::string & name)
  {
    server_ = rclcpp_action::create_server<ActionT>(
      node, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
        ++goals;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle) {
        std::thread(
          [handle]() {
            std::this_thread::sleep_for(30ms);
            auto result = std::make_shared<typename ActionT::Result>();
            if constexpr (std::is_same_v<ActionT, amr_msgs::action::Dock>) {
              result->success = true;
              result->attempts_used = 1;
            }
            handle->succeed(result);
          }).detach();
      });
  }

  std::atomic<int> goals{0};

private:
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
};

class ExecutorNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    const std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter(
        "bt_xml", std::string(AMR_BEHAVIOR_SOURCE_DIR) + "/behavior_trees/task_executor.xml"),
      rclcpp::Parameter("robot_id", "amr_07"),
      rclcpp::Parameter("groot.publisher_port", 16766),
      rclcpp::Parameter("groot.server_port", 16767),
      rclcpp::Parameter("payload.medium.load_time", 0.2),
      rclcpp::Parameter("perception_timeout_ms", 2000),
      rclcpp::Parameter("server_wait_timeout_ms", 3000),
      rclcpp::Parameter("docks.ids", std::vector<std::string>{"dock_1", "dock_a", "bad"}),
      rclcpp::Parameter("docks.dock_1.staging", std::vector<double>{-28.29, 17.0, M_PI}),
      rclcpp::Parameter("docks.dock_a.staging", std::vector<double>{28.29, 17.0, 0.0}),
      rclcpp::Parameter("docks.bad.staging", std::vector<double>{1.0}),
      rclcpp::Parameter("charger_dock_ids", std::vector<std::string>{"missing_charger"}),
      rclcpp::Parameter("charger_claims_topic", "fleet_test/charger_claims"),
      rclcpp::Parameter("bt_log_file", "/tmp/amr_behavior_test_executor.fbl"),
    };
    executor_ = std::make_shared<amr_behavior::TaskExecutorNode>(
      rclcpp::NodeOptions().parameter_overrides(overrides));
    executor_->init();

    world_ = std::make_shared<rclcpp::Node>("mock_world");
    nav_ = std::make_unique<InstantServer<nav2_msgs::action::NavigateToPose>>(
      world_, "navigate_to_pose");
    dock_ = std::make_unique<InstantServer<amr_msgs::action::Dock>>(world_, "dock");
    spin_ = std::make_unique<InstantServer<nav2_msgs::action::Spin>>(world_, "spin");
    backup_ = std::make_unique<InstantServer<nav2_msgs::action::BackUp>>(world_, "backup");
    wait_ = std::make_unique<InstantServer<nav2_msgs::action::Wait>>(world_, "wait");
    objects_pub_ = world_->create_publisher<amr_msgs::msg::DetectedObjectArray>(
      "perception/detected_objects", rclcpp::SensorDataQoS());
    battery_pub_ = world_->create_publisher<sensor_msgs::msg::BatteryState>(
      "battery_state", rclcpp::QoS(10));
    estop_pub_ = world_->create_publisher<std_msgs::msg::Bool>(
      "safety/estop_active", rclcpp::QoS(1).reliable().transient_local());
    box_timer_ = world_->create_wall_timer(
      100ms, [this]() {
        amr_msgs::msg::DetectedObjectArray arr;
        amr_msgs::msg::DetectedObject o;
        o.class_name = "box";
        o.confidence = 0.9F;
        o.distance = 1.0F;
        arr.objects.push_back(o);
        objects_pub_->publish(arr);
      });
    phase_sub_ = world_->create_subscription<std_msgs::msg::String>(
      "executor/phase", rclcpp::QoS(10).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        phases_.push_back(msg->data);
      });
    status_sub_ = world_->create_subscription<Task>(
      "task_status", rclcpp::QoS(10), [this](const Task::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        statuses_.push_back(msg->status);
      });
    assign_ = world_->create_client<amr_msgs::srv::AssignTask>("assign_task");

    exec_.add_node(executor_);
    exec_.add_node(world_);
    spinner_ = std::make_unique<amr_behavior_test::SpinThread>(exec_);
    ASSERT_TRUE(assign_->wait_for_service(5s));
  }

  void TearDown() override
  {
    spinner_.reset();
  }

  amr_msgs::srv::AssignTask::Response::SharedPtr assign(const std::string & id)
  {
    auto req = std::make_shared<amr_msgs::srv::AssignTask::Request>();
    req->task.task_id = id;
    req->task.item_type = "medium";
    req->task.header.stamp = world_->now();
    req->task.pickup_pose.header.frame_id = "map";
    req->task.pickup_pose.pose.position.x = -27.5;
    req->task.pickup_pose.pose.position.y = 17.0;
    req->task.pickup_pose.pose.orientation.w = 1.0;
    req->task.dropoff_pose = req->task.pickup_pose;
    req->task.dropoff_pose.pose.position.x = 27.5;
    auto future = assign_->async_send_request(req);
    if (future.wait_for(5s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  bool waitFor(const std::function<bool()> & cond, std::chrono::seconds limit)
  {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      if (cond()) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return cond();
  }

  std::shared_ptr<amr_behavior::TaskExecutorNode> executor_;
  rclcpp::Node::SharedPtr world_;
  std::unique_ptr<InstantServer<nav2_msgs::action::NavigateToPose>> nav_;
  std::unique_ptr<InstantServer<amr_msgs::action::Dock>> dock_;
  std::unique_ptr<InstantServer<nav2_msgs::action::Spin>> spin_;
  std::unique_ptr<InstantServer<nav2_msgs::action::BackUp>> backup_;
  std::unique_ptr<InstantServer<nav2_msgs::action::Wait>> wait_;
  rclcpp::Publisher<amr_msgs::msg::DetectedObjectArray>::SharedPtr objects_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::TimerBase::SharedPtr box_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;
  rclcpp::Subscription<Task>::SharedPtr status_sub_;
  rclcpp::Client<amr_msgs::srv::AssignTask>::SharedPtr assign_;
  rclcpp::executors::MultiThreadedExecutor exec_{rclcpp::ExecutorOptions(), 2};
  std::unique_ptr<amr_behavior_test::SpinThread> spinner_;
  std::mutex mutex_;
  std::vector<std::string> phases_;
  std::vector<uint8_t> statuses_;
};

}  // namespace

TEST_F(ExecutorNodeTest, AssignRunsFullTaskAndRejectsWhileBusy)
{
  EXPECT_TRUE(executor_->grootActive());
  ASSERT_TRUE(waitFor([this]() {return executor_->context()->phase() == "IDLE";}, 5s));
  // 구독자가 latched IDLE 을 받을 때까지 기다린다 (서비스 발견이 토픽 발견 완료를 뜻하지 않는다)
  ASSERT_TRUE(
    waitFor(
      [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        return !phases_.empty() && phases_.back() == "IDLE";
      }, 5s));
  auto res = assign("T-100");
  ASSERT_TRUE(res);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->robot_id, "amr_07");
  auto busy = assign("T-101");
  ASSERT_TRUE(busy);
  EXPECT_FALSE(busy->success);
  EXPECT_EQ(busy->message.rfind("busy", 0), 0U) << busy->message;

  ASSERT_TRUE(
    waitFor(
      [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        return !statuses_.empty() && statuses_.back() == Task::STATUS_COMPLETED &&
        !phases_.empty() && phases_.back() == "IDLE" && phases_.size() > 2U;
      }, 60s));
  std::lock_guard<std::mutex> lock(mutex_);
  const std::vector<std::string> expected = {
    "IDLE", "MOVING", "PERCEIVING", "DOCKING", "LOADING", "UNDOCKING", "MOVING", "DOCKING",
    "UNLOADING", "UNDOCKING", "RETURNING", "IDLE"};
  EXPECT_EQ(phases_, expected);
  EXPECT_EQ(statuses_, (std::vector<uint8_t>{Task::STATUS_IN_PROGRESS, Task::STATUS_COMPLETED}));
  EXPECT_EQ(nav_->goals.load(), 3);
  EXPECT_EQ(dock_->goals.load(), 2);
  EXPECT_EQ(backup_->goals.load(), 2);   // 적재·하역 뒤 이탈
}

TEST_F(ExecutorNodeTest, RejectsTaskPosesThatMatchNoDock)
{
  auto req = std::make_shared<amr_msgs::srv::AssignTask::Request>();
  req->task.task_id = "T-300";
  req->task.item_type = "medium";
  req->task.pickup_pose.header.frame_id = "map";
  req->task.pickup_pose.pose.position.x = 12.5;   // amr_fleet 예시 좌표 (도크 아님)
  req->task.pickup_pose.pose.position.y = 8.0;
  req->task.pickup_pose.pose.orientation.w = 1.0;
  req->task.dropoff_pose = req->task.pickup_pose;
  req->task.dropoff_pose.pose.position.x = 27.5;
  req->task.dropoff_pose.pose.position.y = 17.0;
  auto future = assign_->async_send_request(req);
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  const auto res = future.get();
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->message, "invalid:no_dock:pickup");
}

TEST_F(ExecutorNodeTest, ClearPayloadServiceUnblocksTheRobot)
{
  auto client = world_->create_client<std_srvs::srv::Trigger>("clear_payload");
  ASSERT_TRUE(client->wait_for_service(5s));
  auto call = [&client]() {
      auto f = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      EXPECT_EQ(f.wait_for(5s), std::future_status::ready);
      return f.get();
    };
  EXPECT_EQ(call()->message, "empty");
  executor_->context()->attachPayload("large", 25.0);   // 되돌려 놓지 못한 물품
  auto res = assign("T-400");
  ASSERT_TRUE(res);
  EXPECT_EQ(res->message, "blocked:payload");
  auto cleared = call();
  EXPECT_TRUE(cleared->success);
  EXPECT_EQ(cleared->message, "cleared:large");
  EXPECT_TRUE(executor_->context()->attachedPayload().empty());
}

TEST_F(ExecutorNodeTest, GrootFailureDoesNotStopTheExecutor)
{
  // Groot 퍼블리셔를 열 수 없으면(포트 사용 중, 또는 BT.CPP v3 의 프로세스당 PublisherZMQ
  // 1 개 제한) 경고 후 시각화 없이 계속한다. 로봇마다 다른 포트가 함께 열리는지는 프로세스를
  // 나눠야 하므로 test_groot_ports.py 가 실제 실행 파일 3 개로 확인한다.
  auto make = [](const std::string & ns, int pub, int srv) {
      const std::vector<rclcpp::Parameter> overrides = {
        rclcpp::Parameter(
          "bt_xml", std::string(AMR_BEHAVIOR_SOURCE_DIR) + "/behavior_trees/task_executor.xml"),
        rclcpp::Parameter("groot.publisher_port", pub),
        rclcpp::Parameter("groot.server_port", srv),
        rclcpp::Parameter("charger_claims_topic", ""),
      };
      auto node = std::make_shared<amr_behavior::TaskExecutorNode>(
        rclcpp::NodeOptions().parameter_overrides(overrides).arguments(
          {"--ros-args", "-r", "__ns:=/" + ns}));
      node->init();
      return node;
    };
  EXPECT_TRUE(executor_->grootActive());
  auto clash = make("amr_gr4", 16766, 16767);
  EXPECT_FALSE(clash->grootActive());
  EXPECT_NE(clash->tree(), nullptr);
  clash->tickOnce();
  EXPECT_EQ(clash->context()->phase(), "IDLE");
}

TEST(ChargerClaimCodec, RoundTripAndRobotIndex)
{
  const std::string json = amr_behavior::encodeChargerClaim("amr_02", "charger_c3", 12.5);
  std::string robot;
  std::string charger;
  double since = 0.0;
  ASSERT_TRUE(amr_behavior::decodeChargerClaim(json, robot, charger, since));
  EXPECT_EQ(robot, "amr_02");
  EXPECT_EQ(charger, "charger_c3");
  EXPECT_DOUBLE_EQ(since, 12.5);
  ASSERT_TRUE(
    amr_behavior::decodeChargerClaim(
      amr_behavior::encodeChargerClaim("amr_02", "", 0.0), robot, charger, since));
  EXPECT_TRUE(charger.empty());   // 해제
  EXPECT_FALSE(amr_behavior::decodeChargerClaim("{\"charger\": \"c1\"}", robot, charger, since));
  EXPECT_FALSE(amr_behavior::decodeChargerClaim("{\"robot\": \"a\"}", robot, charger, since));
  EXPECT_FALSE(
    amr_behavior::decodeChargerClaim(
      "{\"robot\": \"a\", \"charger\": \"c\", \"since\": 1e999999}", robot, charger, since));
  // 무손실 왕복: 같은 1 ms 안의 두 점유(에포크 초)도 받는 쪽 비교가 보내는 쪽과 같아야 한다
  for (const double t : {1790083158.7019653, 1790083158.7016001, 0.1 + 0.2}) {
    ASSERT_TRUE(
      amr_behavior::decodeChargerClaim(
        amr_behavior::encodeChargerClaim("amr_04", "charger_c3", t), robot, charger, since));
    EXPECT_EQ(since, t);
  }
  EXPECT_EQ(amr_behavior::robotIndexFromId("amr_01"), 0);
  EXPECT_EQ(amr_behavior::robotIndexFromId("amr_05"), 4);
  EXPECT_EQ(amr_behavior::robotIndexFromId("robot"), 0);
  EXPECT_EQ(amr_behavior::robotIndexFromId(""), 0);
  EXPECT_EQ(amr_behavior::robotIndexFromId("amr_1234567"), 0);   // 비정상적으로 긴 번호
}

TEST_F(ExecutorNodeTest, ChargerClaimsAreSharedBetweenExecutors)
{
  // 두 실행기가 전역 점유 토픽으로 충전소를 나눈다: amr_01 이 c1 을 점유하면 amr_04 는 c1 을
  // 고르지 않는다
  auto make = [](const std::string & id) {
      const std::vector<rclcpp::Parameter> overrides = {
        rclcpp::Parameter(
          "bt_xml", std::string(AMR_BEHAVIOR_SOURCE_DIR) + "/behavior_trees/task_executor.xml"),
        rclcpp::Parameter("robot_id", id),
        rclcpp::Parameter("groot.enabled", false),
        rclcpp::Parameter("docks.ids", std::vector<std::string>{"charger_c1", "charger_c2"}),
        rclcpp::Parameter("docks.charger_c1.staging", std::vector<double>{-26.0, -16.64, -1.57}),
        rclcpp::Parameter("docks.charger_c2.staging", std::vector<double>{-22.0, -16.64, -1.57}),
        rclcpp::Parameter(
          "charger_dock_ids", std::vector<std::string>{"charger_c1", "charger_c2"}),
        rclcpp::Parameter("charger_claims_topic", "/claims_test/charger_claims"),
      };
      auto node = std::make_shared<amr_behavior::TaskExecutorNode>(
        rclcpp::NodeOptions().parameter_overrides(overrides).arguments(
          {"--ros-args", "-r", "__ns:=/" + id}));
      return node;
    };
  auto a = make("amr_01");
  auto b = make("amr_04");   // 번호 3 → 충전소 2 개 중 3 mod 2 = 1 → 선호 c2, c1
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(a);
  exec.add_node(b);
  amr_behavior_test::SpinThread spinner(exec);
  EXPECT_EQ(
    b->context()->chargerPreference(),
    (std::vector<std::string>{"charger_c2", "charger_c1"}));
  // c2 는 다른 로봇(amr_09)이 먼저 점유 → amr_04 는 c1 을 원하지만 amr_01 이 점유하면 양보해야 한다
  ASSERT_TRUE(a->context()->selectCharger().has_value());
  EXPECT_EQ(a->context()->chargerClaim(), "charger_c1");
  const auto end = std::chrono::steady_clock::now() + 5s;
  std::optional<amr_behavior::DockSpec> got;
  while (std::chrono::steady_clock::now() < end) {
    b->context()->updateChargerClaim("amr_09", "charger_c2", -100.0);
    got = b->context()->selectCharger();
    if (!got) {
      break;   // amr_01 의 c1 점유를 들었다 → 빈 곳 없음
    }
    b->context()->releaseCharger();
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_FALSE(got.has_value());
}

TEST_F(ExecutorNodeTest, ChargerClaimHeartbeatFollowsNodeClock)
{
  // 점유 심장박동은 노드 시계(use_sim_time 이면 /clock)로 낸다 — 받는 쪽 만료와 같은 시간 축
  // (보너스 4 P0, fleet-distributed/p0.md §2). sim 시각이 멈추면 벽시계가 흘러도 내지 않는다.
  const std::string topic = "/claims_sim_test/charger_claims";
  auto aux = std::make_shared<rclcpp::Node>("claims_sim_aux");
  auto clock_pub = aux->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  std::atomic<int> beats{0};
  auto sub = aux->create_subscription<std_msgs::msg::String>(
    topic, rclcpp::QoS(10), [&beats](const std_msgs::msg::String::SharedPtr) {++beats;});
  const std::vector<rclcpp::Parameter> overrides = {
    rclcpp::Parameter(
      "bt_xml", std::string(AMR_BEHAVIOR_SOURCE_DIR) + "/behavior_trees/task_executor.xml"),
    rclcpp::Parameter("robot_id", "amr_02"),
    rclcpp::Parameter("use_sim_time", true),
    rclcpp::Parameter("groot.enabled", false),
    rclcpp::Parameter("docks.ids", std::vector<std::string>{"charger_c1"}),
    rclcpp::Parameter("docks.charger_c1.staging", std::vector<double>{-26.0, -16.64, -1.57}),
    rclcpp::Parameter("charger_dock_ids", std::vector<std::string>{"charger_c1"}),
    rclcpp::Parameter("charger_claims_topic", topic),
  };
  auto node = std::make_shared<amr_behavior::TaskExecutorNode>(
    rclcpp::NodeOptions().parameter_overrides(overrides).arguments(
      {"--ros-args", "-r", "__ns:=/amr_02"}));
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(aux);
  amr_behavior_test::SpinThread spinner(exec);
  auto publish_clock = [&clock_pub](int32_t sec) {
      rosgraph_msgs::msg::Clock c;
      c.clock.sec = sec;
      clock_pub->publish(c);
    };
  auto wait = [](const std::function<bool()> & cond, std::chrono::milliseconds limit) {
      const auto end = std::chrono::steady_clock::now() + limit;
      while (std::chrono::steady_clock::now() < end) {
        if (cond()) {
          return true;
        }
        std::this_thread::sleep_for(20ms);
      }
      return cond();
    };
  ASSERT_TRUE(
    wait(
      [&]() {
        publish_clock(10);
        return node->now().seconds() == 10.0 && sub->get_publisher_count() > 0;
      }, 5000ms));
  // 점유 바뀜 → 한 번 바로 낸다
  ASSERT_TRUE(node->context()->selectCharger().has_value());
  ASSERT_TRUE(wait([&]() {return beats.load() >= 1;}, 3000ms));
  const int frozen = beats.load();
  std::this_thread::sleep_for(2500ms);   // 벽시계 2.5 s, sim 은 10 s 에 정지
  EXPECT_EQ(beats.load(), frozen) << "sim 시각이 멈췄는데 심장박동이 나갔다 (벽시계 타이머?)";
  for (int32_t sec = 11; sec <= 14; ++sec) {   // sim 1 s 씩 4 번
    publish_clock(sec);
    std::this_thread::sleep_for(150ms);
  }
  EXPECT_TRUE(wait([&]() {return beats.load() >= frozen + 2;}, 3000ms))
    << "sim 시각이 흘렀는데 심장박동이 없다: " << beats.load() - frozen;
}

TEST_F(ExecutorNodeTest, EstopAndBatteryGateAcceptance)
{
  std_msgs::msg::Bool on;
  on.data = true;
  estop_pub_->publish(on);
  ASSERT_TRUE(waitFor([this]() {return executor_->context()->estopActive();}, 5s));
  auto res = assign("T-200");
  ASSERT_TRUE(res);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->message, "estop");
  std_msgs::msg::Bool off;
  estop_pub_->publish(off);
  sensor_msgs::msg::BatteryState battery;
  battery.percentage = 0.1F;   // REP: 0~1
  battery_pub_->publish(battery);
  ASSERT_TRUE(
    waitFor(
      [this]() {
        const auto b = executor_->context()->battery();
        return !executor_->context()->estopActive() && b && *b < 11.0;
      }, 5s));
  res = assign("T-201");
  ASSERT_TRUE(res);
  EXPECT_EQ(res->message, "battery_low");
  EXPECT_NE(executor_->tree(), nullptr);
}

namespace
{

/// traffic/hold 시험용 실행기 (네임스페이스 /<ns>, Groot·점유 공유 끔).
/// init() 로 tick 타이머까지 띄운다.
std::shared_ptr<amr_behavior::TaskExecutorNode> makeTrafficExecutor(
  const std::string & ns, double grace_s, const std::string & policy = "auto",
  const std::vector<rclcpp::Parameter> & extra = {})
{
  std::vector<rclcpp::Parameter> overrides = {
    rclcpp::Parameter(
      "bt_xml", std::string(AMR_BEHAVIOR_SOURCE_DIR) + "/behavior_trees/task_executor.xml"),
    rclcpp::Parameter("groot.enabled", false),
    rclcpp::Parameter("charger_claims_topic", ""),
    rclcpp::Parameter("traffic_hold.policy", policy),
    rclcpp::Parameter("traffic_hold.discovery_grace_s", grace_s),
  };
  overrides.insert(overrides.end(), extra.begin(), extra.end());
  auto node = std::make_shared<amr_behavior::TaskExecutorNode>(
    rclcpp::NodeOptions().parameter_overrides(overrides).arguments(
      {"--ros-args", "-r", "__ns:=/" + ns}));
  node->init();
  return node;
}

/// traffic_manager_node 의 LATCHED_QOS 와 같다 (reliable + transient_local, depth 1).
rclcpp::QoS tmLatchedQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

}  // namespace

TEST_F(ExecutorNodeTest, LateExecutorReceivesLatchedHoldAndPocket)
{
  using amr_behavior::TrafficHoldState;
  // 교통 관리자가 hold=true 와 포켓을 먼저 낸다 (바뀔 때만 발행 — 다시 내지 않는다)
  auto tm = std::make_shared<rclcpp::Node>("fake_traffic_manager");
  auto hold_pub = tm->create_publisher<std_msgs::msg::Bool>("/amr_11/traffic/hold", tmLatchedQos());
  auto pose_pub = tm->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/amr_11/traffic/yield_pose", tmLatchedQos());
  geometry_msgs::msg::PoseStamped pocket;
  pocket.header.frame_id = "map";
  pocket.pose.position.x = 18.0;
  pocket.pose.position.y = 8.0;
  pocket.pose.orientation.w = 1.0;
  pose_pub->publish(pocket);
  std_msgs::msg::Bool on;
  on.data = true;
  hold_pub->publish(on);
  // 그 뒤에 뜬 실행기 (재기동 · 늦은 매칭). 대조: 같은 때 만든 volatile 구독(수정 전 QoS)
  auto late = makeTrafficExecutor("amr_11", 0.0);
  std::atomic<int> volatile_got{0};
  auto probe = tm->create_subscription<std_msgs::msg::Bool>(
    "/amr_11/traffic/hold", rclcpp::QoS(10),
    [&volatile_got](const std_msgs::msg::Bool::SharedPtr) {++volatile_got;});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(tm);
  exec.add_node(late);
  amr_behavior_test::SpinThread spinner(exec);
  ASSERT_TRUE(
    waitFor(
      [&late]() {return late->context()->trafficHoldState() == TrafficHoldState::kHold;}, 5s));
  EXPECT_TRUE(late->context()->trafficHold());
  ASSERT_TRUE(waitFor([&late]() {return late->context()->yieldPose().has_value();}, 5s));
  EXPECT_DOUBLE_EQ(late->context()->yieldPose()->pose.position.x, 18.0);
  // volatile 구독은 짝이 맺어져도 지난 표본을 받지 못한다 → 수정 전에는 hold=true 를 놓쳤다
  ASSERT_TRUE(waitFor([&probe]() {return probe->get_publisher_count() == 1U;}, 5s));
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(volatile_got.load(), 0);
  // 첫 값 감시: 받은 뒤에도 매칭 수는 기록하고, 재생성 · 경보는 없다 (hold_failsafe.md §7)
  ASSERT_TRUE(waitFor([&late]() {return late->context()->trafficLink().matched() == 1U;}, 5s));
  EXPECT_EQ(late->context()->trafficLink().resubscribes(), 0);
  EXPECT_EQ(late->context()->trafficLink().alarms(), 0);
}

TEST_F(ExecutorNodeTest, AwaitsFirstValueWhileTrafficPublisherExists)
{
  using amr_behavior::TrafficHoldState;
  // 교통 관리자는 떠 있지만 이 로봇에 대해 아직 아무것도 내지 않았다 → 허가 없음 = 대기
  auto tm = std::make_shared<rclcpp::Node>("fake_traffic_manager_silent");
  auto hold_pub = tm->create_publisher<std_msgs::msg::Bool>("/amr_12/traffic/hold", tmLatchedQos());
  auto ex = makeTrafficExecutor("amr_12", 0.0);   // 발견 유예 없음: 발행자로만 판정
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(tm);
  exec.add_node(ex);
  amr_behavior_test::SpinThread spinner(exec);
  ASSERT_TRUE(
    waitFor(
      [&ex]() {return ex->context()->trafficHoldState() == TrafficHoldState::kAwaiting;}, 5s));
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(ex->context()->trafficHoldState(), TrafficHoldState::kAwaiting);
  EXPECT_TRUE(ex->context()->trafficHold());
  std_msgs::msg::Bool off;
  hold_pub->publish(off);
  ASSERT_TRUE(
    waitFor([&ex]() {return ex->context()->trafficHoldState() == TrafficHoldState::kClear;}, 5s));
  EXPECT_FALSE(ex->context()->trafficHold());
}

TEST_F(ExecutorNodeTest, WithoutTrafficManagerProceedsAfterDiscoveryGrace)
{
  using amr_behavior::TrafficHoldState;
  // traffic_manager 가 없는 구성 (단일 로봇 시나리오): 발견 유예 동안만 대기,
  // 그 뒤 기존 동작(통과). 모르는 정책 문자열은 auto 로 본다
  auto ex = makeTrafficExecutor("amr_13", 1.0, "bogus");
  EXPECT_EQ(ex->context()->trafficHoldState(), TrafficHoldState::kAwaiting);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(ex);
  amr_behavior_test::SpinThread spinner(exec);
  ASSERT_TRUE(
    waitFor(
      [&ex]() {return ex->context()->trafficHoldState() == TrafficHoldState::kUnmanaged;}, 5s));
  EXPECT_FALSE(ex->context()->trafficHold());
}

TEST_F(ExecutorNodeTest, RequiredPolicyWaitsWithoutPublisher)
{
  using amr_behavior::TrafficHoldState;
  auto ex = makeTrafficExecutor("amr_14", 0.0, "required");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(ex);
  amr_behavior_test::SpinThread spinner(exec);
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(ex->context()->trafficHoldState(), TrafficHoldState::kAwaiting);
  EXPECT_TRUE(ex->context()->trafficHold());
  // 첫 값 감시 기본값 (hold_failsafe.md §7 D11·D12): 재생성 30 s, 경보 120 s
  EXPECT_DOUBLE_EQ(ex->context()->trafficLink().config().resubscribe_after_s, 30.0);
  EXPECT_DOUBLE_EQ(ex->context()->trafficLink().config().alarm_after_s, 120.0);
  EXPECT_EQ(ex->context()->trafficLink().resubscribes(), 0);
}

namespace
{

/// 토픽의 구독 끝점 GID 목록 (재생성 확인: 새 DataReader 는 새 GID).
std::vector<std::array<uint8_t, RMW_GID_STORAGE_SIZE>> subscriptionGids(
  const rclcpp::Node::SharedPtr & node, const std::string & topic)
{
  std::vector<std::array<uint8_t, RMW_GID_STORAGE_SIZE>> out;
  for (const auto & info : node->get_subscriptions_info_by_topic(topic)) {
    out.push_back(info.endpoint_gid());
  }
  return out;
}

}  // namespace

TEST_F(ExecutorNodeTest, ResubscribesWhileMatchedWithoutValueThenAlarmsOnce)
{
  using amr_behavior::TrafficHoldState;
  // 관리자 발행자와 매칭됐는데 값이 오지 않는다 (시나리오 12 의 늦은 참여자) →
  // resubscribe_after_s 뒤 구독 재생성, awaiting 이 alarm_after_s 를 넘으면 경보 한 번.
  // 값이 오면(재생성한 구독으로) 멈춘다
  auto tm = std::make_shared<rclcpp::Node>("fake_traffic_manager_mute");
  auto hold_pub = tm->create_publisher<std_msgs::msg::Bool>("/amr_15/traffic/hold", tmLatchedQos());
  auto pose_pub = tm->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/amr_15/traffic/yield_pose", tmLatchedQos());
  auto ex = makeTrafficExecutor(
    "amr_15", 0.0, "auto",
    {rclcpp::Parameter("traffic_hold.resubscribe_after_s", 0.6),
      rclcpp::Parameter("traffic_hold.alarm_after_s", 1.5)});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(tm);
  exec.add_node(ex);
  amr_behavior_test::SpinThread spinner(exec);
  const auto link = [&ex]() {return ex->context()->trafficLink();};
  ASSERT_TRUE(waitFor([&link]() {return link().matched() == 1U;}, 5s));
  EXPECT_EQ(ex->context()->trafficHoldState(), TrafficHoldState::kAwaiting);
  const auto hold_gids = subscriptionGids(tm, "/amr_15/traffic/hold");
  const auto pose_gids = subscriptionGids(tm, "/amr_15/traffic/yield_pose");
  ASSERT_EQ(hold_gids.size(), 1U);
  ASSERT_EQ(pose_gids.size(), 1U);
  ASSERT_TRUE(waitFor([&link]() {return link().resubscribes() >= 1;}, 5s));
  // 새 DataReader 두 개 (옛 것은 사라진다)
  ASSERT_TRUE(
    waitFor(
      [&tm, &hold_gids, &pose_gids]() {
        const auto h = subscriptionGids(tm, "/amr_15/traffic/hold");
        const auto p = subscriptionGids(tm, "/amr_15/traffic/yield_pose");
        return h.size() == 1U && p.size() == 1U && h[0] != hold_gids[0] && p[0] != pose_gids[0];
      }, 5s));
  ASSERT_TRUE(waitFor([&link]() {return link().alarms() == 1;}, 5s));
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(link().alarms(), 1);   // 한 번만
  EXPECT_EQ(ex->context()->trafficHoldState(), TrafficHoldState::kAwaiting);
  std_msgs::msg::Bool on;
  on.data = true;
  hold_pub->publish(on);
  ASSERT_TRUE(
    waitFor([&ex]() {return ex->context()->trafficHoldState() == TrafficHoldState::kHold;}, 5s));
  const int after = link().resubscribes();
  std::this_thread::sleep_for(1s);
  EXPECT_EQ(link().resubscribes(), after);   // 받은 뒤에는 재생성하지 않는다
  EXPECT_EQ(link().alarms(), 1);
}

TEST_F(ExecutorNodeTest, IncompatiblePublisherIsSeenButNotMatchedSoNoResubscribe)
{
  using amr_behavior::TrafficHoldState;
  // volatile 발행자: 그래프에는 보이지만(auto → awaiting) transient_local 구독과 DDS 매칭되지
  // 않는다 (hold_failsafe.md §7 E3). 매칭이 0 이면 재생성하지 않고 (재생성으로 고쳐지지
  // 않는다), 경보는 awaiting 시간으로 낸다
  auto tm = std::make_shared<rclcpp::Node>("fake_traffic_manager_volatile");
  auto hold_pub = tm->create_publisher<std_msgs::msg::Bool>(
    "/amr_16/traffic/hold", rclcpp::QoS(10).reliable());
  auto ex = makeTrafficExecutor(
    "amr_16", 0.0, "auto",
    {rclcpp::Parameter("traffic_hold.resubscribe_after_s", 0.5),
      rclcpp::Parameter("traffic_hold.alarm_after_s", 1.0)});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(tm);
  exec.add_node(ex);
  amr_behavior_test::SpinThread spinner(exec);
  ASSERT_TRUE(
    waitFor(
      [&ex]() {return ex->context()->trafficHoldState() == TrafficHoldState::kAwaiting;}, 5s));
  ASSERT_TRUE(waitFor([&ex]() {return ex->context()->trafficLink().alarms() == 1;}, 5s));
  std_msgs::msg::Bool on;
  on.data = true;
  hold_pub->publish(on);
  std::this_thread::sleep_for(700ms);
  EXPECT_EQ(ex->count_publishers("/amr_16/traffic/hold"), 1U);   // 그래프: 보인다
  EXPECT_EQ(ex->context()->trafficLink().matched(), 0U);         // DDS 매칭: 없다
  EXPECT_EQ(ex->context()->trafficLink().resubscribes(), 0);
  EXPECT_EQ(ex->context()->trafficHoldState(), TrafficHoldState::kAwaiting);   // fail-closed
}

TEST(TrafficLinkWatch, MatchedChangeIsFlaggedInEveryState)
{
  using amr_behavior::TrafficHoldState;
  const std::vector<size_t> seq = {0, 0, 1, 1, 0, 2};
  const std::vector<bool> changed = {false, false, true, false, true, true};
  const std::vector<size_t> prev = {0, 0, 0, 1, 1, 0};
  const TrafficHoldState states[] = {
    TrafficHoldState::kUnmanaged, TrafficHoldState::kAwaiting, TrafficHoldState::kClear};
  for (const auto state : states) {
    amr_behavior::TrafficLinkWatch fresh;
    for (size_t i = 0; i < seq.size(); ++i) {
      const auto step = fresh.step(static_cast<double>(i), state, seq[i]);
      EXPECT_EQ(step.matched_changed, changed[i]) << i;
      EXPECT_EQ(step.previous_matched, prev[i]) << i;
      EXPECT_EQ(fresh.matched(), seq[i]);
    }
  }
}

TEST(TrafficLinkWatch, NotAwaitingNeverResubscribesOrAlarms)
{
  using amr_behavior::TrafficHoldState;
  for (const auto state :
    {TrafficHoldState::kUnmanaged, TrafficHoldState::kHold, TrafficHoldState::kClear})
  {
    amr_behavior::TrafficLinkWatch w;
    for (double t = 0.0; t <= 1000.0; t += 0.5) {
      const auto step = w.step(t, state, 1U);
      EXPECT_FALSE(step.resubscribe);
      EXPECT_FALSE(step.alarm);
      EXPECT_DOUBLE_EQ(step.awaiting_s, 0.0);
    }
    EXPECT_EQ(w.resubscribes(), 0);
    EXPECT_EQ(w.alarms(), 0);
  }
}

TEST(TrafficLinkWatch, ResubscribesAtBoundaryAndRestartsClockAfterward)
{
  using amr_behavior::TrafficHoldState;
  const auto kA = TrafficHoldState::kAwaiting;
  amr_behavior::TrafficLinkWatch w;   // 기본 30 s / 120 s
  EXPECT_FALSE(w.step(0.0, kA, 1U).resubscribe);
  EXPECT_FALSE(w.step(29.9, kA, 1U).resubscribe);
  const auto at = w.step(30.0, kA, 1U);   // 경계 포함
  EXPECT_TRUE(at.resubscribe);
  EXPECT_DOUBLE_EQ(at.matched_waiting_s, 30.0);
  EXPECT_EQ(at.resubscribes, 1);
  // 새 구독이 매칭된 때(30.2)부터 다시 잰다
  EXPECT_FALSE(w.step(30.2, kA, 1U).resubscribe);
  EXPECT_FALSE(w.step(60.1, kA, 1U).resubscribe);
  EXPECT_TRUE(w.step(60.2, kA, 1U).resubscribe);
  EXPECT_EQ(w.resubscribes(), 2);
}

TEST(TrafficLinkWatch, MatchLossRestartsResubscribeClockAndZeroMatchNeverResubscribes)
{
  using amr_behavior::TrafficHoldState;
  const auto kA = TrafficHoldState::kAwaiting;
  amr_behavior::TrafficLinkWatch w(amr_behavior::TrafficLinkConfig{30.0, 0.0});   // 경보 끔
  w.step(0.0, kA, 1U);
  w.step(20.0, kA, 0U);                     // 매칭이 끊겼다 → 시계 비움
  EXPECT_FALSE(w.step(25.0, kA, 1U).resubscribe);
  EXPECT_FALSE(w.step(54.9, kA, 1U).resubscribe);
  EXPECT_TRUE(w.step(55.0, kA, 1U).resubscribe);
  amr_behavior::TrafficLinkWatch zero(amr_behavior::TrafficLinkConfig{30.0, 0.0});
  for (double t = 0.0; t <= 1000.0; t += 0.5) {
    EXPECT_FALSE(zero.step(t, kA, 0U).resubscribe) << t;
  }
  EXPECT_EQ(zero.resubscribes(), 0);
  EXPECT_EQ(zero.alarms(), 0);              // 끔
}

TEST(TrafficLinkWatch, AlarmsOncePerAwaitingEvenWithoutMatchAndRearmsAfterLeaving)
{
  using amr_behavior::TrafficHoldState;
  const auto kA = TrafficHoldState::kAwaiting;
  amr_behavior::TrafficLinkWatch w(amr_behavior::TrafficLinkConfig{0.0, 120.0});   // 재생성 끔
  EXPECT_FALSE(w.step(0.0, kA, 0U).alarm);
  EXPECT_FALSE(w.step(119.9, kA, 0U).alarm);
  const auto at = w.step(120.0, kA, 0U);   // 매칭 없어도 awaiting 시간으로
  EXPECT_TRUE(at.alarm);
  EXPECT_DOUBLE_EQ(at.awaiting_s, 120.0);
  EXPECT_FALSE(w.step(121.0, kA, 1U).alarm);   // 한 번만
  EXPECT_FALSE(w.step(500.0, kA, 1U).resubscribe);   // 재생성 끔
  EXPECT_EQ(w.alarms(), 1);
  // awaiting 을 벗어났다(unmanaged) 다시 들어오면 시계 · 경보를 새로
  w.step(510.0, TrafficHoldState::kUnmanaged, 0U);
  EXPECT_FALSE(w.step(511.0, kA, 0U).alarm);
  EXPECT_FALSE(w.step(630.9, kA, 0U).alarm);
  EXPECT_TRUE(w.step(631.0, kA, 0U).alarm);
  EXPECT_EQ(w.alarms(), 2);
}

TEST(TrafficLinkWatch, FirstValueClearsClocks)
{
  using amr_behavior::TrafficHoldState;
  const auto kA = TrafficHoldState::kAwaiting;
  amr_behavior::TrafficLinkWatch w;
  w.step(0.0, kA, 1U);
  w.step(29.0, kA, 1U);
  // 첫 값(clear) 을 받았다 → 기다리는 것이 없다. (auto 의 발행자 깜빡임처럼) 다시 awaiting 이
  // 되면 새로 잰다
  EXPECT_FALSE(w.step(29.5, TrafficHoldState::kClear, 1U).resubscribe);
  EXPECT_FALSE(w.step(30.5, kA, 1U).resubscribe);
  EXPECT_FALSE(w.step(60.4, kA, 1U).resubscribe);
  EXPECT_TRUE(w.step(60.5, kA, 1U).resubscribe);
  EXPECT_FALSE(w.step(150.4, kA, 1U).alarm);   // awaiting 시작 30.5 + 120
  EXPECT_TRUE(w.step(150.5, kA, 1U).alarm);
}

TEST(TrafficLinkWatch, ContextStepsWithItsOwnGateState)
{
  // ExecutorContext 는 자기 게이트 상태(받은 값 · 교통 관리 여부)로 감시를 진행한다
  amr_behavior::ExecutorContext ctx([]() {return 0.0;});
  ctx.setTrafficLinkConfig(amr_behavior::TrafficLinkConfig{1.0, 2.0});
  EXPECT_FALSE(ctx.stepTrafficLink(0.0, 1U).resubscribe);   // unmanaged
  EXPECT_FALSE(ctx.stepTrafficLink(5.0, 1U).resubscribe);
  ctx.setTrafficManaged(true);                                // awaiting
  EXPECT_FALSE(ctx.stepTrafficLink(10.0, 1U).resubscribe);
  EXPECT_TRUE(ctx.stepTrafficLink(11.0, 1U).resubscribe);
  EXPECT_TRUE(ctx.stepTrafficLink(12.0, 1U).alarm);
  ctx.updateTrafficHold(false);                               // 첫 값
  EXPECT_FALSE(ctx.stepTrafficLink(100.0, 1U).resubscribe);
  EXPECT_EQ(ctx.trafficLink().resubscribes(), 1);
  EXPECT_EQ(ctx.trafficLink().alarms(), 1);
  EXPECT_DOUBLE_EQ(ctx.trafficLink().config().alarm_after_s, 2.0);
}

namespace
{

/// 분산 플릿 모드 실행기 (b4-wire): fleet.distributed 면 존 노드를 등록하고 분산 트리를 띄운다.
class DistributedExecutorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<amr_behavior::TaskExecutorNode> make(bool distributed)
  {
    const std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter(
        "bt_xml",
        std::string(AMR_BEHAVIOR_SOURCE_DIR) + "/behavior_trees/task_executor_distributed.xml"),
      rclcpp::Parameter("robot_id", "amr_08"),
      rclcpp::Parameter("fleet.distributed", distributed),
      rclcpp::Parameter("groot.enabled", false),
      rclcpp::Parameter("charger_claims_topic", ""),
    };
    return std::make_shared<amr_behavior::TaskExecutorNode>(
      rclcpp::NodeOptions().parameter_overrides(overrides));
  }
};

}  // namespace

TEST_F(DistributedExecutorTest, RegistersZoneNodesAndLoadsTheDistributedTree)
{
  auto node = make(true);
  ASSERT_NO_THROW(node->init());
  std::set<std::string> ids;
  for (const auto & n : node->tree()->nodes) {
    ids.insert(n->registrationName());
  }
  for (const char * id : {"ZonePathClear", "IsFleetServerAlive", "CommitTaskLock"}) {
    EXPECT_EQ(ids.count(id), 1U) << id;
  }
  // 에이전트 연결: 실행기 노드가 fleet_agent/state 를 구독한다
  EXPECT_EQ(node->count_subscribers("fleet_agent/state"), 1U);
}

TEST_F(DistributedExecutorTest, CentralExecutorDoesNotRegisterZoneNodes)
{
  // 중앙 모드 실행기는 존 노드를 등록하지 않는다 → 분산 트리를 주면 트리 생성이 실패한다
  auto node = make(false);
  EXPECT_THROW(node->init(), std::exception);
  EXPECT_EQ(node->count_subscribers("fleet_agent/state"), 0U);   // 에이전트 연결을 만들지 않는다
}
