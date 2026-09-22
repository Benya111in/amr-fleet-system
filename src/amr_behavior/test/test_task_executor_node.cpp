// task_executor_node 통합 시험: 실제 트리 XML + 프로세스 안 모의 Nav2/도킹 서버로 assign_task →
// 대기-이동-인식-작업-복귀 → task_status COMPLETED, 실행 중 재할당 거절, Groot 퍼블리셔 기동.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "amr_behavior/task_executor_node.hpp"
#include "amr_msgs/action/dock.hpp"
#include "amr_msgs/msg/detected_object_array.hpp"
#include "amr_msgs/msg/task.hpp"
#include "amr_msgs/srv/assign_task.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "spin_thread.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

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
      rclcpp::Parameter("charger_dock_id", "missing_charger"),
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
    "IDLE", "MOVING", "DOCKING", "LOADING", "MOVING", "DOCKING", "UNLOADING", "RETURNING", "IDLE"};
  EXPECT_EQ(phases_, expected);
  EXPECT_EQ(statuses_, (std::vector<uint8_t>{Task::STATUS_IN_PROGRESS, Task::STATUS_COMPLETED}));
  EXPECT_EQ(nav_->goals.load(), 3);
  EXPECT_EQ(dock_->goals.load(), 2);
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
