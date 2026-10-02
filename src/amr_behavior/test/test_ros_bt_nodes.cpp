// ROS 액션/서비스 BT 노드 시험: 프로세스 안 모의 서버로 성공·실패·거절·halt 취소·서버 없음·시간
// 초과를 검증한다.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "amr_behavior/bt_registry.hpp"
#include "amr_msgs/action/dock.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "spin_thread.hpp"

using namespace std::chrono_literals;
using BT::NodeStatus;

namespace
{

enum class Mode {kSucceed, kAbort, kReject, kHang, kSlowAccept};

/// 액션 모의 서버: 모드에 따라 성공/중단/거절/취소 대기. 마지막 goal 과 취소 여부를 기록한다.
template<class ActionT>
class MockActionServer
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  MockActionServer(const rclcpp::Node::SharedPtr & node, const std::string & name)
  {
    server_ = rclcpp_action::create_server<ActionT>(
      node, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal> goal) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_goal_ = *goal;
          ++goals_;
        }
        if (mode == Mode::kSlowAccept) {
          std::this_thread::sleep_for(600ms);
        }
        return mode == Mode::kReject ? rclcpp_action::GoalResponse::REJECT :
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        canceled = true;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        std::thread([this, handle]() {execute(handle);}).detach();
      });
  }

  typename ActionT::Goal lastGoal()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_goal_;
  }

  int goals()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_;
  }

  std::atomic<Mode> mode{Mode::kSucceed};
  std::atomic<bool> canceled{false};
  std::function<void(typename ActionT::Result &)> fill_result;
  std::function<void(typename ActionT::Feedback &)> fill_feedback;

private:
  void execute(const std::shared_ptr<GoalHandle> handle)
  {
    auto result = std::make_shared<typename ActionT::Result>();
    if (fill_result) {
      fill_result(*result);
    }
    if (fill_feedback) {
      // 첫 goal 에서는 피드백 토픽 매칭이 늦을 수 있어 여러 번 보낸다
      for (int i = 0; i < 5; ++i) {
        auto fb = std::make_shared<typename ActionT::Feedback>();
        fill_feedback(*fb);
        handle->publish_feedback(fb);
        std::this_thread::sleep_for(40ms);
      }
    }
    if (mode == Mode::kHang || mode == Mode::kSlowAccept) {
      for (int i = 0; i < 400 && rclcpp::ok(); ++i) {
        if (handle->is_canceling()) {
          handle->canceled(result);
          return;
        }
        std::this_thread::sleep_for(10ms);
      }
      if (handle->is_active()) {
        handle->abort(result);
      }
      return;
    }
    std::this_thread::sleep_for(20ms);
    if (mode == Mode::kAbort) {
      handle->abort(result);
    } else {
      handle->succeed(result);
    }
  }

  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::mutex mutex_;
  typename ActionT::Goal last_goal_;
  int goals_{0};
};

class RosBtNodesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    client_ = std::make_shared<rclcpp::Node>("bt_client");
    server_ = std::make_shared<rclcpp::Node>("bt_server");
    nav_ = std::make_unique<MockActionServer<nav2_msgs::action::NavigateToPose>>(
      server_, "navigate_to_pose");
    dock_ = std::make_unique<MockActionServer<amr_msgs::action::Dock>>(server_, "dock");
    spin_ = std::make_unique<MockActionServer<nav2_msgs::action::Spin>>(server_, "spin");
    backup_ = std::make_unique<MockActionServer<nav2_msgs::action::BackUp>>(server_, "backup");
    wait_ = std::make_unique<MockActionServer<nav2_msgs::action::Wait>>(server_, "wait");
    clear_srv_ = server_->create_service<nav2_msgs::srv::ClearEntireCostmap>(
      "global_costmap/clear_entirely_global_costmap",
      [this](
        const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>,
        std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response>) {
        ++clears_;
        if (slow_clear_) {
          std::this_thread::sleep_for(700ms);
        }
      });
    client_exec_.add_node(client_);
    server_exec_.add_node(server_);
    client_spin_ = std::make_unique<amr_behavior_test::SpinThread>(client_exec_);
    server_spin_ = std::make_unique<amr_behavior_test::SpinThread>(server_exec_);

    params_ = amr_behavior::RosNodeParams::make(client_);
    params_.server_timeout = 300ms;
    params_.wait_for_server_timeout = 300ms;
    amr_behavior::registerRosNodes(factory_, params_);
    blackboard_ = BT::Blackboard::create();
  }

  void TearDown() override
  {
    trees_.clear();
    client_spin_.reset();
    server_spin_.reset();
  }

  BT::Tree & tree(const std::string & body)
  {
    const std::string xml = "<root main_tree_to_execute=\"T\"><BehaviorTree ID=\"T\">" + body +
      "</BehaviorTree></root>";
    trees_.push_back(std::make_unique<BT::Tree>(factory_.createTreeFromText(xml, blackboard_)));
    return *trees_.back();
  }

  static NodeStatus run(BT::Tree & t, std::chrono::milliseconds limit = 5s)
  {
    const auto end = std::chrono::steady_clock::now() + limit;
    NodeStatus s = t.tickRoot();
    while (s == NodeStatus::RUNNING && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(5ms);
      s = t.tickRoot();
    }
    return s;
  }

  void setGoal(double x, double y, const std::string & frame = "map")
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = frame;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.w = 1.0;
    blackboard_->set<geometry_msgs::msg::PoseStamped>("goal", p);
  }

  rclcpp::Node::SharedPtr client_;
  rclcpp::Node::SharedPtr server_;
  rclcpp::executors::SingleThreadedExecutor client_exec_;
  rclcpp::executors::SingleThreadedExecutor server_exec_;
  std::unique_ptr<amr_behavior_test::SpinThread> client_spin_;
  std::unique_ptr<amr_behavior_test::SpinThread> server_spin_;
  std::unique_ptr<MockActionServer<nav2_msgs::action::NavigateToPose>> nav_;
  std::unique_ptr<MockActionServer<amr_msgs::action::Dock>> dock_;
  std::unique_ptr<MockActionServer<nav2_msgs::action::Spin>> spin_;
  std::unique_ptr<MockActionServer<nav2_msgs::action::BackUp>> backup_;
  std::unique_ptr<MockActionServer<nav2_msgs::action::Wait>> wait_;
  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_srv_;
  std::atomic<int> clears_{0};
  std::atomic<bool> slow_clear_{false};
  amr_behavior::RosNodeParams params_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::vector<std::unique_ptr<BT::Tree>> trees_;
};

}  // namespace

TEST_F(RosBtNodesTest, NavigateToPoseOutcomes)
{
  setGoal(1.0, 2.0);
  nav_->fill_feedback = [](nav2_msgs::action::NavigateToPose::Feedback & fb) {
      fb.distance_remaining = 1.25F;
    };
  auto & t = tree("<NavigateToPose goal=\"{goal}\" distance_remaining=\"{dist}\"/>");
  EXPECT_EQ(run(t), NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(nav_->lastGoal().pose.pose.position.y, 2.0);
  double dist = 0.0;
  EXPECT_TRUE(blackboard_->get<double>("dist", dist));
  EXPECT_NEAR(dist, 1.25, 1e-6);

  nav_->mode = Mode::kAbort;
  EXPECT_EQ(run(t), NodeStatus::FAILURE);
  nav_->mode = Mode::kReject;
  EXPECT_EQ(run(t), NodeStatus::FAILURE);

  // 잘못된 goal (frame 없음) → 서버에 보내지 않고 FAILURE, skip_if_empty 면 SUCCESS
  nav_->mode = Mode::kSucceed;
  const int goals = nav_->goals();
  setGoal(0.0, 0.0, "");
  EXPECT_EQ(run(t), NodeStatus::FAILURE);
  auto & skip = tree("<NavigateToPose goal=\"{goal}\" skip_if_empty=\"true\"/>");
  EXPECT_EQ(run(skip), NodeStatus::SUCCESS);
  EXPECT_EQ(nav_->goals(), goals);
}

TEST_F(RosBtNodesTest, HaltCancelsRunningGoal)
{
  setGoal(1.0, 0.0);
  nav_->mode = Mode::kHang;
  auto & t = tree("<NavigateToPose goal=\"{goal}\"/>");
  EXPECT_EQ(run(t, 500ms), NodeStatus::RUNNING);
  t.haltTree();
  for (int i = 0; i < 200 && !nav_->canceled; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(nav_->canceled);
}

TEST_F(RosBtNodesTest, SlowAcceptTimesOutAndLateGoalIsCanceled)
{
  setGoal(1.0, 0.0);
  nav_->mode = Mode::kSlowAccept;   // 600 ms 뒤 수락 > server_timeout 300 ms
  auto & t = tree("<NavigateToPose goal=\"{goal}\"/>");
  EXPECT_EQ(run(t), NodeStatus::FAILURE);
  for (int i = 0; i < 300 && !nav_->canceled; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(nav_->canceled);   // 고아 goal 방지
}

TEST_F(RosBtNodesTest, MissingServerFails)
{
  setGoal(1.0, 0.0);
  auto & t = tree("<NavigateToPose goal=\"{goal}\" server_name=\"no_such_server\"/>");
  EXPECT_EQ(run(t), NodeStatus::FAILURE);
  auto & s = tree("<ClearCostmap service_name=\"no_such_service\"/>");
  EXPECT_EQ(run(s), NodeStatus::FAILURE);
}

TEST_F(RosBtNodesTest, DockResultOutputsAndSkip)
{
  dock_->fill_result = [](amr_msgs::action::Dock::Result & r) {
      r.success = true;
      r.final_position_error = 0.012F;
      r.final_angle_error = 0.01F;
      r.attempts_used = 1;
    };
  dock_->fill_feedback = [](amr_msgs::action::Dock::Feedback & fb) {fb.current_phase = "final";};
  const std::string body =
    "<Dock dock_id=\"{dock_id}\" max_retries=\"2\" attempts_used=\"{att}\""
    " final_position_error=\"{pe}\" final_angle_error=\"{ae}\" dock_phase=\"{ph}\"/>";
  blackboard_->set<std::string>("dock_id", "dock_1");
  auto & t = tree(body);
  EXPECT_EQ(run(t), NodeStatus::SUCCESS);
  EXPECT_EQ(dock_->lastGoal().dock_id, "dock_1");
  EXPECT_EQ(dock_->lastGoal().max_retries, 2);
  EXPECT_EQ(blackboard_->get<unsigned>("att"), 1U);
  EXPECT_NEAR(blackboard_->get<double>("pe"), 0.012, 1e-6);
  std::string dock_phase;
  EXPECT_TRUE(blackboard_->get<std::string>("ph", dock_phase));
  EXPECT_EQ(dock_phase, "final");

  dock_->fill_result = [](amr_msgs::action::Dock::Result & r) {
      r.success = false;
      r.attempts_used = 3;
    };
  EXPECT_EQ(run(t), NodeStatus::FAILURE);   // SUCCEEDED 라도 success=false 면 실패
  EXPECT_EQ(blackboard_->get<unsigned>("att"), 3U);

  const int goals = dock_->goals();
  blackboard_->set<std::string>("dock_id", "");
  EXPECT_EQ(run(t), NodeStatus::SUCCESS);   // 도킹 생략
  EXPECT_EQ(dock_->goals(), goals);
}

TEST_F(RosBtNodesTest, RecoveryActionsSendGoals)
{
  auto & s = tree("<Spin spin_dist=\"0.52\" time_allowance=\"5\"/>");
  EXPECT_EQ(run(s), NodeStatus::SUCCESS);
  EXPECT_NEAR(spin_->lastGoal().target_yaw, 0.52, 1e-6);
  EXPECT_EQ(spin_->lastGoal().time_allowance.sec, 5);

  auto & b = tree("<BackUp backup_dist=\"0.3\" backup_speed=\"0.1\"/>");
  EXPECT_EQ(run(b), NodeStatus::SUCCESS);
  EXPECT_NEAR(backup_->lastGoal().target.x, 0.3, 1e-9);
  EXPECT_NEAR(backup_->lastGoal().speed, 0.1, 1e-6);
  auto & bad = tree("<BackUp backup_dist=\"-1\"/>");
  EXPECT_EQ(run(bad), NodeStatus::FAILURE);

  auto & w = tree("<Wait wait_duration=\"1.5\"/>");
  EXPECT_EQ(run(w), NodeStatus::SUCCESS);
  EXPECT_EQ(wait_->lastGoal().time.sec, 1);
  EXPECT_EQ(wait_->lastGoal().time.nanosec, 500000000U);
  auto & wbad = tree("<Wait wait_duration=\"-1\"/>");
  EXPECT_EQ(run(wbad), NodeStatus::FAILURE);
  auto & sbad = tree("<Spin time_allowance=\"0\"/>");
  EXPECT_EQ(run(sbad), NodeStatus::FAILURE);
}

TEST_F(RosBtNodesTest, ClearCostmapServiceOutcomes)
{
  auto & t = tree("<ClearCostmap/>");
  EXPECT_EQ(run(t), NodeStatus::SUCCESS);
  EXPECT_EQ(clears_.load(), 1);
  slow_clear_ = true;   // 700 ms > server_timeout 300 ms
  EXPECT_EQ(run(t), NodeStatus::FAILURE);
  auto & h = tree("<ClearCostmap server_timeout=\"2000\"/>");
  EXPECT_EQ(run(h, 100ms), NodeStatus::RUNNING);
  h.haltTree();   // 대기 중 요청 폐기
  slow_clear_ = false;
  std::this_thread::sleep_for(800ms);
  EXPECT_EQ(run(t), NodeStatus::SUCCESS);
}

TEST_F(RosBtNodesTest, ClientCacheSharesClients)
{
  auto cache = params_.clients;
  const std::size_t before = cache->size();
  auto a = cache->action<nav2_msgs::action::Spin>("spin_x");
  auto b = cache->action<nav2_msgs::action::Spin>("spin_x");
  auto c = cache->service<nav2_msgs::srv::ClearEntireCostmap>("clear_x");
  auto d = cache->service<nav2_msgs::srv::ClearEntireCostmap>("clear_x");
  EXPECT_EQ(a, b);
  EXPECT_EQ(c, d);
  EXPECT_EQ(cache->size(), before + 2U);
}
