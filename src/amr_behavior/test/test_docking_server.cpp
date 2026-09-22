// docking_server_node 시험: 마커 자세 변환, 운동학 모사 폐루프 도킹(성공·재시도 소진·취소·중복
// goal 거절).
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "amr_behavior/docking/docking_server_node.hpp"
#include "amr_msgs/action/dock.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "spin_thread.hpp"

using namespace std::chrono_literals;
using amr_behavior::docking::DockingServerNode;
using amr_behavior::docking::markerToObservation;
using Dock = amr_msgs::action::Dock;

namespace
{

geometry_msgs::msg::Pose poseWithQuat(
  double x, double y, double qx, double qy, double qz,
  double qw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation.x = qx;
  p.orientation.y = qy;
  p.orientation.z = qz;
  p.orientation.w = qw;
  return p;
}

/// 도크 프레임 운동학 모사: cmd_vel_nav 적분(100 Hz), 마커 관측 발행(30 Hz, base_link, OpenCV z
/// 법선).
class DockSim
{
public:
  DockSim(const rclcpp::Node::SharedPtr & node, double standoff, double x, double y, double yaw)
  : node_(node), standoff_(standoff), x_(x), y_(y), yaw_(yaw)
  {
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "perception/dock_marker_pose", rclcpp::SensorDataQoS());
    sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav", rclcpp::QoS(10), [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_ = *msg;
        ++cmds_;
      });
    step_timer_ = node_->create_wall_timer(10ms, [this]() {step(0.01);});
    obs_timer_ = node_->create_wall_timer(33ms, [this]() {observe();});
  }

  void step(double dt)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double mid = yaw_ + 0.5 * cmd_.angular.z * dt;
    x_ += cmd_.linear.x * dt * std::cos(mid);
    y_ += cmd_.linear.x * dt * std::sin(mid);
    yaw_ += cmd_.angular.z * dt;
  }

  void observe()
  {
    if (!visible) {
      return;
    }
    geometry_msgs::msg::PoseStamped msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double dx = standoff_ - x_;
      const double dy = -y_;
      const double c = std::cos(yaw_);
      const double s = std::sin(yaw_);
      msg.pose.position.x = c * dx + s * dy;
      msg.pose.position.y = -s * dx + c * dy;
      msg.pose.position.z = 0.25;
      // 마커 z(법선) = base_link 에서 방위 π − ψ, 수평 → R = R_z(π − ψ)·R_y(90°)
      const double a = M_PI - yaw_;
      const double h = std::sqrt(0.5);
      // q = q_z(a) ⊗ q_y(π/2)
      const double cz = std::cos(0.5 * a);
      const double sz = std::sin(0.5 * a);
      msg.pose.orientation.w = cz * h;
      msg.pose.orientation.x = -sz * h;
      msg.pose.orientation.y = cz * h;
      msg.pose.orientation.z = sz * h;
    }
    msg.header.frame_id = "base_link";
    msg.header.stamp = node_->now();
    pub_->publish(msg);
  }

  double positionError()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::hypot(x_, y_);
  }

  double angleError()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::fabs(std::atan2(std::sin(yaw_), std::cos(yaw_)));
  }

  int commands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cmds_;
  }

  std::atomic<bool> visible{true};

private:
  rclcpp::Node::SharedPtr node_;
  double standoff_;
  double x_;
  double y_;
  double yaw_;
  std::mutex mutex_;
  geometry_msgs::msg::Twist cmd_;
  int cmds_{0};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr obs_timer_;
};

class DockingServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void start(double x, double y, double yaw, std::vector<rclcpp::Parameter> extra = {})
  {
    std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter("docks.ids", std::vector<std::string>{"dock_1"}),
      rclcpp::Parameter("docks.dock_1.standoff", 0.6),
      rclcpp::Parameter("detector_node", "no_such_detector"),
    };
    overrides.insert(overrides.end(), extra.begin(), extra.end());
    server_ = std::make_shared<DockingServerNode>(
      rclcpp::NodeOptions().parameter_overrides(overrides));
    sim_node_ = std::make_shared<rclcpp::Node>("dock_sim");
    sim_ = std::make_unique<DockSim>(sim_node_, 0.6, x, y, yaw);
    client_node_ = std::make_shared<rclcpp::Node>("dock_client");
    client_ = rclcpp_action::create_client<Dock>(client_node_, "dock");
    exec_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 3);
    exec_->add_node(server_);
    exec_->add_node(sim_node_);
    exec_->add_node(client_node_);
    spin_ = std::make_unique<amr_behavior_test::SpinThread>(*exec_);
    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  void TearDown() override
  {
    spin_.reset();
  }

  rclcpp_action::ClientGoalHandle<Dock>::SharedPtr send(uint8_t retries)
  {
    Dock::Goal goal;
    goal.dock_id = "dock_1";
    goal.max_retries = retries;
    auto options = rclcpp_action::Client<Dock>::SendGoalOptions();
    options.feedback_callback = [this](
      rclcpp_action::ClientGoalHandle<Dock>::SharedPtr,
      const std::shared_ptr<const Dock::Feedback> fb) {
        std::lock_guard<std::mutex> lock(fb_mutex_);
        if (phases_.empty() || phases_.back() != fb->current_phase) {
          phases_.push_back(fb->current_phase);
        }
      };
    auto future = client_->async_send_goal(goal, options);
    if (future.wait_for(5s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  rclcpp_action::ClientGoalHandle<Dock>::WrappedResult result(
    const rclcpp_action::ClientGoalHandle<Dock>::SharedPtr & handle, std::chrono::seconds limit)
  {
    auto future = client_->async_get_result(handle);
    EXPECT_EQ(future.wait_for(limit), std::future_status::ready);
    return future.get();
  }

  std::shared_ptr<DockingServerNode> server_;
  rclcpp::Node::SharedPtr sim_node_;
  std::unique_ptr<DockSim> sim_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Client<Dock>::SharedPtr client_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::unique_ptr<amr_behavior_test::SpinThread> spin_;
  std::mutex fb_mutex_;
  std::vector<std::string> phases_;
};

}  // namespace

TEST(MarkerToObservation, AxisConventions)
{
  const double h = std::sqrt(0.5);
  // OpenCV: 마커 z 가 로봇 쪽(−x_base) → R_y(−90°)
  auto obs = markerToObservation(poseWithQuat(0.8, 0.1, 0.0, -h, 0.0, h), "z");
  ASSERT_TRUE(obs.has_value());
  EXPECT_NEAR(std::fabs(obs->normal_yaw), M_PI, 1e-9);
  EXPECT_DOUBLE_EQ(obs->x, 0.8);
  // x 법선 규약: 방위 π 회전이면 +x → −x
  obs = markerToObservation(poseWithQuat(0.8, 0.0, 0.0, 0.0, 1.0, 0.0), "x");
  ASSERT_TRUE(obs.has_value());
  EXPECT_NEAR(std::fabs(obs->normal_yaw), M_PI, 1e-9);
  obs = markerToObservation(poseWithQuat(0.8, 0.0, 0.0, 0.0, 0.0, 1.0), "-x");
  ASSERT_TRUE(obs.has_value());
  EXPECT_NEAR(std::fabs(obs->normal_yaw), M_PI, 1e-9);
  // 법선이 수직(항등 자세의 z) → 평면 투영 불가
  EXPECT_FALSE(markerToObservation(poseWithQuat(0.8, 0.0, 0.0, 0.0, 0.0, 1.0), "z").has_value());
  EXPECT_FALSE(markerToObservation(poseWithQuat(0.8, 0.0, 0.0, 0.0, 0.0, 0.0), "z").has_value());
  EXPECT_FALSE(
    markerToObservation(poseWithQuat(std::nan(""), 0.0, 0.0, -h, 0.0, h), "z").has_value());
}

TEST_F(DockingServerTest, DocksWithinSpecTolerance)
{
  start(-0.35, 0.04, 4.0 * M_PI / 180.0);
  EXPECT_DOUBLE_EQ(server_->standoffFor("dock_1"), 0.6);
  EXPECT_DOUBLE_EQ(server_->standoffFor("other"), 0.65);
  auto handle = send(3);
  ASSERT_TRUE(handle);
  const auto res = result(handle, 60s);
  EXPECT_EQ(res.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(res.result);
  EXPECT_TRUE(res.result->success);
  EXPECT_EQ(res.result->attempts_used, 1);
  EXPECT_LE(res.result->final_position_error, 0.02F);
  EXPECT_LE(res.result->final_angle_error, 0.01745F);
  // 모사 진값 기준 (관측 지연·예측 오차 포함)
  EXPECT_LE(sim_->positionError(), 0.02);
  EXPECT_LE(sim_->angleError(), 0.01745);
  std::lock_guard<std::mutex> lock(fb_mutex_);
  ASSERT_GE(phases_.size(), 2U);
  EXPECT_EQ(phases_.back(), "succeeded");
  EXPECT_EQ(phases_[phases_.size() - 2], "final");
}

TEST_F(DockingServerTest, ExhaustsRetriesWithoutMarker)
{
  start(
    -0.35, 0.0, 0.0, {rclcpp::Parameter("search_timeout", 0.4),
      rclcpp::Parameter("backup_distance", 0.02)});
  sim_->visible = false;
  auto handle = send(2);
  ASSERT_TRUE(handle);
  const auto res = result(handle, 30s);
  EXPECT_EQ(res.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(res.result);
  EXPECT_FALSE(res.result->success);
  EXPECT_EQ(res.result->attempts_used, 2);
  EXPECT_LT(res.result->final_position_error, 0.0F);   // 추정 없음 → −1
  std::lock_guard<std::mutex> lock(fb_mutex_);
  EXPECT_NE(std::find(phases_.begin(), phases_.end(), "backup"), phases_.end());
}

TEST_F(DockingServerTest, CancelAndRejectConcurrentGoal)
{
  start(-0.8, 0.0, 0.0, {rclcpp::Parameter("control_law", "proportional")});
  auto handle = send(0);   // 0 → 서버 기본 max_attempts
  ASSERT_TRUE(handle);
  auto second = send(1);
  EXPECT_FALSE(second);    // 실행 중 → 거절 (goal handle null)
  std::this_thread::sleep_for(300ms);
  auto cancel = client_->async_cancel_goal(handle);
  EXPECT_EQ(cancel.wait_for(5s), std::future_status::ready);
  const auto res = result(handle, 10s);
  EXPECT_EQ(res.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_GT(sim_->commands(), 0);
}
