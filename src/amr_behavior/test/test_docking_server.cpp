// docking_server_node 시험: 마커 자세 변환(검출기 규약 = 마커 모델 프레임 +x 법선, 계약 C3), 운동학
// 모사 폐루프 도킹(성공·재시도 소진·취소·중복 goal 거절·취소 중 goal 선점), 도킹 예외 사각형
// safety/dock_exclusion 발행(계약 C2: 세션 동안 ≥ 10 Hz, 물러나면 중단).
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
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "spin_thread.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using amr_behavior::docking::distanceFromPlate;
using amr_behavior::docking::DockingServerNode;
using amr_behavior::docking::exclusionPolygon;
using amr_behavior::docking::ExclusionBox;
using amr_behavior::docking::MarkerObservation;
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

/// 검출기(amr_perception aruco_detector_node) 규약의 마커 자세: 마커 모델 프레임,
/// +x = 판 바깥 법선, +z = 위. base_link 에서 법선 방위가 normal_yaw 인 수직 판 → R_z(normal_yaw).
geometry_msgs::msg::Quaternion detectorQuat(double normal_yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * normal_yaw);
  q.w = std::cos(0.5 * normal_yaw);
  return q;
}

/// 도크 프레임 운동학 모사: cmd_vel_nav 적분(100 Hz), 마커 관측 발행(30 Hz, base_link, 검출기 규약
/// — 마커 모델 프레임 +x 법선).
class DockSim
{
public:
  DockSim(const rclcpp::Node::SharedPtr & node, double standoff, double x, double y, double yaw)
  : node_(node), standoff_(standoff), x_(x), y_(y), yaw_(yaw)
  {
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "perception/dock_marker_pose", rclcpp::SensorDataQoS());
    id_pub_ = node_->create_publisher<std_msgs::msg::Int32>("perception/dock_marker_id", 10);
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
      msg.pose.position.z = 0.07;
      // 마커 x(법선) = base_link 에서 방위 π − ψ (도크 프레임 −x 가 로봇 쪽)
      msg.pose.orientation = detectorQuat(M_PI - yaw_);
    }
    msg.header.frame_id = "base_link";
    msg.header.stamp = node_->now();
    pub_->publish(msg);
    if (marker_id >= 0) {   // 검출기처럼 자세 다음에 id
      std_msgs::msg::Int32 id;
      id.data = marker_id;
      id_pub_->publish(id);
    }
  }

  double positionError()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::hypot(x_, y_);
  }

  /// 도크 프레임에서 로봇을 옮긴다 (이탈 후진 모사).
  void teleport(double x, double y, double yaw)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    cmd_ = geometry_msgs::msg::Twist();
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
  std::atomic<int> marker_id{-1};   ///< −1 = id 를 내지 않음

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
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_pub_;
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
      rclcpp::Parameter("detector_enable_service", "no_such_detector/enable"),
    };
    overrides.insert(overrides.end(), extra.begin(), extra.end());
    server_ = std::make_shared<DockingServerNode>(
      rclcpp::NodeOptions().parameter_overrides(overrides));
    sim_node_ = std::make_shared<rclcpp::Node>("dock_sim");
    sim_ = std::make_unique<DockSim>(sim_node_, 0.6, x, y, yaw);
    client_node_ = std::make_shared<rclcpp::Node>("dock_client");
    client_ = rclcpp_action::create_client<Dock>(client_node_, "dock");
    exclusion_sub_ = client_node_->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "safety/dock_exclusion", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(fb_mutex_);
        exclusions_.push_back(*msg);
        exclusion_times_.push_back(std::chrono::steady_clock::now());
      });
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
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr exclusion_sub_;
  std::mutex fb_mutex_;
  std::vector<std::string> phases_;
  std::vector<geometry_msgs::msg::PolygonStamped> exclusions_;
  std::vector<std::chrono::steady_clock::time_point> exclusion_times_;
};

/// 점이 다각형 안인지 (반직선 교차).
bool inside(const geometry_msgs::msg::Polygon & poly, double x, double y)
{
  bool in = false;
  const size_t n = poly.points.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = poly.points[i].x;
    const double yi = poly.points[i].y;
    const double xj = poly.points[j].x;
    const double yj = poly.points[j].y;
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
      in = !in;
    }
  }
  return in;
}

}  // namespace

TEST(MarkerToObservation, DetectorModelFrameConventionIsTheDefault)
{
  // 계약 C3 (amr_perception aruco.py: R_cv→model 의 열 = [z_cv, x_cv, y_cv]): 정면으로 마주 본
  // 마커는 base_link 기준 yaw = π, 즉 쿼터니언 (0, 0, 1, 0).
  // 리뷰 회귀: 기본값이 z 여서 모든 실관측을 버렸다
  // (노드 기본값은 DocksWithinSpecTolerance 가 확인).
  auto obs = markerToObservation(poseWithQuat(1.4, 0.05, 0.0, 0.0, 1.0, 0.0), "x");
  ASSERT_TRUE(obs.has_value());
  EXPECT_NEAR(std::fabs(obs->normal_yaw), M_PI, 1e-9);   // 법선이 로봇 쪽 (−x_base)
  // 같은 자세를 OpenCV z 규약으로 읽으면 법선이 수직(z 축 = 위) → 관측을 버린다
  // (이전 기본값의 실패)
  EXPECT_FALSE(markerToObservation(poseWithQuat(1.4, 0.05, 0.0, 0.0, 1.0, 0.0), "z").has_value());
  // 10° 비스듬한 검출기 자세 → 법선 방위 π − 10°
  const auto q = detectorQuat(M_PI - 10.0 * M_PI / 180.0);
  obs = markerToObservation(poseWithQuat(1.2, 0.2, q.x, q.y, q.z, q.w), "x");
  ASSERT_TRUE(obs.has_value());
  EXPECT_NEAR(obs->normal_yaw, M_PI - 10.0 * M_PI / 180.0, 1e-9);
}

TEST(DockExclusion, PolygonCoversPlateAndWallButNotTheRobot)
{
  // docked: 판 면이 base_link 전방 0.65 m, 법선이 로봇 쪽
  MarkerObservation m;
  m.x = 0.65;
  m.y = 0.0;
  m.normal_yaw = M_PI;
  EXPECT_NEAR(distanceFromPlate(m), 0.65, 1e-12);
  const ExclusionBox box;
  const auto poly = exclusionPolygon(m, box);
  ASSERT_EQ(poly.size(), 4U);
  geometry_msgs::msg::Polygon p;
  for (const auto & v : poly) {
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(v.first);
    pt.y = static_cast<float>(v.second);
    p.points.push_back(pt);
  }
  EXPECT_TRUE(inside(p, 0.65, 0.0));      // 판 중심
  EXPECT_TRUE(inside(p, 0.68, 0.45));     // 판 옆 벽 (+2 cm 뒤)
  EXPECT_TRUE(inside(p, 0.56, -0.3));     // LiDAR 노이즈로 앞당겨진 벽 점 (σ 3 cm × 3)
  EXPECT_FALSE(inside(p, 0.30, 0.0));     // 범퍼 (base_link 전방 0.30 m)
  EXPECT_FALSE(inside(p, 0.50, 0.0));     // 판 앞 0.15 m 의 물체 → 0.30 m 정지 규칙 그대로
  EXPECT_FALSE(inside(p, 0.65, 0.8));     // 가로 밖
  // 비스듬한 판 (법선 방위 π − 0.2): 사각형도 함께 돈다
  m.normal_yaw = M_PI - 0.2;
  const auto rotated = exclusionPolygon(m, box);
  EXPECT_NEAR(
    rotated[0].first, 0.65 + box.front_margin * std::cos(m.normal_yaw) +
    box.half_width * std::sin(m.normal_yaw), 1e-12);
}

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
  EXPECT_EQ(server_->normalAxis(), "x");   // 계약 C3 기본값 (모사는 검출기 규약 자세를 낸다)
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

TEST_F(DockingServerTest, PublishesDockExclusionDuringTheSession)
{
  start(-0.35, 0.04, 4.0 * M_PI / 180.0);
  auto handle = send(3);
  ASSERT_TRUE(handle);
  const auto res = result(handle, 60s);
  ASSERT_EQ(res.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(server_->exclusionSessionActive());   // docked: 적재·하역 동안 유지
  std::this_thread::sleep_for(500ms);
  {
    std::lock_guard<std::mutex> lock(fb_mutex_);
    ASSERT_GE(exclusions_.size(), 20U);
    // 계약 C2: ≥ 10 Hz (safety_node exclusion_timeout 0.3 s 안에 여러 번)
    const double span = std::chrono::duration<double>(
      exclusion_times_.back() - exclusion_times_.front()).count();
    EXPECT_GE((exclusions_.size() - 1) / span, 10.0);
    double max_gap = 0.0;
    for (size_t i = 1; i < exclusion_times_.size(); ++i) {
      const auto gap = exclusion_times_[i] - exclusion_times_[i - 1];
      max_gap = std::max(max_gap, std::chrono::duration<double>(gap).count());
    }
    EXPECT_LT(max_gap, 0.3);
    const auto & last = exclusions_.back();
    EXPECT_EQ(last.header.frame_id, "base_link");
    // docked: 판(base_link 전방 0.6 m) 은 안, 로봇 풋프린트 전면(0.30 m) 은 밖
    EXPECT_TRUE(inside(last.polygon, 0.6, 0.0));
    EXPECT_FALSE(inside(last.polygon, 0.30, 0.0));
    exclusions_.clear();
    exclusion_times_.clear();
  }
  // 이탈: standoff + release(0.5 m) 밖으로 물러나면 세션 종료 → 발행 중단
  sim_->teleport(-0.7, 0.0, 0.0);
  std::this_thread::sleep_for(400ms);
  EXPECT_FALSE(server_->exclusionSessionActive());
  {
    std::lock_guard<std::mutex> lock(fb_mutex_);
    exclusions_.clear();
  }
  std::this_thread::sleep_for(300ms);
  std::lock_guard<std::mutex> lock(fb_mutex_);
  EXPECT_TRUE(exclusions_.empty());
}

TEST_F(DockingServerTest, ExclusionSessionEndsWhenMarkerIsLostAfterTheGoal)
{
  start(-0.35, 0.0, 0.0);
  auto handle = send(1);
  ASSERT_TRUE(handle);
  ASSERT_EQ(result(handle, 60s).code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(server_->exclusionSessionActive());
  sim_->visible = false;   // 로봇이 돌아서 마커가 안 보인다
  std::this_thread::sleep_for(800ms);
  EXPECT_FALSE(server_->exclusionSessionActive());
}

TEST_F(DockingServerTest, IgnoresTheNeighbourDocksMarker)
{
  // search 회전에서는 4 m 옆 이웃 도크 마커도 보인다 → 도크 표의 marker_id 와 다른 id 의 관측은
  // 버린다
  start(-0.35, 0.0, 0.0, {rclcpp::Parameter("docks.dock_1.marker_id", 0)});
  EXPECT_EQ(server_->markerIdFor("dock_1"), 0);
  EXPECT_EQ(server_->markerIdFor("other"), -1);
  sim_->marker_id = 1;   // 이웃 도크 (id 1) — 검출기처럼 goal 전부터 자세·id 를 낸다
  std::this_thread::sleep_for(300ms);
  auto handle = send(1);
  ASSERT_TRUE(handle);
  std::this_thread::sleep_for(1500ms);
  {
    std::lock_guard<std::mutex> lock(fb_mutex_);
    ASSERT_FALSE(phases_.empty());
    EXPECT_EQ(phases_.back(), "search");   // 관측을 받지 않았다
  }
  sim_->marker_id = 0;   // 자기 도크 마커
  const auto res = result(handle, 60s);
  EXPECT_EQ(res.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_LE(sim_->positionError(), 0.02);
}

TEST_F(DockingServerTest, StaleMarkerIdSkipsTheIdCheckInsteadOfDroppingObservations)
{
  // 회귀 (통합 시나리오 10): id 가 marker_id_max_age 보다 오래되면 "다른 마커" 로 보고
  // 관측을 전부 버려 도킹이 3 시도를 소진했다. 오래된 id 는 확인만 생략하고 자세는 쓴다.
  start(
    -0.35, 0.0, 0.0, {rclcpp::Parameter("docks.dock_1.marker_id", 0),
      rclcpp::Parameter("marker_id_max_age", 0.2)});
  sim_->marker_id = 0;
  std::this_thread::sleep_for(300ms);
  auto handle = send(1);
  ASSERT_TRUE(handle);
  std::this_thread::sleep_for(400ms);
  sim_->marker_id = -1;   // 검출기가 id 발행을 멈춘다 (자세는 계속 온다)
  const auto res = result(handle, 60s);
  EXPECT_EQ(res.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_LE(sim_->positionError(), 0.02);
}

TEST_F(DockingServerTest, CanceledGoalIsPreemptedByTheNextGoal)
{
  // BT 가 hold 로 Dock 을 halt(취소)한 직후 다시 보낸 goal 은 거절되지 않아야 한다
  start(-0.8, 0.0, 0.0);
  auto first = send(1);
  ASSERT_TRUE(first);
  std::this_thread::sleep_for(200ms);
  auto cancel = client_->async_cancel_goal(first);
  ASSERT_EQ(cancel.wait_for(5s), std::future_status::ready);   // 서버가 취소를 받은 직후
  auto second = send(1);
  ASSERT_TRUE(second);
  const auto res = result(second, 60s);
  EXPECT_EQ(res.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result(first, 5s).code, rclcpp_action::ResultCode::CANCELED);
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
