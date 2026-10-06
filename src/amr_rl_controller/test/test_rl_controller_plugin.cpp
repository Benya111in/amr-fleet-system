// RLController 플러그인 ROS 계층 시험 (Gazebo 없음).
// 실제 Costmap2DROS(빈 방) 와 LifecycleNode 위에서 configure → activate → setPlan →
// computeVelocityCommands 를 부르고, 스캔은 같은 프로세스의 발행자로 넣는다.
//  - pluginlib 로 이름 로드 (controller_server 와 같은 경로)
//  - 스캔 없음 → DWA 폴백(입력 무효), 스캔 있음 → 정책 결정(C·B), 명령 한계
//  - 계약 위반 모델·모드 불일치·빈 경로 → configure 예외
//  - 추론 시간 초과 → DWA, DWA 폴백 끔 → 계약 제동 규칙, setSpeedLimit → v_cap
//  - 정책 → DWA 폴백 전환에서도 틱당 계약 가속 한계·속도 제한·벽 앞 제동 (내부 DWA 의 창 중심이
//    낡음 → 실제 직전 명령으로 syncLastCommand, 측정이 window_reset 밖이면 계약 동적 창 자름)
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "amr_rl_controller/rl_controller.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace fs = std::filesystem;
namespace C = amr_rl_controller::contract;
using amr_rl_controller::DwaReason;
using amr_rl_controller::RLController;
using amr_rl_controller::Source;

namespace
{
bool g_models_ok = false;

class RosEnv : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    const std::string cmd = std::string("RL_CONTRACT_DIR=") + RL_CONTRACT_DIR + " python3 " +
      MAKE_MODELS + " " + FIXTURE_DIR + " > " + FIXTURE_DIR + "_plugin.log 2>&1";
    g_models_ok = std::system(cmd.c_str()) == 0 &&
      fs::exists(std::string(FIXTURE_DIR) + "/fixture_c.onnx");
  }
  void TearDown() override {rclcpp::shutdown();}
};
const auto * const g_env = ::testing::AddGlobalTestEnvironment(new RosEnv);

#define REQUIRE_MODELS() \
  if (!g_models_ok) {GTEST_SKIP() << "시험 모델을 만들지 못했다";}

std::string model(const std::string & n) {return std::string(FIXTURE_DIR) + "/" + n;}

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeCostmap(const std::string & name, int width = 10)
{
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "-r", "__node:=" + name});
  opts.parameter_overrides(
    {
      rclcpp::Parameter("global_frame", "map"),
      rclcpp::Parameter("robot_base_frame", "base_footprint"),
      rclcpp::Parameter(
        "footprint", "[[0.30, 0.20], [0.30, -0.20], [-0.30, -0.20], [-0.30, 0.20]]"),
      rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}),
      rclcpp::Parameter("inflation_layer.plugin", "nav2_costmap_2d::InflationLayer"),
      rclcpp::Parameter("width", width),
      rclcpp::Parameter("height", 6),
      rclcpp::Parameter("resolution", 0.05),
    });
  auto cm = std::make_shared<nav2_costmap_2d::Costmap2DROS>(opts);
  cm->on_configure(rclcpp_lifecycle::State());
  return cm;
}

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  return p;
}

nav_msgs::msg::Path straightPlan(double x0, double y, double len)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    path.poses.push_back(pose(x0 + s, y, 0.0));
  }
  return path;
}

struct Rig
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<tf2_ros::Buffer> tf;
  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  rclcpp::executors::SingleThreadedExecutor exec;
  std::vector<std_msgs::msg::Float64MultiArray> stats;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stats_sub;

  explicit Rig(
    const std::string & id, const std::vector<rclcpp::Parameter> & params, int costmap_width = 10)
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "rl_test_" + id, rclcpp::NodeOptions().parameter_overrides(params));
    costmap = makeCostmap("rl_costmap_" + id, costmap_width);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    pub_node = std::make_shared<rclcpp::Node>("rl_scan_pub_" + id);
    scan_pub = pub_node->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);
    stats_sub = pub_node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/rl_controller/stats", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {stats.push_back(*m);});
    exec.add_node(node->get_node_base_interface());
    exec.add_node(pub_node);
  }

  /// 720 빔 스캔 (range 이 inf 면 무반사) 을 내고 구독 콜백까지 돌린다.
  void publishScan(
    double range = std::numeric_limits<double>::infinity(), double age = 0.0,
    int beams = C::kLidarSamples)
  {
    sensor_msgs::msg::LaserScan s;
    s.header.frame_id = "lidar_link";
    s.header.stamp = node->now() - rclcpp::Duration::from_seconds(age);
    s.angle_min = static_cast<float>(C::kLidarAngleMin);
    s.angle_max = static_cast<float>(C::kLidarAngleMax);
    s.angle_increment = static_cast<float>(C::kLidarAngleInc);
    s.range_min = 0.1f;
    s.range_max = 25.0f;
    s.ranges.assign(static_cast<std::size_t>(beams), static_cast<float>(range));
    scan_pub->publish(s);
    spin();
  }
  void spin()
  {
    for (int k = 0; k < 20; ++k) {
      exec.spin_some(std::chrono::milliseconds(5));
    }
  }
};

std::vector<rclcpp::Parameter> params(
  const std::string & model_file, const std::string & mode,
  const std::vector<rclcpp::Parameter> & extra = {})
{
  std::vector<rclcpp::Parameter> p{
    rclcpp::Parameter("controller_frequency", 20.0),
    rclcpp::Parameter("FollowPath.rl.model_path", model(model_file)),
    rclcpp::Parameter("FollowPath.rl.mode", mode)};
  p.insert(p.end(), extra.begin(), extra.end());
  return p;
}

/// 완전 추종 폐루프 (측정 속도 = 직전 명령, 직선 경로 y = 3 위 전진). 틱마다 직전 명령과의
/// 차이 최댓값을 모은다.
struct Loop
{
  explicit Loop(RLController & c)
  : ctrl(c) {}
  RLController & ctrl;
  geometry_msgs::msg::Twist vel;
  double x{1.0};
  double max_dv{0.0};
  double max_dw{0.0};

  amr_rl_controller::TickResult step()
  {
    const auto cmd = ctrl.computeVelocityCommands(pose(x, 3.0, 0.0), vel, nullptr);
    max_dv = std::max(max_dv, std::abs(cmd.twist.linear.x - vel.linear.x));
    max_dw = std::max(max_dw, std::abs(cmd.twist.angular.z - vel.angular.z));
    vel.linear.x = cmd.twist.linear.x;
    vel.angular.z = cmd.twist.angular.z;
    x += vel.linear.x * 0.05;
    return ctrl.lastResult();
  }
  void resetMax() {max_dv = max_dw = 0.0;}
};

constexpr double kDvTick = C::kAMax * 0.05;       // a_max·Δt_c (20 Hz)
constexpr double kDwTick = C::kAlphaMax * 0.05;   // α_max·Δt_c
}  // namespace

TEST(RLControllerPlugin, PluginlibLoadsByName)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  std::shared_ptr<nav2_core::Controller> c;
  ASSERT_NO_THROW(c = loader.createSharedInstance("amr_rl_controller::RLController"));
  EXPECT_NE(dynamic_cast<RLController *>(c.get()), nullptr);
}

TEST(RLControllerPlugin, ModeCRunsPolicyWhenScanArrives)
{
  REQUIRE_MODELS();
  Rig rig("c", params("fixture_c.onnx", "C"));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 6.0));
  geometry_msgs::msg::Twist vel;

  // 스캔 없음 → 입력 무효 → 그 결정 주기(2 틱)는 DWA 폴백
  geometry_msgs::msg::TwistStamped cmd;
  for (int k = 0; k < 2; ++k) {
    cmd = ctrl.computeVelocityCommands(pose(1.0, 3.0, 0.0), vel, nullptr);
    EXPECT_EQ(ctrl.lastResult().source, Source::kDwa);
    EXPECT_EQ(ctrl.lastResult().dwa_reason, DwaReason::kStateInvalid);
    EXPECT_TRUE(std::isfinite(cmd.twist.linear.x));
  }

  double x = 1.0;
  for (int k = 0; k < 40; ++k) {
    rig.publishScan();
    cmd = ctrl.computeVelocityCommands(pose(x, 3.0, 0.0), vel, nullptr);
    const auto r = ctrl.lastResult();
    ASSERT_NE(r.source, Source::kDwa) << k << " " << r.error;
    EXPECT_GE(cmd.twist.linear.x, -1e-9);
    EXPECT_LE(cmd.twist.linear.x, 1.0 + 1e-9);
    EXPECT_LE(std::abs(cmd.twist.angular.z), 1.5 + 1e-9);
    vel.linear.x = cmd.twist.linear.x;
    vel.angular.z = cmd.twist.angular.z;
    x += cmd.twist.linear.x * 0.05;
  }
  const auto c = ctrl.counters();
  EXPECT_EQ(c.decisions, 21U);   // 첫 DWA 결정 + 40 틱 / 2
  EXPECT_EQ(c.dwa[1], 1U);
  rig.spin();
  ASSERT_FALSE(rig.stats.empty());
  EXPECT_EQ(rig.stats.back().data.size(), 26U);
  ctrl.deactivate();
  ctrl.cleanup();
}

TEST(RLControllerPlugin, ModeBRunsPolicy)
{
  REQUIRE_MODELS();
  Rig rig("b", params("fixture_b.onnx", "B"));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 6.0));
  geometry_msgs::msg::Twist vel;
  for (int k = 0; k < 10; ++k) {
    rig.publishScan(3.0);
    const auto cmd = ctrl.computeVelocityCommands(pose(1.0, 3.0, 0.0), vel, nullptr);
    EXPECT_EQ(ctrl.lastResult().source, Source::kPolicy) << ctrl.lastResult().error;
    EXPECT_GE(cmd.twist.linear.x, -1e-9);
    EXPECT_LE(std::abs(cmd.twist.angular.z), 1.5 + 1e-9);
    vel.linear.x = cmd.twist.linear.x;
    vel.angular.z = cmd.twist.angular.z;
  }
  EXPECT_EQ(ctrl.counters().decisions, 5U);
}

TEST(RLControllerPlugin, ConfigureRejectsBadModelsAndParams)
{
  REQUIRE_MODELS();
  struct Case
  {
    std::string id;
    std::string file;
    std::string mode;
  };
  for (const auto & c : std::vector<Case>{{"x1", "fixture_b.onnx", "C"},
      {"x2", "bad_major_c.onnx", "C"}, {"x3", "missing.onnx", "C"},
      {"x4", "fixture_c.onnx", "D"}})
  {
    Rig rig(c.id, params(c.file, c.mode));
    RLController ctrl;
    EXPECT_THROW(ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap), std::runtime_error)
      << c.id;
  }
}

TEST(RLControllerPlugin, TimeoutFallsBackToDwaAndSpeedLimitSetsVcap)
{
  REQUIRE_MODELS();
  Rig rig("t", params(
      "fixture_c.onnx", "C", {rclcpp::Parameter("FollowPath.rl.max_inference_ms", 1e-6)}));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 6.0));
  ctrl.setSpeedLimit(50.0, true);
  geometry_msgs::msg::Twist vel;
  for (int k = 0; k < 6; ++k) {
    rig.publishScan();
    const auto cmd = ctrl.computeVelocityCommands(pose(1.0, 3.0, 0.0), vel, nullptr);
    EXPECT_EQ(ctrl.lastResult().source, Source::kDwa);
    EXPECT_EQ(ctrl.lastResult().dwa_reason, DwaReason::kTimeout);
    EXPECT_LE(cmd.twist.linear.x, 0.5 + 1e-9);
  }
  EXPECT_EQ(ctrl.counters().dwa[3], 3U);
  rig.spin();
  ASSERT_FALSE(rig.stats.empty());
  EXPECT_NEAR(rig.stats.back().data[21], 0.5, 1e-9);   // v_cap = 1.0 × 50 %
}

TEST(RLControllerPlugin, WithoutDwaFallbackBrakesByContractRule)
{
  REQUIRE_MODELS();
  Rig rig("n", params(
      "fixture_c.onnx", "C", {rclcpp::Parameter("FollowPath.rl.dwa_fallback", false)}));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 6.0));
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.6;
  vel.angular.z = 0.3;
  // 스캔 없음: 재동기 v 0.6 → 틱마다 a 1.0 × 0.05 감속, ω 는 α 2.0 × 0.05 씩 0 쪽으로
  auto cmd = ctrl.computeVelocityCommands(pose(1.0, 3.0, 0.0), vel, nullptr);
  EXPECT_NEAR(cmd.twist.linear.x, 0.55, 1e-9);
  EXPECT_NEAR(cmd.twist.angular.z, 0.2, 1e-9);
  cmd = ctrl.computeVelocityCommands(pose(1.0, 3.0, 0.0), vel, nullptr);
  EXPECT_NEAR(cmd.twist.linear.x, 0.50, 1e-9);
  EXPECT_NEAR(cmd.twist.angular.z, 0.1, 1e-9);
}

TEST(RLControllerPlugin, StaleOrMalformedScanFallsBackToDwa)
{
  REQUIRE_MODELS();
  Rig rig("s", params("fixture_c.onnx", "C"));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 6.0));
  geometry_msgs::msg::Twist vel;
  // 나이 1.0 s > scan_timeout 0.3 s, 그다음 빔 719 개: 두 결정 모두 입력 무효 → DWA
  rig.publishScan(5.0, 1.0);
  for (int k = 0; k < 2; ++k) {
    ctrl.computeVelocityCommands(pose(1.0, 3.0, 0.0), vel, nullptr);
    EXPECT_EQ(ctrl.lastResult().dwa_reason, DwaReason::kStateInvalid);
  }
  rig.publishScan(5.0, 0.0, C::kLidarSamples - 1);
  for (int k = 0; k < 2; ++k) {
    ctrl.computeVelocityCommands(pose(1.0, 3.0, 0.0), vel, nullptr);
    EXPECT_EQ(ctrl.lastResult().dwa_reason, DwaReason::kStateInvalid);
  }
  EXPECT_EQ(ctrl.counters().dwa[1], 2U);
  // 정상 스캔 → 정책
  rig.publishScan(5.0);
  ctrl.computeVelocityCommands(pose(1.0, 3.0, 0.0), vel, nullptr);
  EXPECT_NE(ctrl.lastResult().source, Source::kDwa);
}

TEST(RLControllerPlugin, NoPlanThrows)
{
  REQUIRE_MODELS();
  Rig rig("p", params("fixture_c.onnx", "C"));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  EXPECT_THROW(
    ctrl.computeVelocityCommands(pose(1.0, 3.0, 0.0), geometry_msgs::msg::Twist(), nullptr),
    nav2_core::PlannerException);
}

// 1차 물결 리뷰 (낡은 창 중심, 낮은 쪽): DWA 2 틱(내부 v_last 0.10) → 정책이 0.30 까지 가속 → 낡은
// 스캔 → DWA. 고치기 전에는 DWA 가 0.10 중심 창에서 골라 0.30 → 0.15 (−3.0 m/s²) 로 뛰었다.
// 동기화 뒤에는 DWA 창 = 계약 창이라 자름(commitDwa)도 필요 없다 (A2, syncLastCommand).
TEST(RLControllerPlugin, PolicyToDwaHandoverKeepsContractStepLimits)
{
  REQUIRE_MODELS();
  Rig rig("h", params("heuristic_c.onnx", "C"));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 8.0));
  Loop loop(ctrl);
  for (int k = 0; k < 2; ++k) {
    ASSERT_EQ(loop.step().source, Source::kDwa) << k;   // 스캔 없음
  }
  bool reached = false;
  for (int k = 0; k < 200 && !reached; ++k) {
    rig.publishScan();
    const auto r = loop.step();
    ASSERT_NE(r.source, Source::kDwa) << k << " " << r.error;
    reached = loop.vel.linear.x >= 0.30 - 1e-9 && !r.new_decision;   // 결정 주기 끝
  }
  ASSERT_TRUE(reached);
  ASSERT_LE(loop.vel.linear.x, 0.10 + 0.3);   // 내부 DWA 가 측정값으로 창을 다시 잡지 않는 범위
  const auto syncs0 = ctrl.counters().dwa_window_syncs;
  const auto limited0 = ctrl.counters().dwa_limited;
  for (int j = 0; j < 6; ++j) {
    rig.publishScan(std::numeric_limits<double>::infinity(), 1.0);   // 낡은 스캔 → DWA
    const auto r = loop.step();
    ASSERT_EQ(r.source, Source::kDwa) << j;
    EXPECT_FALSE(r.dwa_limited) << j;
    if (j == 0) {
      EXPECT_EQ(ctrl.counters().dwa_window_syncs, syncs0 + 1U);   // 전환 틱에 창 중심 맞춤
    }
  }
  EXPECT_LE(loop.max_dv, kDvTick + 1e-9);
  EXPECT_LE(loop.max_dw, kDwTick + 1e-9);
  EXPECT_EQ(ctrl.counters().dwa_window_syncs, syncs0 + 1U);   // 그 뒤는 DWA 명령 = 낸 명령
  EXPECT_EQ(ctrl.counters().dwa_limited, limited0);
  rig.spin();
  ASSERT_FALSE(rig.stats.empty());
  EXPECT_NEAR(
    rig.stats.back().data[25], static_cast<double>(ctrl.counters().dwa_window_syncs), 1e-12);
  EXPECT_NEAR(rig.stats.back().data[24], static_cast<double>(ctrl.counters().dwa_limited), 1e-12);
}

// 1차 물결 리뷰 (낡은 창 중심, 높은 쪽 + 속도 제한): DWA 8 틱(내부 v_last 0.40) → 속도 제한
// 0.20 아래로 정책 구간이 감속 → 낡은 스캔 → DWA. 고치기 전에는 DWA 가 0.40 중심 창의 "최대
// 감속" 0.35 를 내 제한 0.20 위로 다시 가속했다. 동기화 뒤에는 0.20 중심 창이라 자름 없이
// 제한 아래다.
TEST(RLControllerPlugin, DwaAfterPolicyDecelerationStaysUnderSpeedLimit)
{
  REQUIRE_MODELS();
  Rig rig("hs", params("heuristic_c.onnx", "C"));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 8.0));
  Loop loop(ctrl);
  for (int k = 0; k < 8; ++k) {
    ASSERT_EQ(loop.step().source, Source::kDwa) << k;   // 스캔 없음
  }
  ASSERT_GT(loop.vel.linear.x, 0.30);   // 내부 DWA 의 v_last
  ctrl.setSpeedLimit(0.20, false);
  bool reached = false;
  for (int k = 0; k < 40 && !reached; ++k) {
    rig.publishScan();
    const auto r = loop.step();
    ASSERT_NE(r.source, Source::kDwa) << k << " " << r.error;
    reached = loop.vel.linear.x <= 0.20 + 1e-9 && !r.new_decision;
  }
  ASSERT_TRUE(reached);
  ASSERT_GE(loop.vel.linear.x, 0.40 - 0.3);   // 내부 DWA 가 낡은 v_last 를 창 중심으로 쓰는 범위
  loop.resetMax();
  const auto syncs0 = ctrl.counters().dwa_window_syncs;
  const auto limited0 = ctrl.counters().dwa_limited;
  for (int j = 0; j < 8; ++j) {
    rig.publishScan(std::numeric_limits<double>::infinity(), 1.0);
    const auto r = loop.step();
    ASSERT_EQ(r.source, Source::kDwa) << j;
    EXPECT_LE(loop.vel.linear.x, 0.20 + 1e-9) << j;
  }
  EXPECT_LE(loop.max_dv, kDvTick + 1e-9);
  EXPECT_LE(loop.max_dw, kDwTick + 1e-9);
  EXPECT_GE(ctrl.counters().dwa_window_syncs, syncs0 + 1U);
  EXPECT_EQ(ctrl.counters().dwa_limited, limited0);
}

/// 정책 구간을 끝까지 돌린 뒤의 상태: DWA (스캔 없음) n_dwa 틱 → 속도 제한 0.20 아래로 정책 감속.
void dwaThenPolicyDecel(Rig & rig, RLController & ctrl, Loop & loop, int n_dwa)
{
  for (int k = 0; k < n_dwa; ++k) {
    ASSERT_EQ(loop.step().source, Source::kDwa) << k;
  }
  ctrl.setSpeedLimit(0.20, false);
  bool reached = false;
  for (int k = 0; k < 40 && !reached; ++k) {
    rig.publishScan();
    const auto r = loop.step();
    ASSERT_NE(r.source, Source::kDwa) << k << " " << r.error;
    reached = loop.vel.linear.x <= 0.20 + 1e-9 && !r.new_decision;
  }
  ASSERT_TRUE(reached);
}

// 1차 물결 리뷰 (낡은 창 중심 + 제동): DWA 8 틱(내부 v_last 0.40) → 속도 제한 0.20 아래로 정책 감속
// → 제한 해제, 바로 앞에 벽 → 낡은 스캔 → DWA. 유효 후보가 없으면 DWA 는 "창 중심 − 감속" 을 낸다
// (dwa.cpp 제동 후보). 중심이 낡은 0.40 이면 그 제동이 0.35 (수정 전) — 자르기만 해도
// 0.25 → 0.30 으로 벽 쪽 가속이었다. 창 중심을 실제 직전 명령 0.20 에 맞추면 거기서 제동한다.
TEST(RLControllerPlugin, DwaBrakeAfterPolicyStretchDoesNotAccelerateTowardWall)
{
  REQUIRE_MODELS();
  Rig rig("hw", params("heuristic_c.onnx", "C"));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 8.0));
  Loop loop(ctrl);
  dwaThenPolicyDecel(rig, ctrl, loop, 8);
  if (HasFatalFailure()) {
    return;
  }
  ctrl.setSpeedLimit(0.0, false);   // NO_SPEED_LIMIT
  // 로봇 앞면(+0.30 m)에서 0.20 m 앞, 폭 2 m 벽. 이 시험 코스트맵은 갱신 루프가 없어 팽창이
  // 안 돌고, DWA 풋프린트 검사는 중심 셀 비용이 외접 비용보다 낮으면 외곽선을 보지 않는다
  // (footprint.cpp:36) → 벽 앞 0.36 m (외접 반경) 띠를 내접 팽창 비용 253 으로 직접 칠한다.
  // 중심이 그 띠에 들어가는 롤아웃 = 충돌. 0.15 m/s 이상 1.5 s 롤아웃은 모두 닿는다.
  auto * cm = rig.costmap->getCostmap();
  const double wx = loop.x + 0.30 + 0.20;
  int n_cells = 0;
  for (double y = 1.5; y <= 4.5 + 1e-9; y += 0.025) {
    for (double x = wx - 0.36; x <= wx + 0.10 + 1e-9; x += 0.025) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (cm->worldToMap(x, y, mx, my)) {
        const bool wall = x >= wx && y >= 2.0 && y <= 4.0;
        cm->setCost(
          mx, my,
          wall ? nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
        ++n_cells;
      }
    }
  }
  ASSERT_GT(n_cells, 0);
  loop.resetMax();
  const double v0 = loop.vel.linear.x;
  for (int j = 0; j < 6; ++j) {
    rig.publishScan(std::numeric_limits<double>::infinity(), 1.0);
    const double v_prev = loop.vel.linear.x;
    const auto r = loop.step();
    ASSERT_EQ(r.source, Source::kDwa) << j;
    EXPECT_LE(loop.vel.linear.x, v_prev + 1e-9) << j << " 벽 앞에서 가속";
  }
  EXPECT_LT(loop.vel.linear.x, v0 - 0.1);
  EXPECT_LE(loop.max_dv, kDvTick + 1e-9);
  EXPECT_LE(loop.max_dw, kDwTick + 1e-9);
}

// 추종 지연이 window_reset 안 (A2): 측정이 명령보다 0.15 m/s 늦어도 (< window_reset_v 0.3) DWA 는
// 동기화한 직전 명령을 창 중심으로 쓴다 → 명령이 직전 명령 ± a_max·Δt_c 안이라 자르지 않는다.
// 1차 물결(activate 우회)에서는 창이 측정 속도 중심이라 0.15 떨어진 창에서 골라 매번 잘랐다.
TEST(RLControllerPlugin, DwaUsesSyncedCenterWhenMeasuredSpeedLags)
{
  REQUIRE_MODELS();
  Rig rig("hl", params("heuristic_c.onnx", "C"));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 8.0));
  Loop loop(ctrl);
  for (int k = 0; k < 2; ++k) {
    ASSERT_EQ(loop.step().source, Source::kDwa) << k;
  }
  bool reached = false;
  for (int k = 0; k < 200 && !reached; ++k) {
    rig.publishScan();
    const auto r = loop.step();
    ASSERT_NE(r.source, Source::kDwa) << k << " " << r.error;
    reached = loop.vel.linear.x >= 0.30 - 1e-9 && !r.new_decision;
  }
  ASSERT_TRUE(reached);
  const double v_cmd = loop.vel.linear.x;
  const auto syncs0 = ctrl.counters().dwa_window_syncs;
  const auto limited0 = ctrl.counters().dwa_limited;
  geometry_msgs::msg::Twist lag;
  lag.linear.x = v_cmd - 0.15;   // 측정이 명령보다 늦다 (window_reset_v 0.3 안)
  rig.publishScan(std::numeric_limits<double>::infinity(), 1.0);
  auto cmd = ctrl.computeVelocityCommands(pose(loop.x, 3.0, 0.0), lag, nullptr);
  auto r = ctrl.lastResult();
  ASSERT_EQ(r.source, Source::kDwa);
  EXPECT_FALSE(r.dwa_limited);
  EXPECT_LE(std::abs(cmd.twist.linear.x - v_cmd), kDvTick + 1e-9);
  EXPECT_EQ(ctrl.counters().dwa_window_syncs, syncs0 + 1U);
  const double v1 = cmd.twist.linear.x;
  lag.linear.x = v1 - 0.15;
  rig.publishScan(std::numeric_limits<double>::infinity(), 1.0);
  cmd = ctrl.computeVelocityCommands(pose(loop.x, 3.0, 0.0), lag, nullptr);
  EXPECT_FALSE(ctrl.lastResult().dwa_limited);
  EXPECT_EQ(ctrl.counters().dwa_window_syncs, syncs0 + 1U);   // DWA 명령 = 낸 명령 → 맞출 것 없음
  EXPECT_EQ(ctrl.counters().dwa_limited, limited0);
  EXPECT_LE(std::abs(cmd.twist.linear.x - v1), kDvTick + 1e-9);
}

// 추종 지연이 window_reset 밖 (A2): 측정이 명령보다 0.35 m/s 늦으면 (> window_reset_v 0.3) DWA 는
// 자기 규칙대로 측정 속도로 창을 다시 잡는다 → 그 명령은 직전 명령에서 a_max·Δt_c 넘게 떨어져
// 계약 동적 창으로 자른다 (commitDwa — 계약 불변식). 자른 명령은 DWA 가 기억한 것과 달라 다음
// 틱에 다시 맞춘다.
TEST(RLControllerPlugin, DwaCommandIsClampedWhenMeasuredLagExceedsWindowReset)
{
  REQUIRE_MODELS();
  Rig rig("hx", params("heuristic_c.onnx", "C"));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 8.0));
  Loop loop(ctrl);
  for (int k = 0; k < 2; ++k) {
    ASSERT_EQ(loop.step().source, Source::kDwa) << k;
  }
  bool reached = false;
  for (int k = 0; k < 300 && !reached; ++k) {
    rig.publishScan();
    const auto r = loop.step();
    ASSERT_NE(r.source, Source::kDwa) << k << " " << r.error;
    reached = loop.vel.linear.x >= 0.45 - 1e-9 && !r.new_decision;
  }
  ASSERT_TRUE(reached);
  const double v_cmd = loop.vel.linear.x;
  const auto syncs0 = ctrl.counters().dwa_window_syncs;
  geometry_msgs::msg::Twist lag;
  lag.linear.x = v_cmd - 0.35;
  rig.publishScan(std::numeric_limits<double>::infinity(), 1.0);
  auto cmd = ctrl.computeVelocityCommands(pose(loop.x, 3.0, 0.0), lag, nullptr);
  auto r = ctrl.lastResult();
  ASSERT_EQ(r.source, Source::kDwa);
  EXPECT_TRUE(r.dwa_limited);
  EXPECT_NEAR(cmd.twist.linear.x, v_cmd - kDvTick, 1e-9);
  EXPECT_EQ(ctrl.counters().dwa_window_syncs, syncs0 + 1U);
  const double v1 = cmd.twist.linear.x;
  lag.linear.x = v1 - 0.35;
  rig.publishScan(std::numeric_limits<double>::infinity(), 1.0);
  cmd = ctrl.computeVelocityCommands(pose(loop.x, 3.0, 0.0), lag, nullptr);
  EXPECT_EQ(ctrl.counters().dwa_window_syncs, syncs0 + 2U);   // 자른 명령으로 다시 맞춤
  EXPECT_LE(std::abs(cmd.twist.linear.x - v1), kDvTick + 1e-9);
}

// ω 만 어긋난 전환 (A2 — 1차 물결 변이 R3 "동기 판정에서 ω 비교 생략" 이 살아남았다): 경로 방향과
// 0.6 rad 틀어진 자세에서 DWA 2 틱 뒤 정책이 돌면서 정렬 → DWA 의 기억 ω 와 실제 ω 가
// 0.3–0.45 rad/s 어긋난 순간 (window_reset_w 0.5 안) 낡은 스캔 → DWA. ω 를 맞추지 않으면 DWA 가
// 낡은 ω 중심 창(± α·Δt_c = 0.1)에서 골라 실제 ω 와 0.2 넘게 떨어져 → 계약 창 밖 → 자름이 생긴다.
// (A2 변이 검사 1 회차: 문턱을 0.2 로 두었더니 ω 가 정확히 0.1 차로 창 경계에 닿아 "v 만 sync"
// 변이가 살아남았다 → 0.3 으로 올렸다.)
TEST(RLControllerPlugin, OmegaOnlyMismatchIsSyncedWithoutClamp)
{
  REQUIRE_MODELS();
  Rig rig("hw2", params("heuristic_c.onnx", "C"));
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 8.0));
  geometry_msgs::msg::Twist vel;
  double x = 1.0;
  double y = 3.0;
  double th = -0.6;
  auto step = [&]() {
      const auto cmd = ctrl.computeVelocityCommands(pose(x, y, th), vel, nullptr);
      vel = cmd.twist;
      x += vel.linear.x * std::cos(th) * 0.05;
      y += vel.linear.x * std::sin(th) * 0.05;
      th += vel.angular.z * 0.05;
      return ctrl.lastResult();
    };
  double w_dwa = 0.0;
  for (int k = 0; k < 2; ++k) {
    ASSERT_EQ(step().source, Source::kDwa) << k;
    w_dwa = vel.angular.z;   // 내부 DWA 가 기억하는 ω
  }
  bool ready = false;
  for (int k = 0; k < 200 && !ready; ++k) {
    rig.publishScan();
    const auto r = step();
    ASSERT_NE(r.source, Source::kDwa) << k << " " << r.error;
    const double dw = std::abs(vel.angular.z - w_dwa);
    ready = !r.new_decision && dw >= 0.3 && dw <= 0.45;
  }
  ASSERT_TRUE(ready) << "정책 구간에서 ω 차 0.3–0.45 를 만들지 못했다";
  const auto syncs0 = ctrl.counters().dwa_window_syncs;
  const auto limited0 = ctrl.counters().dwa_limited;
  double max_dw = 0.0;
  for (int j = 0; j < 4; ++j) {
    rig.publishScan(std::numeric_limits<double>::infinity(), 1.0);
    const double w_prev = vel.angular.z;
    const auto r = step();
    ASSERT_EQ(r.source, Source::kDwa) << j;
    max_dw = std::max(max_dw, std::abs(vel.angular.z - w_prev));
  }
  EXPECT_EQ(ctrl.counters().dwa_window_syncs, syncs0 + 1U);
  EXPECT_EQ(ctrl.counters().dwa_limited, limited0);
  EXPECT_LE(max_dw, kDwTick + 1e-9);
}

// 긴 정책 구간 뒤 DWA 폴백 (A2 에서 찾음): 내부 DWA 의 경로 창(PlanWindow)은 DWA 가 돈 틱에만
// 앞으로 간다 — 직전 색인에서 앞쪽 200 점(10 m)만 찾으므로 (새 경로면 앞 400 점), 정책이 그보다
// 멀리 몬 뒤 DWA 를 부르면 로봇 뒤쪽 점을 최근접으로 잡는다. 30 m 고정 경로(FollowPath, BT 재계획
// 없음)에서 정책이 20 m 를 몬 뒤 낡은 스캔 → DWA: 경로를 따라 계속 가야 한다 (돌아서거나 멈추면
// 안 된다).
TEST(RLControllerPlugin, DwaAfterLongPolicyStretchTracksPathAhead)
{
  REQUIRE_MODELS();
  Rig rig("lp", params("heuristic_c.onnx", "C"), 40);
  RLController ctrl;
  ctrl.configure(rig.node, "FollowPath", rig.tf, rig.costmap);
  ctrl.activate();
  ctrl.setPlan(straightPlan(1.0, 3.0, 30.0));
  Loop loop(ctrl);
  for (int k = 0; k < 2; ++k) {
    ASSERT_EQ(loop.step().source, Source::kDwa) << k;   // 스캔 없음 → DWA (경로 창 색인 ≈ 0)
  }
  bool far = false;
  for (int k = 0; k < 3000 && !far; ++k) {
    rig.publishScan();
    const auto r = loop.step();
    ASSERT_NE(r.source, Source::kDwa) << k << " " << r.error;
    far = loop.x >= 21.0 && !r.new_decision;
  }
  ASSERT_TRUE(far) << loop.x;
  const double v0 = loop.vel.linear.x;
  ASSERT_GT(v0, 0.3);
  double min_v = v0;
  double max_abs_w = 0.0;
  for (int j = 0; j < 20; ++j) {
    rig.publishScan(std::numeric_limits<double>::infinity(), 1.0);
    const auto r = loop.step();
    ASSERT_EQ(r.source, Source::kDwa) << j;
    min_v = std::min(min_v, loop.vel.linear.x);
    max_abs_w = std::max(max_abs_w, std::abs(loop.vel.angular.z));
  }
  EXPECT_GE(min_v, v0 - 0.05) << "DWA 가 경로 앞쪽을 보지 못하고 감속했다";
  EXPECT_LT(max_abs_w, 0.05) << "DWA 가 뒤쪽 경로로 돌아서려 했다";
}
