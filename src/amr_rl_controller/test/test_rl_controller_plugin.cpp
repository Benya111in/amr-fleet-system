// RLController 플러그인 ROS 계층 시험 (Gazebo 없음).
// 실제 Costmap2DROS(빈 방) 와 LifecycleNode 위에서 configure → activate → setPlan →
// computeVelocityCommands 를 부르고, 스캔은 같은 프로세스의 발행자로 넣는다.
//  - pluginlib 로 이름 로드 (controller_server 와 같은 경로)
//  - 스캔 없음 → DWA 폴백(입력 무효), 스캔 있음 → 정책 결정(C·B), 명령 한계
//  - 계약 위반 모델·모드 불일치·빈 경로 → configure 예외
//  - 추론 시간 초과 → DWA, DWA 폴백 끔 → 계약 제동 규칙, setSpeedLimit → v_cap
#include <gtest/gtest.h>

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

std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeCostmap(const std::string & name)
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
      rclcpp::Parameter("width", 10),
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

  explicit Rig(const std::string & id, const std::vector<rclcpp::Parameter> & params)
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "rl_test_" + id, rclcpp::NodeOptions().parameter_overrides(params));
    costmap = makeCostmap("rl_costmap_" + id);
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
  EXPECT_EQ(rig.stats.back().data.size(), 24U);
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
