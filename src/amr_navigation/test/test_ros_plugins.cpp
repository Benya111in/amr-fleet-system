// Nav2 플러그인(AStarPlanner, DWAController, PurePursuitController)과
//   velocity_profiler_node 의 ROS 계층 시험.
// 실제 Costmap2DROS 를 configure 한 뒤 코스트맵 값을 직접 채워 넣고(업데이트 스레드 없음)
//   플러그인을 호출한다.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "amr_navigation/astar_planner.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/dwa_controller.hpp"
#include "amr_navigation/pure_pursuit_controller.hpp"
#include "amr_navigation/ros_utils.hpp"
#include "amr_navigation/velocity_profiler_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"

using amr_navigation::core::OwnedGrid;
using amr_navigation::core::kLethalObstacle;

namespace
{
class RosEnv : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

struct Rect
{
  double x0, y0, x1, y1;
};

// 10 x 6 m 방, 가운데 선반 두 개 사이 1.2 m 통로 (Nav2 식 팽창 0.2 / 1.2 / 2.0).
// extra: 추가 장애물 사각형 [m]. 실제 InflationLayer 처럼 모든 장애물을 팽창해서 넣는다
// (풋프린트 검사의 외접 비용 조기 판정은 팽창된 코스트맵을 전제로 한다).
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> makeCostmap(
  const std::string & name, bool with_racks = true, const std::vector<Rect> & extra = {})
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
      rclcpp::Parameter("inflation_layer.inflation_radius", 1.2),
      rclcpp::Parameter("inflation_layer.cost_scaling_factor", 2.0),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 6),
      rclcpp::Parameter("resolution", 0.05),
    });
  auto cm = std::make_shared<nav2_costmap_2d::Costmap2DROS>(opts);
  cm->on_configure(rclcpp_lifecycle::State());
  OwnedGrid g(200, 120, 0.05);
  if (with_racks) {
    g.fillRect(0.0, 0.0, 10.0, 0.05, kLethalObstacle);
    g.fillRect(0.0, 5.95, 10.0, 6.0, kLethalObstacle);
    g.fillRect(3.0, 0.0, 7.0, 2.4, kLethalObstacle);
    g.fillRect(3.0, 3.6, 7.0, 6.0, kLethalObstacle);
  }
  for (const auto & r : extra) {
    g.fillRect(r.x0, r.y0, r.x1, r.y1, kLethalObstacle);
  }
  amr_navigation::core::inflate(g, 0.2, 1.2, 2.0);
  auto * costmap = cm->getCostmap();
  EXPECT_EQ(costmap->getSizeInCellsX(), 200U);
  EXPECT_EQ(costmap->getSizeInCellsY(), 120U);
  for (int y = 0; y < 120; ++y) {
    for (int x = 0; x < 200; ++x) {
      costmap->setCost(x, y, g.at(x, y));
    }
  }
  return cm;
}

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose = amr_navigation::toPoseMsg({x, y, yaw});
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
}  // namespace

TEST(AStarPlannerPlugin, PlansThroughAisleAndHandlesErrors)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("astar_test");
  auto cm = makeCostmap("astar_costmap");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  amr_navigation::AStarPlanner planner;
  planner.configure(node, "AStar", tf, cm);
  planner.activate();
  const auto path = planner.createPlan(pose(1.0, 1.0, 0.0), pose(9.0, 5.0, 0.5));
  ASSERT_GT(path.poses.size(), 100U);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_NEAR(path.poses.front().pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.y, 5.0, 1e-9);
  EXPECT_NEAR(amr_navigation::yawOf(path.poses.back().pose), 0.5, 1e-6);
  // 통로 (x 3~7) 안에서는 중앙(y 3.0) 근처를 지난다
  for (const auto & p : path.poses) {
    if (p.pose.position.x > 3.5 && p.pose.position.x < 6.5) {
      EXPECT_NEAR(p.pose.position.y, 3.0, 0.15);
    }
  }
  // 같은 셀 → 두 점
  EXPECT_EQ(planner.createPlan(pose(1.0, 1.0, 0.0), pose(1.01, 1.01, 0.0)).poses.size(), 2U);
  // 오류: 지도 밖, 다른 프레임, 막힌 목표(허용 오차 밖)
  EXPECT_THROW(
    planner.createPlan(pose(1.0, 1.0, 0.0), pose(20.0, 1.0, 0.0)), nav2_core::PlannerException);
  auto wrong = pose(1.0, 1.0, 0.0);
  wrong.header.frame_id = "odom";
  EXPECT_THROW(planner.createPlan(wrong, pose(2.0, 1.0, 0.0)), nav2_core::PlannerException);
  EXPECT_THROW(
    planner.createPlan(pose(1.0, 1.0, 0.0), pose(5.0, 1.0, 0.0)), nav2_core::PlannerException);
  // 동적 파라미터: 허용 오차를 키우면 막힌 목표 근처로 대체
  node->set_parameter(rclcpp::Parameter("AStar.tolerance", 1.5));
  node->set_parameter(rclcpp::Parameter("AStar.cost_weight", 0.0));
  node->set_parameter(rclcpp::Parameter("AStar.smoother.w_smooth", 0.2));
  const auto near = planner.createPlan(pose(1.0, 1.0, 0.0), pose(5.0, 1.2, 0.0));
  EXPECT_GE(near.poses.size(), 2U);
  // 시작이 지도 밖
  EXPECT_THROW(
    planner.createPlan(pose(-1.0, 1.0, 0.0), pose(2.0, 1.0, 0.0)), nav2_core::PlannerException);
  planner.deactivate();
  planner.cleanup();
}

TEST(DWAControllerPlugin, AcceleratesAlongPlanAndRespectsLimits)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dwa_test");
  node->declare_parameter("controller_frequency", 20.0);
  node->declare_parameter("limits.max_linear_acceleration", 1.0);
  auto cm = makeCostmap("dwa_costmap", false);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  amr_navigation::DWAController dwa;
  dwa.configure(node, "DWA", tf, cm);
  dwa.activate();
  geometry_msgs::msg::Twist vel;
  EXPECT_THROW(
    dwa.computeVelocityCommands(pose(1.0, 3.0, 0.0), vel, nullptr),
    nav2_core::PlannerException);
  dwa.setPlan(straightPlan(1.0, 3.0, 7.0));
  double v = 0.0;
  for (int k = 0; k < 20; ++k) {
    vel.linear.x = v;
    const auto cmd = dwa.computeVelocityCommands(pose(1.0 + 0.02 * k, 3.0, 0.0), vel, nullptr);
    EXPECT_LE(cmd.twist.linear.x, v + 0.05 + 1e-9);   // 동적 창: a·Δt = 0.05
    EXPECT_NEAR(cmd.twist.angular.z, 0.0, 0.05);
    v = cmd.twist.linear.x;
  }
  EXPECT_GT(v, 0.9);
  // 속도 제한: 절대값 0.3 → 창이 허용하는 만큼씩 감속
  dwa.setSpeedLimit(0.3, false);
  for (int k = 0; k < 20; ++k) {
    vel.linear.x = v;
    v = dwa.computeVelocityCommands(pose(2.0, 3.0, 0.0), vel, nullptr).twist.linear.x;
  }
  EXPECT_LE(v, 0.3 + 1e-9);
  dwa.setSpeedLimit(50.0, true);   // 백분율 50 % of 1.0
  dwa.setSpeedLimit(0.0, false);   // 제한 해제
  // 동적 장애물·적재 질량 토픽
  auto obs_pub = node->create_publisher<amr_msgs::msg::TrackedObstacleArray>(
    "perception/tracked_obstacles", 5);
  auto mass_pub = node->create_publisher<std_msgs::msg::Float32>(
    "payload/mass", rclcpp::QoS(1).transient_local());
  amr_msgs::msg::TrackedObstacleArray arr;
  arr.header.frame_id = "map";
  arr.header.stamp = node->get_clock()->now();
  amr_msgs::msg::TrackedObstacle o;
  o.position.x = 4.0;
  o.position.y = 3.0;
  o.velocity.x = -1.0;
  arr.obstacles.push_back(o);
  std_msgs::msg::Float32 mass;
  mass.data = 25.0F;
  for (int k = 0; k < 10; ++k) {
    obs_pub->publish(arr);
    mass_pub->publish(mass);
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  vel.linear.x = 0.5;
  const auto cmd = dwa.computeVelocityCommands(pose(2.0, 3.0, 0.0), vel, nullptr);
  EXPECT_LE(cmd.twist.linear.x, 0.5 + 0.05 * 47.6 / 72.6 + 1e-9);   // 적재 25 kg → 가속 한계 축소
  dwa.deactivate();
  dwa.cleanup();
}

TEST(DWAControllerPlugin, ThrowsWhenStoppedAndBlocked)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dwa_block_test");
  // x 2.30~3.0 벽 (팽창 포함)
  auto cm = makeCostmap("dwa_block_costmap", false, {{2.30, 0.0, 3.0, 6.0}});
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  amr_navigation::DWAController dwa;
  node->declare_parameter("DWA.no_valid_patience", 3);
  dwa.configure(node, "DWA", tf, cm);
  dwa.activate();
  dwa.setPlan(straightPlan(1.0, 3.0, 5.0));
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.0;
  // 벽에서 떨어진 자세에서는 정상 명령
  EXPECT_NO_THROW(dwa.computeVelocityCommands(pose(1.5, 3.0, 0.0), vel, nullptr));
  // 풋프린트가 이미 벽에 걸린 자세(앞 변 x 2.40): 정지·제자리 회전까지 모든 샘플 충돌 →
  // 제동 명령을 내다가 patience(3) 주기 뒤 정지 상태이면 예외 → BT 복구
  bool thrown = false;
  for (int k = 0; k < 10 && !thrown; ++k) {
    try {
      dwa.computeVelocityCommands(pose(2.1, 3.0, 0.0), vel, nullptr);
    } catch (const nav2_core::PlannerException &) {
      thrown = true;
    }
  }
  EXPECT_TRUE(thrown);
}

TEST(PurePursuitControllerPlugin, TracksAndDetectsCollision)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("pp_test");
  auto cm = makeCostmap("pp_costmap", false);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  amr_navigation::PurePursuitController pp;
  pp.configure(node, "PurePursuit", tf, cm);
  pp.activate();
  geometry_msgs::msg::Twist vel;
  EXPECT_THROW(
    pp.computeVelocityCommands(pose(1.0, 3.0, 0.0), vel, nullptr),
    nav2_core::PlannerException);
  pp.setPlan(straightPlan(1.0, 3.0, 6.0));
  vel.linear.x = 0.5;
  auto cmd = pp.computeVelocityCommands(pose(1.5, 3.1, 0.0), vel, nullptr);
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_LT(cmd.twist.angular.z, 0.0);   // 경로 오른쪽으로
  pp.setSpeedLimit(0.2, false);
  cmd = pp.computeVelocityCommands(pose(1.5, 3.0, 0.0), vel, nullptr);
  EXPECT_LE(cmd.twist.linear.x, 0.2 + 1e-9);
  pp.setSpeedLimit(0.0, false);
  // 적재 질량 → 가속 한계 축소 (예외 없이 반영)
  auto mass_pub = node->create_publisher<std_msgs::msg::Float32>(
    "payload/mass", rclcpp::QoS(1).transient_local());
  std_msgs::msg::Float32 mass;
  mass.data = 10.0F;
  mass_pub->publish(mass);
  rclcpp::spin_some(node->get_node_base_interface());
  // 경로 위 장애물(앞 변 0.5 m 앞) → 명령 원호 충돌 예외 (같은 설정의 두 번째 코스트맵)
  auto cm2 = makeCostmap("pp_costmap_blocked", false, {{2.30, 2.5, 2.50, 3.5}});
  amr_navigation::PurePursuitController pp2;
  pp2.configure(node, "PurePursuit", tf, cm2);
  pp2.activate();
  pp2.setPlan(straightPlan(1.0, 3.0, 6.0));
  EXPECT_THROW(
    pp2.computeVelocityCommands(pose(1.5, 3.0, 0.0), vel, nullptr),
    nav2_core::PlannerException);
  pp2.deactivate();
  pp2.cleanup();
  pp.deactivate();
  pp.cleanup();
}

TEST(VelocityProfilerNode, SmoothsStepCommand)
{
  rclcpp::NodeOptions opts;
  // 이 시험의 가짜 플랜트(측정 = 직전 출력, 지연 0)는 기준 모델(지연 0.12 s)과 달라
  // PI 가 보정한다 →
  // 프로파일 자체의 단조성을 보려고 PI 를 끈다 (PI 는 test_profiler.cpp 에서 검증)
  opts.parameter_overrides({rclcpp::Parameter("rate", 50.0), rclcpp::Parameter("use_pid", false)});
  auto prof = std::make_shared<amr_navigation::VelocityProfilerNode>(opts);
  auto io = std::make_shared<rclcpp::Node>("profiler_io");
  auto pub = io->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", rclcpp::QoS(1));
  auto mass_pub = io->create_publisher<std_msgs::msg::Float32>(
    "payload/mass", rclcpp::QoS(1).transient_local());
  auto odom_pub = io->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 5);
  std::vector<double> vs;
  auto sub = io->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_smoothed", 10, [&vs](const geometry_msgs::msg::Twist::SharedPtr m) {
      vs.push_back(m->linear.x);
    });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(prof);
  exec.add_node(io);
  std_msgs::msg::Float32 mass;
  mass.data = 0.0F;
  mass_pub->publish(mass);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  cmd.angular.z = 0.5;
  nav_msgs::msg::Odometry odom;
  const auto t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
  while (std::chrono::steady_clock::now() < t_end) {
    pub->publish(cmd);
    odom.twist.twist.linear.x = vs.empty() ? 0.0 : vs.back();
    odom_pub->publish(odom);
    exec.spin_some(std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(vs.size(), 10U);
  for (std::size_t i = 1; i < vs.size(); ++i) {
    EXPECT_GE(vs[i], vs[i - 1] - 1e-9);   // 단조 증가 (계단 명령에 급가속 없음)
  }
  EXPECT_LT(vs.back(), 1.0);
  EXPECT_GT(vs.back(), 0.05);
  EXPECT_GT(prof->profiler().linearFilter().velocity(), 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new RosEnv);
  return RUN_ALL_TESTS();
}
