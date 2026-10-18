// 플러그인 시험: 원본 amr_navigation::DWAController 와 amr_nav_research::DwaControllerX 를
// pluginlib 으로 **한 프로세스에** 올린다 (ROS 시험대의 controller_server 와 같은 상황).
//   1) 둘 다 올라오고, 같은 파라미터(기본값 + 배포 nav2_params 의 몇 값)·같은 입력이면
//      폐루프 명령열이 비트 단위로 같다.
//   2) research.* 파라미터가 읽히고(명령이 달라진다), research.log_resets 에 기록이 남는다.
// 코스트맵은 실제 Costmap2DROS 를 configure 한 뒤 값을 직접 채운다 (원본 test_ros_plugins 와 같다).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/pid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
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

// 10 x 6 m 방, 선반 두 개 사이 1.2 m 통로 (원본 test_ros_plugins makeCostmap 과 같다)
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
      rclcpp::Parameter("inflation_layer.inflation_radius", 1.2),
      rclcpp::Parameter("inflation_layer.cost_scaling_factor", 2.0),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 6),
      rclcpp::Parameter("resolution", 0.05),
    });
  auto cm = std::make_shared<nav2_costmap_2d::Costmap2DROS>(opts);
  cm->on_configure(rclcpp_lifecycle::State());
  OwnedGrid g(200, 120, 0.05);
  g.fillRect(0.0, 0.0, 10.0, 0.05, kLethalObstacle);
  g.fillRect(0.0, 5.95, 10.0, 6.0, kLethalObstacle);
  g.fillRect(3.0, 0.0, 7.0, 2.4, kLethalObstacle);
  g.fillRect(3.0, 3.6, 7.0, 6.0, kLethalObstacle);
  amr_navigation::core::inflate(g, 0.2, 1.2, 2.0);
  auto * costmap = cm->getCostmap();
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
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(0.5 * yaw);
  p.pose.orientation.w = std::cos(0.5 * yaw);
  return p;
}

// 통로를 지나 S 자로 휘는 경로 (x 1 → 9, 통로 y = 3)
nav_msgs::msg::Path plan()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double s = 0.0; s <= 8.0 + 1e-9; s += 0.05) {
    const double x = 1.0 + s;
    const double y = 3.0 + (x > 7.2 ? 0.6 * std::sin((x - 7.2) * 1.2) : 0.0);
    const double dy = x > 7.2 ? 0.72 * std::cos((x - 7.2) * 1.2) : 0.0;
    path.poses.push_back(pose(x, y, std::atan2(dy, 1.0)));
  }
  return path;
}

// 배포 nav2_params.yaml DWA 블록의 값 몇 개 (기본값과 다른 것) — 두 이름에 똑같이 선언
void declareDeployed(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & n)
{
  node->declare_parameter(n + ".off_path_band", 1.0);
  node->declare_parameter(n + ".path_horizon", 4.0);
  node->declare_parameter(n + ".yield_decision", false);
  node->declare_parameter(n + ".goal_align_distance", 0.08);
}

struct Loop
{
  /// tau: 서보 1차 시간 상수 [s]. 크면 가속 중 명령–측정 차가 재설정 임계 0.3 m/s 를 넘는다.
  explicit Loop(double tau = 0.08)
  : sv(1.0, tau, 0.04, 0.01, 0.0), sw(1.0, tau, 0.04, 0.01, 0.0) {}
  std::shared_ptr<nav2_core::Controller> ctrl;
  amr_navigation::core::FirstOrderPlant sv;
  amr_navigation::core::FirstOrderPlant sw;
  double x{1.0};
  double y{3.0};
  double th{0.0};

  geometry_msgs::msg::Twist step()
  {
    geometry_msgs::msg::Twist vel;
    vel.linear.x = sv.output();
    vel.angular.z = sw.output();
    const auto cmd = ctrl->computeVelocityCommands(pose(x, y, th), vel, nullptr);
    for (int k = 0; k < 5; ++k) {   // 1차 서보 (지연 0.04 s, τ) — 명령–측정 괴리를 만든다
      const double v = sv.step(cmd.twist.linear.x);
      const double w = sw.step(cmd.twist.angular.z);
      x += v * std::cos(th) * 0.01;
      y += v * std::sin(th) * 0.01;
      th += w * 0.01;
    }
    return cmd.twist;
  }
};

bool sameBits(double a, double b)
{
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}
}  // namespace

TEST(DwaControllerXPlugin, LoadsBesideTheOriginalAndMatchesItBitForBit)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dwa_x_equiv_test");
  node->declare_parameter("controller_frequency", 20.0);
  declareDeployed(node, "DWA");
  declareDeployed(node, "DWAX");
  auto cm = makeCostmap("dwa_x_costmap");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  Loop a;
  Loop b;
  ASSERT_NO_THROW(a.ctrl = loader.createSharedInstance("amr_navigation::DWAController"));
  ASSERT_NO_THROW(b.ctrl = loader.createSharedInstance("amr_nav_research::DwaControllerX"));
  a.ctrl->configure(node, "DWA", tf, cm);
  b.ctrl->configure(node, "DWAX", tf, cm);
  a.ctrl->activate();
  b.ctrl->activate();
  a.ctrl->setPlan(plan());
  b.ctrl->setPlan(plan());
  double vmax = 0.0;
  for (int k = 0; k < 220; ++k) {
    const auto ca = a.step();
    const auto cb = b.step();
    ASSERT_TRUE(sameBits(ca.linear.x, cb.linear.x)) << "cycle " << k << ": " << ca.linear.x <<
      " != " << cb.linear.x;
    ASSERT_TRUE(sameBits(ca.angular.z, cb.angular.z)) << "cycle " << k;
    vmax = std::max(vmax, ca.linear.x);
  }
  EXPECT_GT(vmax, 0.8);    // 실제로 달렸다
  EXPECT_GT(a.x, 7.0);     // 통로를 지나 S 자까지
  a.ctrl->deactivate();
  b.ctrl->deactivate();
  a.ctrl->cleanup();
  b.ctrl->cleanup();
}

TEST(DwaControllerXPlugin, ReadsResearchParametersAndLogsResets)
{
  const std::string fn = ::testing::TempDir() + "dwa_x_plugin_resets.csv";
  std::remove(fn.c_str());
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dwa_x_research_test");
  node->declare_parameter("controller_frequency", 20.0);
  declareDeployed(node, "DWA");
  declareDeployed(node, "DWAR");
  declareDeployed(node, "DWAL");
  node->declare_parameter("DWAR.research.reset_persist_cycles", 4);
  node->declare_parameter("DWAR.research.jerk_consistent", true);
  node->declare_parameter("DWAR.research.pp_candidate", true);
  node->declare_parameter("DWAL.research.log_resets", fn);   // 기록만 켠 팔 (명령은 원본과 같다)
  // 2 라운드 스위치 (F1·F2·F3) 를 켠 팔
  declareDeployed(node, "DWAF");
  node->declare_parameter("DWAF.research.reset_persist_cycles", 6);
  node->declare_parameter("DWAF.research.reset_model", true);
  node->declare_parameter("DWAF.research.reset_model_theta", 0.15);
  node->declare_parameter("DWAF.research.goal_extension_dist", 1.0);
  node->declare_parameter("DWAF.research.blocked_detour_shift", 0.6);
  auto cm = makeCostmap("dwa_x_research_costmap");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  Loop a(0.4);
  Loop r(0.4);
  Loop l(0.4);
  Loop lf(0.4);
  a.ctrl = loader.createSharedInstance("amr_navigation::DWAController");
  r.ctrl = loader.createSharedInstance("amr_nav_research::DwaControllerX");
  l.ctrl = loader.createSharedInstance("amr_nav_research::DwaControllerX");
  lf.ctrl = loader.createSharedInstance("amr_nav_research::DwaControllerX");
  a.ctrl->configure(node, "DWA", tf, cm);
  r.ctrl->configure(node, "DWAR", tf, cm);
  l.ctrl->configure(node, "DWAL", tf, cm);
  lf.ctrl->configure(node, "DWAF", tf, cm);
  for (auto * p : {&a, &r, &l, &lf}) {
    p->ctrl->activate();
    p->ctrl->setPlan(plan());
  }
  int differ = 0;
  int differ_f = 0;
  for (int k = 0; k < 120; ++k) {
    const auto ca = a.step();
    const auto cr = r.step();
    const auto cl = l.step();
    const auto cf = lf.step();
    differ += !sameBits(ca.linear.x, cr.linear.x) || !sameBits(ca.angular.z, cr.angular.z);
    differ_f += !sameBits(ca.linear.x, cf.linear.x) || !sameBits(ca.angular.z, cf.angular.z);
    ASSERT_TRUE(sameBits(ca.linear.x, cl.linear.x)) << "기록만 켠 팔이 원본과 다르다 " << k;
    ASSERT_TRUE(sameBits(ca.angular.z, cl.angular.z)) << "cycle " << k;
  }
  EXPECT_GT(differ, 0);   // 연구 스위치가 실제로 연결돼 있다
  EXPECT_GT(differ_f, 0);   // 2 라운드 스위치도
  EXPECT_GT(lf.x, 5.0);      // 켠 팔도 정상 주행
  for (auto * p : {&a, &r, &l, &lf}) {
    p->ctrl->deactivate();
    p->ctrl->cleanup();   // 기록 파일을 닫는다
  }
  std::ifstream f(fn);
  ASSERT_TRUE(f.good());
  std::string header;
  std::getline(f, header);
  EXPECT_EQ(
    header,
    "t,v_last,v_meas,w_last,w_meas,gap_v,gap_w,persist_count,reason,d_goal,phase,plan_id,"
    "controller,v_hat,w_hat");
  // 느린 서보(τ 0.4 s)라 정지 출발 가속 중 명령–측정 차가 임계를 넘어 재설정이 난다
  int rows = 0;
  std::string line;
  while (std::getline(f, line)) {
    EXPECT_NE(line.find(",1,DWAL,nan,nan"), std::string::npos) << line;
    ++rows;
  }
  EXPECT_GT(rows, 0);
  std::remove(fn.c_str());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new RosEnv);
  return RUN_ALL_TESTS();
}
