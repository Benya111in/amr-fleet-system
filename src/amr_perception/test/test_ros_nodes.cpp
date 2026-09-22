// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// C++ 노드 rclcpp 통합 테스트: 실제 토픽·QoS·TF·서비스로 노드를 구동해 출력을 확인한다.
//  - safety_node: 기동 전에 발행된 latched(transient_local) E-stop 수신, false 만으로는 해제 안 됨,
//    safety/reset_estop 으로 해제, WARNING 존 0.5 m/s 상한, 0.3 m 이내 정지 + estop_active
//  - obstacle_tracker_node: /map(transient_local) 배경 제거 + 이동 원 추적 (속도·동적·map 프레임)
//  - pointcloud_filter_node: 깊이 이미지 + camera_info → PointCloud2 역투영

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "amr_msgs/msg/tracked_obstacle_array.hpp"
#include "amr_perception/nodes.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scan_sim.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace
{

class RosNodesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

/// pred 가 참이 될 때까지 (또는 timeout) 스핀. tick 은 period 마다 호출.
bool spinUntil(
  rclcpp::Executor & exec, const std::function<bool()> & pred, double timeout,
  const std::function<void()> & tick = {}, double period = 0.02)
{
  const auto start = std::chrono::steady_clock::now();
  auto last_tick = start - std::chrono::seconds(1);
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
    timeout)
  {
    const auto now = std::chrono::steady_clock::now();
    if (tick && std::chrono::duration<double>(now - last_tick).count() >= period) {
      tick();
      last_tick = now;
    }
    exec.spin_some(5ms);
    if (pred && pred()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return pred ? pred() : true;
}

geometry_msgs::msg::TransformStamped makeTf(
  const std::string & parent, const std::string & child, double x, double y, double z,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = parent;
  tf.header.stamp = stamp;
  tf.child_frame_id = child;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = z;
  tf.transform.rotation.w = 1.0;
  return tf;
}

sensor_msgs::msg::LaserScan toMsg(
  const amr_perception::LaserScanData & d, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::LaserScan s;
  s.header.stamp = stamp;
  s.header.frame_id = "lidar_link";
  s.angle_min = static_cast<float>(d.angle_min);
  s.angle_increment = static_cast<float>(d.angle_increment);
  s.angle_max = static_cast<float>(d.angle_min + d.angle_increment * (d.ranges.size() - 1));
  s.range_min = static_cast<float>(d.range_min);
  s.range_max = static_cast<float>(d.range_max);
  s.ranges = d.ranges;
  return s;
}

}  // namespace

TEST_F(RosNodesTest, SafetyNodeGatesCommandsAndLatchesEstop)
{
  auto helper = std::make_shared<rclcpp::Node>("safety_test_helper");
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  auto estop_pub = helper->create_publisher<std_msgs::msg::Bool>("estop", latched);
  std_msgs::msg::Bool on;
  on.data = true;
  estop_pub->publish(on);  // 노드 기동 전 latched true → 늦게 뜬 safety_node 도 받아야 한다
  auto cmd_pub = helper->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 10);
  auto scan_pub = helper->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_filtered", rclcpp::SensorDataQoS());
  tf2_ros::StaticTransformBroadcaster stb(helper);
  stb.sendTransform(makeTf("base_link", "lidar_link", 0.15, 0.0, 0.20, helper->now()));

  double last_cmd = -1.0;
  int cmd_count = 0;
  std::string zone;
  int level = -1;
  bool estop_active = false;
  bool diag = false;
  auto s1 = helper->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [&](geometry_msgs::msg::Twist::ConstSharedPtr m) {
      last_cmd = m->linear.x;
      ++cmd_count;
    });
  auto s2 = helper->create_subscription<std_msgs::msg::String>(
    "safety/zone_name", latched,
    [&](std_msgs::msg::String::ConstSharedPtr m) {zone = m->data;});
  auto s3 = helper->create_subscription<std_msgs::msg::UInt8>(
    "safety/zone", latched,
    [&](std_msgs::msg::UInt8::ConstSharedPtr m) {level = m->data;});
  auto s4 = helper->create_subscription<std_msgs::msg::Bool>(
    "safety/estop_active", latched,
    [&](std_msgs::msg::Bool::ConstSharedPtr m) {estop_active = m->data;});
  auto s5 = helper->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", 10, [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr) {diag = true;});
  auto reset = helper->create_client<std_srvs::srv::Trigger>("safety/reset_estop");

  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
  {
    rclcpp::Parameter("safety.sensor_timeouts.imu", 0.0),
    rclcpp::Parameter("safety.sensor_timeouts.wheel_encoder", 0.0),
    rclcpp::Parameter("safety.sensor_timeouts.rgb_camera", 0.0),
    rclcpp::Parameter("safety.sensor_timeouts.depth_camera", 0.0),
  });
  auto node = amr_perception::createSafetyNode(opts);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);

  double v_cmd = 0.5;
  double front = -1.0;  // 전면 모서리에서 벽까지 [m], < 0 이면 장애물 없음
  int ticks = 0;
  auto tick = [&]() {
      geometry_msgs::msg::Twist t;
      t.linear.x = v_cmd;
      cmd_pub->publish(t);
      if (ticks++ % 5 == 0) {
        amr_perception::LaserScanData d;
        d.angle_min = -M_PI;
        d.angle_increment = 2.0 * M_PI / 720.0;
        d.range_min = 0.1;
        d.range_max = 25.0;
        d.ranges.assign(720, std::numeric_limits<float>::infinity());
        if (front >= 0.0) {
          const double x_wall = 0.30 + front - 0.15;  // LiDAR 는 base_link x = 0.15
          for (int i = 0; i < 720; ++i) {
            const double a = d.angle_min + i * d.angle_increment;
            if (std::cos(a) > 1e-3) {
              const double r = x_wall / std::cos(a);
              if (std::abs(r * std::sin(a)) <= 0.5) {
                d.ranges[i] = static_cast<float>(r);
              }
            }
          }
        }
        scan_pub->publish(toMsg(d, helper->now()));
      }
    };

  // 1) 기동 전 latched E-stop → 정지
  ASSERT_TRUE(spinUntil(exec, [&]() {return estop_active && cmd_count > 10;}, 8.0, tick));
  EXPECT_DOUBLE_EQ(last_cmd, 0.0);
  // 2) false 만으로는 해제되지 않는다
  std_msgs::msg::Bool off;
  off.data = false;
  estop_pub->publish(off);
  spinUntil(exec, {}, 0.5, tick);
  EXPECT_DOUBLE_EQ(last_cmd, 0.0);
  EXPECT_TRUE(estop_active);
  // 3) reset 서비스 → 해제, 명령 통과
  ASSERT_TRUE(reset->wait_for_service(3s));
  auto fut = reset->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(
    spinUntil(
      exec, [&]() {return fut.wait_for(0s) == std::future_status::ready;}, 5.0, tick));
  EXPECT_TRUE(fut.get()->success);
  ASSERT_TRUE(spinUntil(exec, [&]() {return last_cmd > 0.49 && !estop_active;}, 3.0, tick));
  EXPECT_EQ(zone, "CLEAR");
  // 4) WARNING 존: 1.0 m/s → 0.5 m/s
  v_cmd = 1.0;
  front = 0.8;
  ASSERT_TRUE(
    spinUntil(
      exec, [&]() {return zone == "WARNING" && level == 1 && cmd_count > 0;}, 3.0, tick));
  spinUntil(exec, {}, 0.3, tick);
  EXPECT_NEAR(last_cmd, 0.5, 1e-9);
  // 5) 0.3 m 이내 → 정지 + estop_active
  front = 0.25;
  ASSERT_TRUE(
    spinUntil(
      exec, [&]() {return zone == "STOP" && estop_active && last_cmd == 0.0;}, 3.0, tick));
  EXPECT_EQ(level, 3);
  EXPECT_TRUE(diag);
}

TEST_F(RosNodesTest, ObstacleTrackerNodeTracksMovingObstacleAndRemovesMapWall)
{
  auto helper = std::make_shared<rclcpp::Node>("tracker_test_helper");
  tf2_ros::StaticTransformBroadcaster stb(helper);
  tf2_ros::TransformBroadcaster tfb(helper);
  stb.sendTransform(
    std::vector<geometry_msgs::msg::TransformStamped>{
    makeTf("base_link", "lidar_link", 0.15, 0.0, 0.20, helper->now()),
    makeTf("map", "odom", 0.0, 0.0, 0.0, helper->now())});
  // /map: 10 × 10 m, 0.05 m, x = 3.0 m 에 세로 벽 (정적 배경)
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  auto map_pub = helper->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched);
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.05F;
  grid.info.width = 200;
  grid.info.height = 200;
  grid.info.origin.position.x = -5.0;
  grid.info.origin.position.y = -5.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(200 * 200, 0);
  for (int y = 0; y < 200; ++y) {
    grid.data[y * 200 + 160] = 100;  // x ∈ [3.00, 3.05)
  }
  map_pub->publish(grid);
  auto scan_pub = helper->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_filtered", rclcpp::SensorDataQoS());
  auto odom_pub = helper->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered_map", 10);
  auto plan_pub = helper->create_publisher<nav_msgs::msg::Path>("plan", 10);

  amr_msgs::msg::TrackedObstacleArray last;
  int received = 0;
  int markers = 0;
  auto s1 = helper->create_subscription<amr_msgs::msg::TrackedObstacleArray>(
    "perception/tracked_obstacles", 10,
    [&](amr_msgs::msg::TrackedObstacleArray::ConstSharedPtr m) {
      last = *m;
      ++received;
    });
  auto s2 = helper->create_subscription<visualization_msgs::msg::MarkerArray>(
    "perception/tracked_markers", 10,
    [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr) {++markers;});

  auto node = amr_perception::createObstacleTrackerNode(rclcpp::NodeOptions());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);
  spinUntil(exec, {}, 1.0);  // 구독·지도 수신 대기

  std::mt19937 rng(3);
  const auto t0 = std::chrono::steady_clock::now();
  double t_last = 0.0;
  auto tick = [&]() {
      const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      t_last = t;
      const rclcpp::Time stamp = helper->now();
      tfb.sendTransform(makeTf("odom", "base_link", 0.0, 0.0, 0.18, stamp));
      nav_msgs::msg::Odometry od;
      od.header.stamp = stamp;
      od.header.frame_id = "map";
      odom_pub->publish(od);
      nav_msgs::msg::Path path;
      path.header.stamp = stamp;
      path.header.frame_id = "map";
      for (int i = 0; i <= 10; ++i) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = 0.5 * i;
        ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
      }
      plan_pub->publish(path);
      const auto d = scan_sim::makeScan(
        amr_perception::Pose2D{0.15, 0.0, 0.0},
        {{amr_perception::Vec2(2.0, -1.5 + 1.0 * t), 0.2}},
        {{amr_perception::Vec2(3.02, -3.0), amr_perception::Vec2(3.02, 3.0)}}, 0.02, &rng);
      scan_pub->publish(toMsg(d, stamp));
    };
  spinUntil(exec, {}, 3.0, tick, 0.1);

  ASSERT_GT(received, 10);
  EXPECT_GT(markers, 10);
  EXPECT_EQ(last.header.frame_id, "map");
  const double truth_y = -1.5 + 1.0 * t_last;
  int moving = 0;
  for (const auto & o : last.obstacles) {
    EXPECT_LT(o.position.x, 2.7) << "지도 벽이 배경으로 제거되지 않았다";
    if (std::hypot(o.position.x - 2.0, o.position.y - truth_y) < 0.4) {
      ++moving;
      EXPECT_NEAR(std::hypot(o.velocity.x, o.velocity.y), 1.0, 0.25);
      EXPECT_TRUE(o.is_dynamic);
      EXPECT_GT(o.confidence, 0.5F);
      EXPECT_FALSE(std::isnan(o.time_to_collision));
    }
  }
  EXPECT_EQ(moving, 1);
}

TEST_F(RosNodesTest, PointcloudFilterNodeBackProjectsDepth)
{
  auto helper = std::make_shared<rclcpp::Node>("cloud_test_helper");
  auto info_pub = helper->create_publisher<sensor_msgs::msg::CameraInfo>(
    "camera/depth/camera_info", rclcpp::SensorDataQoS());
  auto img_pub = helper->create_publisher<sensor_msgs::msg::Image>(
    "camera/depth/image_raw", rclcpp::SensorDataQoS());
  sensor_msgs::msg::PointCloud2 cloud;
  int received = 0;
  auto sub = helper->create_subscription<sensor_msgs::msg::PointCloud2>(
    "camera/depth/points_filtered", rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
      cloud = *m;
      ++received;
    });
  rclcpp::NodeOptions opts;
  opts.parameter_overrides(
  {
    rclcpp::Parameter("add_noise", false),
    rclcpp::Parameter("leaf_size", 0.0),
    rclcpp::Parameter("pixel_step", 1),
  });
  auto node = amr_perception::createPointcloudFilterNode(opts);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);

  sensor_msgs::msg::CameraInfo info;
  info.width = 64;
  info.height = 48;
  info.k = {50.0, 0.0, 31.5, 0.0, 50.0, 23.5, 0.0, 0.0, 1.0};
  sensor_msgs::msg::Image img;
  img.header.frame_id = "camera_depth_optical_frame";
  img.width = 64;
  img.height = 48;
  img.encoding = "32FC1";
  img.step = 64 * 4;
  img.data.resize(img.step * img.height);
  const float z = 2.0F;
  for (std::size_t i = 0; i < img.data.size(); i += 4) {
    std::memcpy(&img.data[i], &z, 4);
  }
  auto tick = [&]() {
      info_pub->publish(info);
      img.header.stamp = helper->now();
      img_pub->publish(img);
    };
  ASSERT_TRUE(spinUntil(exec, [&]() {return received > 0;}, 5.0, tick, 0.05));
  EXPECT_EQ(cloud.header.frame_id, "camera_depth_optical_frame");
  EXPECT_EQ(cloud.width, 64U * 48U);
  EXPECT_EQ(cloud.point_step, 12U);
  float first[3];
  std::memcpy(first, cloud.data.data(), sizeof(first));
  EXPECT_NEAR(first[0], (0.0 - 31.5) * 2.0 / 50.0, 1e-5);
  EXPECT_NEAR(first[2], 2.0, 1e-6);
}
