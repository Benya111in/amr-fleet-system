// 배포 nav2_params.yaml 로 띄운 실제 Costmap2DROS (지역·전역) 가 센서 데이터를 치명 셀로 찍고
// 지우는가. 리뷰: 소스별 max_obstacle_height 기본값 0.0 때문에 LiDAR 점(odom z = 0.20)이 모두
// 버려져 두 코스트맵이 비어 있었는데, 플러그인 시험은 inflation_layer 만 올린 코스트맵에 값을 직접
// 써서 이를 잡지 못했다. 여기서는 설정 파일을 그대로 --params-file 로 넣고
// (템플릿 '<robot_ns>' → '', '<prefix>' → ''), 센서 extrinsic 은
// config/robot_params.yaml + config/sensors.yaml 에서 읽어 TF 를 만든다:
//   1) lidar_link(지면 +0.20) 스캔의 1.0 m 앞 벽 → 두 코스트맵 치명
//   2) 깊이 점군: LiDAR 평면 아래 0.10 m 상자 → 두 코스트맵 치명, 바닥 점(z ≈ 0)은 치명 아님
//   3) 벽이 사라진 스캔, 상자 자리 너머 바닥 점군 → 영속 0 으로 다음 갱신에 지워진다
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#ifndef NAV2_PARAMS_FILE
#error "NAV2_PARAMS_FILE 정의 필요 (CMakeLists)"
#endif
#ifndef REPO_CONFIG_DIR
#error "REPO_CONFIG_DIR 정의 필요 (CMakeLists)"
#endif

namespace
{
std::vector<std::shared_ptr<nav2_costmap_2d::Costmap2DROS>> g_keep;
double g_lidar_x = 0.0;
double g_lidar_z = 0.0;
double g_cam_x = 0.0;
double g_cam_z = 0.0;

std::string replaceAll(std::string s, const std::string & from, const std::string & to)
{
  for (std::size_t p = s.find(from); p != std::string::npos; p = s.find(from, p + to.size())) {
    s.replace(p, from.size(), to);
  }
  return s;
}

// 배포 파일을 단일 로봇·네임스페이스 없음·실시간 시계로 치환한 임시 파일
std::string templatedParams()
{
  // NAV2_PARAMS_OVERRIDE: 다른 파일로 같은 시험 (예: 리뷰 전 설정이 이 시험에 걸리는지 확인)
  const char * over = std::getenv("NAV2_PARAMS_OVERRIDE");
  std::ifstream in(over != nullptr ? over : NAV2_PARAMS_FILE);
  std::stringstream ss;
  ss << in.rdbuf();
  std::string text = replaceAll(ss.str(), "<robot_ns>", "");
  text = replaceAll(text, "<prefix>", "");
  text = replaceAll(text, "use_sim_time: True", "use_sim_time: False");
  const std::string path = "/tmp/amr_nav_test_costmap_params_" + std::to_string(::getpid()) +
    ".yaml";
  std::ofstream(path) << text;
  return path;
}

void loadExtrinsics()
{
  const YAML::Node robot = YAML::LoadFile(std::string(REPO_CONFIG_DIR) + "/robot_params.yaml");
  const YAML::Node sens = YAML::LoadFile(std::string(REPO_CONFIG_DIR) + "/sensors.yaml");
  const double base = robot["/**"]["ros__parameters"]["robot"]["base_link_height"].as<double>();
  const YAML::Node s = sens["/**"]["ros__parameters"];
  g_lidar_x = s["lidar"]["extrinsic"]["x"].as<double>();
  g_lidar_z = base + s["lidar"]["extrinsic"]["z"].as<double>();
  g_cam_x = s["camera_link"]["extrinsic"]["x"].as<double>();
  g_cam_z = base + s["camera_link"]["extrinsic"]["z"].as<double>();
}

geometry_msgs::msg::TransformStamped tfMsg(
  const std::string & parent, const std::string & child, double x, double z, double roll = 0.0,
  double pitch = 0.0, double yaw = 0.0)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  return t;
}

// lidar_link 스캔: x = wall_x (lidar_link 기준) 평면 벽, |y| ≤ 1 m 안만 반사 (없으면 전부 inf)
sensor_msgs::msg::LaserScan wallScan(double wall_x, rclcpp::Time stamp)
{
  sensor_msgs::msg::LaserScan s;
  s.header.frame_id = "lidar_link";
  s.header.stamp = stamp;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / 720.0;
  s.angle_max = M_PI - s.angle_increment;
  s.range_min = 0.1;
  s.range_max = 25.0;
  s.scan_time = 0.1;
  s.ranges.assign(720, std::numeric_limits<float>::infinity());
  for (int i = 0; i < 720; ++i) {
    const double a = s.angle_min + i * s.angle_increment;
    if (wall_x > 0.0 && std::cos(a) > 1e-3) {
      const double r = wall_x / std::cos(a);
      if (std::abs(r * std::sin(a)) <= 1.0) {
        s.ranges[static_cast<std::size_t>(i)] = static_cast<float>(r);
      }
    }
  }
  return s;
}

// base_footprint 좌표 점들 → 깊이 광학 프레임 점군 (x 우, y 하, z 전방; 원점 = 카메라)
sensor_msgs::msg::PointCloud2 cloud(
  const std::vector<std::array<double, 3>> & pts_base, rclcpp::Time stamp)
{
  sensor_msgs::msg::PointCloud2 pc;
  pc.header.frame_id = "camera_depth_optical_frame";
  pc.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier mod(pc);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts_base.size());
  sensor_msgs::PointCloud2Iterator<float> x(pc, "x");
  sensor_msgs::PointCloud2Iterator<float> y(pc, "y");
  sensor_msgs::PointCloud2Iterator<float> z(pc, "z");
  for (const auto & p : pts_base) {
    *x = static_cast<float>(-p[1]);
    *y = static_cast<float>(-(p[2] - g_cam_z));
    *z = static_cast<float>(p[0] - g_cam_x);
    ++x;
    ++y;
    ++z;
  }
  return pc;
}

// 0.10 m 높이 상자 (x ∈ [1.3, 1.5], y ∈ [0.6, 0.8]) 와 바닥 점 격자 (x ∈ [0.5, 3.0], |y| ≤ 1.2)
std::vector<std::array<double, 3>> depthScene(bool with_box)
{
  std::vector<std::array<double, 3>> pts;
  for (double x = 0.5; x <= 3.0; x += 0.05) {
    for (double y = -1.2; y <= 1.2; y += 0.05) {
      const bool in_box = x >= 1.3 && x <= 1.5 && y >= 0.6 && y <= 0.8;
      if (with_box && in_box) {
        pts.push_back({x, y, 0.10});
      } else if (with_box && x > 1.5 && y >= 0.55 && y <= 0.85 && x < 2.4) {
        continue;   // 상자 뒤 가림 (카메라 높이 0.25 → 바닥 가림 길이 ≈ 0.9 m)
      } else {
        pts.push_back({x, y, 0.004 * std::sin(17.0 * x + 5.0 * y)});   // 바닥 잡음 ±4 mm
      }
    }
  }
  return pts;
}

class ShippedCostmaps : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("costmap_marking_test");
    tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    // map = odom = base_footprint (정지 로봇), 센서 extrinsic 은 설정 파일 값
    tf_->sendTransform(
      {
        tfMsg("map", "odom", 0.0, 0.0), tfMsg("odom", "base_footprint", 0.0, 0.0),
        tfMsg("base_footprint", "lidar_link", g_lidar_x, g_lidar_z),
        tfMsg(
          "base_footprint", "camera_depth_optical_frame", g_cam_x, g_cam_z, -M_PI / 2, 0.0,
          -M_PI / 2),
      });
    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched);
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = 0.05;
    map.info.width = 200;
    map.info.height = 200;
    map.info.origin.position.x = -5.0;
    map.info.origin.position.y = -5.0;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(200 * 200, 0);
    map_pub_->publish(map);
    scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan_costmap",
      rclcpp::SensorDataQoS());
    // 같은 스캔을 전역용(동적 트랙 제외본, 트랙이 없으면 같은 내용)과 필터 전 토픽에도
    // (scan_filtered 를 직접 구독하던 이전 설정도 같은 조건으로 시험)
    raw_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan_filtered",
      rclcpp::SensorDataQoS());
    static_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan_costmap_static", rclcpp::SensorDataQoS());
    pc_static_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points_static", rclcpp::SensorDataQoS());
    pc_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points_filtered", rclcpp::SensorDataQoS());
    local_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", "/", "local_costmap");
    global_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", "/",
      "global_costmap");
    // 구성·활성화를 끝낸 뒤 실행기에 올린다 (구독 생성과 스핀이 겹치면 경합)
    for (auto & cm : {local_, global_}) {
      cm->on_configure(rclcpp_lifecycle::State());
      cm->on_activate(rclcpp_lifecycle::State());
    }
    exec_.add_node(node_);
    exec_.add_node(local_->get_node_base_interface());
    exec_.add_node(global_->get_node_base_interface());
    spin_ = std::thread([this]() {exec_.spin();});
  }

  void TearDown() override
  {
    exec_.cancel();
    spin_.join();
    exec_.remove_node(node_);
    exec_.remove_node(local_->get_node_base_interface());
    exec_.remove_node(global_->get_node_base_interface());
    for (auto & cm : {local_, global_}) {
      cm->on_deactivate(rclcpp_lifecycle::State());
      cm->on_cleanup(rclcpp_lifecycle::State());
    }
    // Humble(1.1.20) Costmap2DROS 소멸자가 cleanup 뒤에도 충돌한다
    // (rclcpp::shutdown 전후 모두, 실측) →
    // 프로세스 끝까지 들고 있다가 결과(XML 포함)를 쓴 뒤 _Exit 로 끝낸다
    g_keep.push_back(local_);
    g_keep.push_back(global_);
  }

  // 스캔 10 Hz + 점군 15 Hz 를 sec 동안 발행 (코스트맵 갱신 주기 여러 번)
  void feed(double wall_x, bool box, double sec)
  {
    const auto pts = depthScene(box);
    const auto t_end = std::chrono::steady_clock::now() + std::chrono::duration<double>(sec);
    int k = 0;
    while (std::chrono::steady_clock::now() < t_end) {
      const auto now = node_->now();
      if (k % 3 == 0) {
        scan_pub_->publish(wallScan(wall_x, now));
        raw_pub_->publish(wallScan(wall_x, now));
        static_pub_->publish(wallScan(wall_x, now));
      }
      if (k % 2 == 0) {
        pc_pub_->publish(cloud(pts, now));
        pc_static_pub_->publish(cloud(pts, now));
      }
      ++k;
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
  }

  // 중심이 (x0..x1, y0..y1) 안인 셀 중 치명 셀 수
  static int lethalIn(
    nav2_costmap_2d::Costmap2DROS & cm, double x0, double x1, double y0,
    double y1)
  {
    auto * c = cm.getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*c->getMutex());
    int n = 0;
    for (unsigned int my = 0; my < c->getSizeInCellsY(); ++my) {
      for (unsigned int mx = 0; mx < c->getSizeInCellsX(); ++mx) {
        double wx = 0.0;
        double wy = 0.0;
        c->mapToWorld(mx, my, wx, wy);
        n += wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1 &&
          c->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE;
      }
    }
    return n;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr static_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_static_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_;
  rclcpp::executors::MultiThreadedExecutor exec_;
  std::thread spin_;
};
}  // namespace

TEST_F(ShippedCostmaps, LidarPlaneAndLowObjectsAreMarkedAndCleared)
{
  ASSERT_NEAR(g_lidar_z, 0.20, 1e-9);   // 재배치 후 스캔 평면 (config/sensors.yaml)
  // 1) 벽: lidar_link 기준 1.0 m → base_footprint x = 1.0 + lidar_x (y ∈ [−0.5, 0.5] 만 확인)
  const double wall = 1.0 + g_lidar_x;
  feed(1.0, true, 2.5);
  // 스캔 게이트 중앙값 노드 없이 /scan_costmap 에 직접 넣는다 (노드 동작은 test_scan_denoise)
  for (auto * cm : {local_.get(), global_.get()}) {
    const int wall_cells = lethalIn(*cm, wall - 0.05, wall + 0.05, -0.5, 0.5);
    const int box_cells = lethalIn(*cm, 1.3, 1.5, 0.6, 0.8);
    const int floor_cells = lethalIn(*cm, 0.5, 1.0, -1.0, -0.2);   // 바닥 점만 있는 구역
    std::printf(
      "[ info ] %s: wall lethal %d, box lethal %d, floor lethal %d\n", cm->getName().c_str(),
      wall_cells, box_cells, floor_cells);
    EXPECT_GE(wall_cells, cm->getName() == "local_costmap" ? 30 : 15) << cm->getName();
    EXPECT_GE(box_cells, 2) << cm->getName();
    EXPECT_EQ(floor_cells, 0) << cm->getName();
  }
  // 2) 벽·상자 사라짐 → 다음 갱신들에서 지워진다 (observation_persistence 0).
  //    새 벽(3 m, |y| ≤ 1)으로 가는 빔이 지나는 |y| ≤ 0.3 구간만 본다
  //    (빔이 없는 곳은 지울 광선이 없다)
  feed(3.0, false, 2.0);
  for (auto * cm : {local_.get(), global_.get()}) {
    EXPECT_EQ(lethalIn(*cm, wall - 0.05, wall + 0.05, -0.3, 0.3), 0) << cm->getName();
    EXPECT_EQ(lethalIn(*cm, 1.3, 1.5, 0.6, 0.8), 0) << cm->getName();
    EXPECT_GE(lethalIn(*cm, 3.0 + g_lidar_x - 0.05, 3.0 + g_lidar_x + 0.05, -0.5, 0.5), 10);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  loadExtrinsics();
  const std::string params_file = templatedParams();
  std::vector<const char *> args = {argv[0], "--ros-args", "--params-file", params_file.c_str()};
  rclcpp::init(static_cast<int>(args.size()), args.data());
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::remove(params_file.c_str());
  std::fflush(nullptr);
  std::_Exit(rc);   // g_keep 의 Costmap2DROS 소멸자를 부르지 않는다 (위 TearDown 주석)
}
