// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// cloud_preprocessor_node rclcpp 통합 시험: 실제 토픽으로 lazy 구독
// (출력 구독자가 생길 때만 입력 구독)과
// 비유한 점 제거 결과(is_dense, frame_id, ring 필드)를 본다.

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "amr_lidar3d/cloud_preprocessor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

using namespace std::chrono_literals;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

PointCloud2 rawCloud()
{
  PointCloud2 c;
  c.header.frame_id = "amr_01/lidar3d_link";
  auto fld = [](const char * n, uint32_t off, uint8_t t) {
      PointField f;
      f.name = n;
      f.offset = off;
      f.datatype = t;
      f.count = 1;
      return f;
    };
  c.fields = {fld("x", 0, PointField::FLOAT32), fld("y", 4, PointField::FLOAT32),
    fld("z", 8, PointField::FLOAT32), fld("intensity", 16, PointField::FLOAT32),
    fld("ring", 24, PointField::UINT16)};
  c.point_step = 32;
  c.width = 4;
  c.height = 2;
  c.row_step = 128;
  c.is_dense = false;
  c.data.assign(256, 0);
  const float inf = std::numeric_limits<float>::infinity();
  const float xs[8] = {1.0F, inf, 2.0F, 3.0F, inf, inf, 4.0F, -inf};
  for (int i = 0; i < 8; ++i) {
    std::memcpy(c.data.data() + i * 32, &xs[i], 4);
    const uint16_t ring = static_cast<uint16_t>(i / 4);
    std::memcpy(c.data.data() + i * 32 + 24, &ring, 2);
  }
  return c;
}

class PreprocessorRos : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void spinFor(std::chrono::milliseconds d, const std::function<bool()> & done = nullptr)
  {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {
      exec_.spin_some(20ms);
      if (done && done()) {
        return;
      }
    }
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
};

TEST_F(PreprocessorRos, LazySubscribesOnlyWithOutputSubscriber)
{
  const std::string ns = "/t_lazy";
  rclcpp::NodeOptions opts;
  opts.arguments(
    {"--ros-args", "-r", "__ns:=" + ns, "-p", "lazy:=true", "-p", "lazy_check_period:=0.05",
      "-p", "report_period:=0.0"});
  auto node = amr_lidar3d::createCloudPreprocessor(opts);
  auto peer = std::make_shared<rclcpp::Node>("peer", ns);
  auto raw_pub = peer->create_publisher<PointCloud2>("lidar3d/points_raw", rclcpp::QoS(5));
  exec_.add_node(node);
  exec_.add_node(peer);

  spinFor(400ms);
  EXPECT_EQ(raw_pub->get_subscription_count(), 0U) << "구독자 없는데 입력을 구독했다";

  std::vector<PointCloud2> got;
  auto out_sub = peer->create_subscription<PointCloud2>(
    "lidar3d/points", rclcpp::QoS(5),
    [&got](PointCloud2::ConstSharedPtr m) {got.push_back(*m);});
  spinFor(3000ms, [&]() {return raw_pub->get_subscription_count() > 0U;});
  ASSERT_GT(raw_pub->get_subscription_count(), 0U) << "output subscribed, input not";

  for (int i = 0; i < 20 && got.empty(); ++i) {
    raw_pub->publish(rawCloud());
    spinFor(100ms, [&]() {return !got.empty();});
  }
  ASSERT_FALSE(got.empty());
  const PointCloud2 & out = got.front();
  EXPECT_TRUE(out.is_dense);
  EXPECT_EQ(out.height, 1U);
  EXPECT_EQ(out.width, 4U);
  EXPECT_EQ(out.header.frame_id, "amr_01/lidar3d_link");
  ASSERT_EQ(out.fields.size(), 5U);
  EXPECT_EQ(out.fields[4].name, "ring");
  uint16_t ring_last = 0;
  std::memcpy(&ring_last, out.data.data() + 3 * 32 + 24, 2);
  EXPECT_EQ(ring_last, 1U);

  out_sub.reset();
  spinFor(3000ms, [&]() {return raw_pub->get_subscription_count() == 0U;});
  EXPECT_EQ(raw_pub->get_subscription_count(), 0U) << "output gone, input kept";
}

TEST_F(PreprocessorRos, NonLazyFrameOverride)
{
  const std::string ns = "/t_eager";
  rclcpp::NodeOptions opts;
  opts.arguments(
    {"--ros-args", "-r", "__ns:=" + ns,   // lazy 기본 false: 기동 즉시 입력 구독
      "-p", "frame_id:=fixed_frame", "-p", "report_period:=0.0"});
  auto node = amr_lidar3d::createCloudPreprocessor(opts);
  auto peer = std::make_shared<rclcpp::Node>("peer2", ns);
  auto raw_pub = peer->create_publisher<PointCloud2>("lidar3d/points_raw", rclcpp::QoS(5));
  std::vector<PointCloud2> got;
  auto out_sub = peer->create_subscription<PointCloud2>(
    "lidar3d/points", rclcpp::SensorDataQoS(),         // best-effort 구독도 reliable 발행에 붙는다
    [&got](PointCloud2::ConstSharedPtr m) {got.push_back(*m);});
  exec_.add_node(node);
  exec_.add_node(peer);
  spinFor(3000ms, [&]() {return raw_pub->get_subscription_count() > 0U;});
  EXPECT_GT(raw_pub->get_subscription_count(), 0U);
  for (int i = 0; i < 20 && got.empty(); ++i) {
    raw_pub->publish(rawCloud());
    spinFor(100ms, [&]() {return !got.empty();});
  }
  ASSERT_FALSE(got.empty());
  EXPECT_EQ(got.front().header.frame_id, "fixed_frame");
  EXPECT_EQ(got.front().width, 4U);
}

}  // namespace
