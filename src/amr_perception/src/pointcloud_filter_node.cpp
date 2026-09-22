// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// pointcloud_filter_node — 깊이 이미지 → 점군 (노이즈 가산 + 거리 컷 + voxel)
// (components.md §3.4 / §5.4).
//
// Sub  camera/depth/image_raw (sensor_msgs/Image, 32FC1 또는 16UC1, sensor QoS)
//      camera/depth/camera_info (sensor_msgs/CameraInfo) — 최신 내부 파라미터 K 를 보관
// Pub  camera/depth/points_filtered (sensor_msgs/PointCloud2, sensor QoS,
//      frame = 깊이 optical 프레임)
//
// 시뮬레이터 점군(camera/depth/points)은 Fortress 6.18 에서 optical 이 아닌 본체 규약 좌표로 나와
// 쓰지 않는다 (config/sensors.yaml 주석). 깊이 이미지 + K 로 직접 역투영한다.
// 파라미터 키 depth_camera.* 는 config/sensors.yaml 과 같은 이름이라 그 파일을 그대로 넘길 수 있다.

#include <chrono>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "amr_perception/depth_cloud.hpp"
#include "amr_perception/nodes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace amr_perception
{

class PointcloudFilterNode : public rclcpp::Node
{
public:
  explicit PointcloudFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pointcloud_filter_node", options)
  {
    params_.range_min = declare_parameter<double>("depth_camera.range_min", 0.20);
    params_.noise_quadratic_coeff =
      declare_parameter<double>("depth_camera.noise_quadratic_coeff", 0.002);
    params_.max_range = declare_parameter<double>("max_range", 5.0);
    params_.leaf_size = declare_parameter<double>("leaf_size", 0.05);
    params_.add_noise = declare_parameter<bool>("add_noise", true);
    params_.pixel_step = declare_parameter<int>("pixel_step", 1);
    params_.min_points_per_voxel = declare_parameter<int>("min_points_per_voxel", 1);
    const int seed = declare_parameter<int>("seed", 0);
    rng_.seed(seed > 0 ? static_cast<uint32_t>(seed) : std::random_device{}());

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("topics.output", "camera/depth/points_filtered"),
      rclcpp::SensorDataQoS());
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      declare_parameter<std::string>("topics.camera_info", "camera/depth/camera_info"),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        intrinsics_.fx = msg->k[0];
        intrinsics_.fy = msg->k[4];
        intrinsics_.cx = msg->k[2];
        intrinsics_.cy = msg->k[5];
      });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("topics.depth", "camera/depth/image_raw"),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {onDepth(*msg);});
    RCLCPP_INFO(
      get_logger(), "깊이 점군: 거리 %.2f~%.2f m, voxel %.3f m, 노이즈 k=%.4f (%s)",
      params_.range_min, params_.max_range, params_.leaf_size, params_.noise_quadratic_coeff,
      params_.add_noise ? "가산" : "끔");
  }

private:
  void onDepth(const sensor_msgs::msg::Image & img)
  {
    if (!intrinsics_.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info 대기 중");
      return;
    }
    DepthEncoding enc;
    if (img.encoding == "32FC1") {
      enc = DepthEncoding::kFloat32Meters;
    } else if (img.encoding == "16UC1" || img.encoding == "mono16") {
      enc = DepthEncoding::kUint16Millimeters;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "지원하지 않는 깊이 인코딩 %s", img.encoding.c_str());
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const auto raw = depthImageToPoints(
      img.data.data(), static_cast<int>(img.width), static_cast<int>(img.height),
      static_cast<int>(img.step), enc, intrinsics_, params_, &rng_);
    const auto pts = voxelDownsample(raw, params_.leaf_size, params_.min_points_per_voxel);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = img.header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(pts.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    const char * names[3] = {"x", "y", "z"};
    for (uint32_t i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField f;
      f.name = names[i];
      f.offset = 4 * i;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      cloud.fields.push_back(f);
    }
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const float xyz[3] = {pts[i].x, pts[i].y, pts[i].z};
      std::memcpy(&cloud.data[i * 12], xyz, sizeof(xyz));
    }
    pub_->publish(cloud);
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_DEBUG(
      get_logger(), "깊이 %ux%u → %zu 점 → voxel %zu 점 (%.2f ms)", img.width, img.height,
      raw.size(), pts.size(), ms);
  }

  DepthCloudParams params_;
  PinholeIntrinsics intrinsics_;
  std::mt19937 rng_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

rclcpp::Node::SharedPtr createPointcloudFilterNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<PointcloudFilterNode>(options);
}

}  // namespace amr_perception
