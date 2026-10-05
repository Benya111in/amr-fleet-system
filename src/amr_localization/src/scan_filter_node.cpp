// scan_filter_node: scan → 거리/각도 필터 + 아웃라이어·섀도우 제거 → scan_filtered (10 Hz).
//
// 인터페이스 (docs/architecture/components.md §5.2)
//   Sub  scan           sensor_msgs/LaserScan  Gazebo gpu_lidar 720 빔 (0.5°), σ = 0.03 m
//   Pub  scan_filtered  sensor_msgs/LaserScan  빔 수·각도 메타데이터 동일,
//                                              제거 빔은 NaN/±Inf (REP-117)
// 구독자: amcl, slam_toolbox, costmap(planner/controller), obstacle_tracker_node, safety_node.

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "amr_localization/scan_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace amr_localization
{

class ScanFilterNode : public rclcpp::Node
{
public:
  explicit ScanFilterNode(const rclcpp::NodeOptions & options)
  : Node("scan_filter_node", options)
  {
    ScanFilterParams p;
    p.range_min = declare_parameter("range_min", 0.10);
    p.range_max = declare_parameter("range_max", 25.0);
    p.angle_window_min = declare_parameter("angle_window_min", -M_PI);
    p.angle_window_max = declare_parameter("angle_window_max", M_PI);
    const auto mask = declare_parameter<std::vector<double>>("angle_mask", std::vector<double>{});
    if (mask.size() % 2 != 0) {
      throw std::invalid_argument("angle_mask must hold [start, end] pairs");
    }
    for (std::size_t i = 0; i + 1 < mask.size(); i += 2) {
      p.angle_mask.emplace_back(mask[i], mask[i + 1]);
    }
    p.outlier_window = static_cast<int>(declare_parameter<std::int64_t>("outlier_window", 2));
    p.outlier_thresh = declare_parameter("outlier_thresh", 0.15);
    p.outlier_range_gain = declare_parameter("outlier_range_gain", 3.0);
    p.outlier_min_neighbors =
      static_cast<int>(declare_parameter<std::int64_t>("outlier_min_neighbors", 1));
    p.shadow_filter_enabled = declare_parameter("shadow_filter_enabled", true);
    p.shadow_min_angle = declare_parameter("shadow_min_angle_deg", 10.0) * M_PI / 180.0;
    p.shadow_range_noise_stddev = declare_parameter("shadow_range_noise_stddev", 0.03);
    p.shadow_noise_factor = declare_parameter("shadow_noise_factor", 3.0);
    stats_period_ = declare_parameter("stats_log_period", 30.0);
    filter_ = std::make_unique<ScanFilter>(p);

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", rclcpp::QoS(10));
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {onScan(*msg);});
    RCLCPP_INFO(
      get_logger(),
      "scan filter: range [%.2f, %.2f] m, window [%.3f, %.3f] rad, %zu masks, outlier w=%d "
      "thr=%.2f+%.1f*r*dA, shadow %s (%.1f deg, |dr| > %.1f*sqrt2*%.3f m)",
      p.range_min, p.range_max, p.angle_window_min, p.angle_window_max, p.angle_mask.size(),
      p.outlier_window, p.outlier_thresh, p.outlier_range_gain,
      p.shadow_filter_enabled ? "on" : "off", p.shadow_min_angle * 180.0 / M_PI,
      p.shadow_noise_factor, p.shadow_range_noise_stddev);
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan & msg)
  {
    sensor_msgs::msg::LaserScan out;
    out.header = msg.header;
    out.angle_min = msg.angle_min;
    out.angle_max = msg.angle_max;
    out.angle_increment = msg.angle_increment;
    out.time_increment = msg.time_increment;
    out.scan_time = msg.scan_time;
    out.range_min = static_cast<float>(filter_->effectiveRangeMin(msg.range_min));
    out.range_max = static_cast<float>(filter_->effectiveRangeMax(msg.range_max));
    const ScanFilterStats s = filter_->apply(
      msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max, out.ranges);
    // 세기는 제거된 빔 위치를 0 으로 맞춰 같은 길이로 넘긴다
    if (msg.intensities.size() == msg.ranges.size()) {
      out.intensities = msg.intensities;
      for (std::size_t i = 0; i < out.ranges.size(); ++i) {
        if (std::isnan(out.ranges[i])) {
          out.intensities[i] = 0.0f;
        }
      }
    }
    pub_->publish(out);

    total_.input += s.input;
    total_.too_close += s.too_close;
    total_.too_far += s.too_far;
    total_.angle_removed += s.angle_removed;
    total_.outliers += s.outliers;
    total_.shadows += s.shadows;
    total_.valid_output += s.valid_output;
    ++scans_;
    const rclcpp::Time now_time = now();
    if (last_stats_.nanoseconds() == 0) {
      last_stats_ = now_time;
    }
    if (stats_period_ > 0.0 && (now_time - last_stats_).seconds() >= stats_period_) {
      const double n = static_cast<double>(scans_);
      RCLCPP_INFO(
        get_logger(),
        "scan filter per scan (%zu scans): in %.1f, valid out %.1f, too_close %.1f, "
        "no_return %.1f, angle %.1f, outliers %.2f, shadows %.2f",
        scans_, total_.input / n, total_.valid_output / n, total_.too_close / n,
        total_.too_far / n, total_.angle_removed / n, total_.outliers / n, total_.shadows / n);
      total_ = ScanFilterStats();
      scans_ = 0;
      last_stats_ = now_time;
    }
  }

  std::unique_ptr<ScanFilter> filter_;
  double stats_period_{30.0};
  ScanFilterStats total_;
  std::size_t scans_{0};
  rclcpp::Time last_stats_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

}  // namespace amr_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_localization::ScanFilterNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
