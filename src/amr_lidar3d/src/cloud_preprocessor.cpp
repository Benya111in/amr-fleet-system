// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// cloud_preprocessor_node — 3D LiDAR 점군 전처리 (rclcpp 컴포넌트 amr_lidar3d::CloudPreprocessor).
//
// Sub  lidar3d/points_raw (sensor_msgs/PointCloud2) — ros_gz_bridge 출력, Gazebo gpu_lidar 원본
// Pub  lidar3d/points     (sensor_msgs/PointCloud2) — 비유한 점 제거, height 1, is_dense true,
//                          필드 배치(x·y·z·intensity·ring, point_step) 그대로
// 계약 (토픽·프레임·QoS·필드): docs/research/slam3d/sensor.md. 순수 로직: cloud_filter.cpp.
//
// 입력은 기동 즉시 구독한다 (lazy 기본 false). lazy:=true 면 출력 구독자가 있을 때만 입력을
// 구독해 브리지·렌더까지 끈다 — 하지만 Fortress 센서의 다음 갱신 시각은 한 주기씩만 늘어나
// (nextUpdateTime += period) 시뮬레이션 시각 T 에 켜진 센서는 T × 10 장을 물리 스텝마다(1 kHz)
// 몰아서 렌더하며 따라잡는다. 실측(s1_check_a): 켠 뒤 3.7 s(sim) 동안 98 장, 간격 중앙값 1 ms,
// 그동안 RTF 저하. 그래서 3D 로봇은 일시 정지 중(스폰 때)부터 구독해 sim 0 에서 센서를 켠다.
// lazy 는 기동 직후(sim ≈ 0)에 소비자가 붙는 것이 보장될 때만 쓴다.
// Humble 에는 발행자 matched 이벤트가 없어 구독자 수를 주기적으로 본다 (lazy_check_period, 벽시계).
// 컴포넌트로 SLAM 노드와 같은 컨테이너에 올리면 intra-process 로 복사 없이 넘긴다
// (use_intra_process_comms).

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "amr_lidar3d/cloud_filter.hpp"
#include "amr_lidar3d/cloud_preprocessor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace amr_lidar3d
{

class CloudPreprocessor : public rclcpp::Node
{
public:
  explicit CloudPreprocessor(const rclcpp::NodeOptions & options)
  : Node("lidar3d_preprocessor", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "lidar3d/points_raw");
    const auto output_topic = declare_parameter<std::string>("output_topic", "lidar3d/points");
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    lazy_ = declare_parameter<bool>("lazy", false);
    input_reliable_ = declare_parameter<bool>("input_reliable", false);
    const bool output_reliable = declare_parameter<bool>("output_reliable", true);
    const int depth = static_cast<int>(declare_parameter<int>("qos_depth", 5));
    const double check_period = declare_parameter<double>("lazy_check_period", 0.5);
    const double report_period = declare_parameter<double>("report_period", 30.0);
    depth_ = depth > 0 ? static_cast<std::size_t>(depth) : 5U;

    // 출력은 reliable 이 기본: reliable 발행자에는 reliable·best-effort 구독자가 모두 붙는다
    // (RTAB-Map 기본 구독은 reliable — best-effort 발행자에는 붙지 못한다)
    rclcpp::QoS out_qos{rclcpp::KeepLast(depth_)};
    if (output_reliable) {
      out_qos.reliable();
    } else {
      out_qos.best_effort();
    }
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, out_qos);

    if (lazy_) {
      lazy_timer_ = create_wall_timer(
        std::chrono::duration<double>(check_period > 0.0 ? check_period : 0.5),
        [this]() {updateSubscription();});
    } else {
      subscribe();
    }
    if (report_period > 0.0) {
      report_timer_ = create_wall_timer(
        std::chrono::duration<double>(report_period), [this]() {report();});
    }
    RCLCPP_INFO(
      get_logger(), "3D 점군 전처리: %s → %s (frame_id %s, lazy %s, 출력 %s)",
      get_node_topics_interface()->resolve_topic_name(input_topic_).c_str(),
      pub_->get_topic_name(), frame_id_.empty() ? "유지" : frame_id_.c_str(),
      lazy_ ? "켬" : "끔", output_reliable ? "reliable" : "best_effort");
  }

private:
  void subscribe()
  {
    rclcpp::QoS in_qos = input_reliable_ ?
      rclcpp::QoS(rclcpp::KeepLast(depth_)).reliable() : rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, in_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(*msg);});
  }

  void updateSubscription()
  {
    const std::size_t subs = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
    if (subs > 0U && !sub_) {
      subscribe();
      RCLCPP_INFO(get_logger(), "출력 구독자 %zu → 입력 구독 시작", subs);
    } else if (subs == 0U && sub_) {
      sub_.reset();
      RCLCPP_INFO(get_logger(), "출력 구독자 0 → 입력 구독 해제 (lazy)");
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2 & in)
  {
    const auto t0 = std::chrono::steady_clock::now();
    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    FilterStats stats;
    const FilterError err = filterNonFinite(in, frame_id_, *out, stats);
    if (err != FilterError::kOk) {
      ++rejected_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "점군을 거를 수 없다: %s (frame %s, %u x %u)",
        toString(err), in.header.frame_id.c_str(), in.width, in.height);
      return;
    }
    pub_->publish(std::move(out));
    ++frames_;
    points_in_ += stats.input_points;
    points_kept_ += stats.kept_points;
    busy_s_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }

  void report()
  {
    if (frames_ == 0U && rejected_ == 0U) {
      return;
    }
    const double n = static_cast<double>(frames_ > 0U ? frames_ : 1U);
    RCLCPP_INFO(
      get_logger(), "최근 창: %zu 스캔 (거부 %zu), 스캔당 입력 %.0f → 유지 %.0f 점, 처리 %.2f ms",
      frames_, rejected_, static_cast<double>(points_in_) / n,
      static_cast<double>(points_kept_) / n, 1e3 * busy_s_ / n);
    frames_ = rejected_ = points_in_ = points_kept_ = 0U;
    busy_s_ = 0.0;
  }

  std::string input_topic_;
  std::string frame_id_;
  bool lazy_{false};
  bool input_reliable_{false};
  std::size_t depth_{5};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr lazy_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  std::size_t frames_{0};
  std::size_t rejected_{0};
  std::size_t points_in_{0};
  std::size_t points_kept_{0};
  double busy_s_{0.0};
};

std::shared_ptr<rclcpp::Node> createCloudPreprocessor(const rclcpp::NodeOptions & options)
{
  return std::make_shared<CloudPreprocessor>(options);
}

}  // namespace amr_lidar3d

RCLCPP_COMPONENTS_REGISTER_NODE(amr_lidar3d::CloudPreprocessor)
