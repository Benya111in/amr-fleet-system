// wheel_odometry_node: joint_states → 인코더 모델(4096 틱 + 슬립 잡음) → 순기구학
// → wheel_odom (50 Hz).
//
// 인터페이스 (docs/architecture/components.md §5.2)
//   Sub  joint_states      sensor_msgs/JointState  바퀴 조인트 누적 각
//                          (+ 속도: 감긴 입력의 2π 분기 힌트)
//   Pub  wheel_odom        nav_msgs/Odometry       frame <prefix>odom, child <prefix>base_footprint
//   Pub  wheel_odom/ticks  sensor_msgs/JointState  (디버그) position = 누적 틱, velocity = 틱/s
//   Srv  wheel_odom/reset  std_srvs/Trigger        자세·공분산 0 으로 (드리프트 실험 시작점)
// TF 는 기본 발행하지 않는다: odom → base_footprint 는 ekf_filter_node_odom 담당
// (publish_tf 는 디버그용).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "amr_localization/diff_drive_kinematics.hpp"
#include "amr_localization/wheel_odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace amr_localization
{

namespace
{
/// 2D 자세(θ) → 쿼터니언 z, w.
void yawToQuaternion(double yaw, geometry_msgs::msg::Quaternion & q)
{
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
}

std::string prefixed(const std::string & prefix, const std::string & frame)
{
  if (prefix.empty() || frame.rfind(prefix, 0) == 0) {
    return frame;
  }
  return prefix + frame;
}
}  // namespace

class WheelOdometryNode : public rclcpp::Node
{
public:
  explicit WheelOdometryNode(const rclcpp::NodeOptions & options)
  : Node("wheel_odometry_node", options)
  {
    WheelOdometryParams p;
    const double radius = declare_parameter("wheel_radius", 0.0825);
    const double left_radius = declare_parameter("left_wheel_radius", 0.0);
    const double right_radius = declare_parameter("right_wheel_radius", 0.0);
    p.geometry.left_wheel_radius = left_radius > 0.0 ? left_radius : radius;
    p.geometry.right_wheel_radius = right_radius > 0.0 ? right_radius : radius;
    // 유효 바퀴 간격 = 기하 b × 보정 배율 (드리프트 실험의 회전 배율 Θ_odom/Θ_gt 로 식별,
    // diff_drive_controller 의 wheel_separation_multiplier 와 같은 의미)
    p.geometry.wheel_separation = declare_parameter("wheel_separation", 0.36) *
      declare_parameter("wheel_separation_multiplier", 1.0);
    p.encoder.ticks_per_revolution =
      static_cast<int>(declare_parameter<std::int64_t>("ticks_per_revolution", 4096));
    p.encoder.slip_noise_stddev = declare_parameter("slip_noise_stddev", 0.01);
    p.encoder.quantize = declare_parameter("enable_quantization", true);
    p.encoder.slip_noise = declare_parameter("enable_slip_noise", true);
    p.noise.slip_noise_stddev = p.encoder.slip_noise ? p.encoder.slip_noise_stddev : 0.0;
    // 거리당 슬립 기준 거리 ℓ_ref (σ_s 가 정의된 굴림 거리).
    // 인코더 φ_ref = ℓ_ref / r 는 코어가 계산한다.
    p.noise.slip_reference_distance = declare_parameter("slip_reference_distance", 0.01);
    p.encoder.wrapped_input = declare_parameter("joint_position_wrapped", false);
    p.encoder.max_wheel_speed = declare_parameter("max_wheel_speed", 30.0);
    p.noise.slip_distance_coeff = declare_parameter("slip_distance_coeff", 0.0);
    p.wheel_param_rel_stddev = declare_parameter("wheel_param_rel_stddev", 0.0);
    p.lateral_stddev = declare_parameter("lateral_velocity_stddev", 0.02);
    p.lateral_skid_coeff = declare_parameter("lateral_skid_coeff", 0.0);
    const double rate = declare_parameter("publish_rate", 50.0);
    p.publish_period = rate > 0.0 ? 1.0 / rate : 0.0;
    p.integration = parseIntegrationMethod(
      declare_parameter<std::string>("integration", "exact_arc"));
    const auto seed = declare_parameter<std::int64_t>("noise_seed", 0);
    p.seed = seed > 0 ? static_cast<std::uint64_t>(seed) : std::random_device{}();
    unfused_variance_ = declare_parameter("unfused_variance", 1e-3);

    left_joint_ = declare_parameter<std::string>("left_wheel_joint", "left_wheel_joint");
    right_joint_ = declare_parameter<std::string>("right_wheel_joint", "right_wheel_joint");
    const std::string prefix = declare_parameter<std::string>("frame_prefix", "");
    // xacro 는 조인트 이름에도 prefix 를 붙인다 (amr_01/left_wheel_joint) → 둘 다 받는다
    left_joint_prefixed_ = prefixed(prefix, left_joint_);
    right_joint_prefixed_ = prefixed(prefix, right_joint_);
    odom_frame_ = prefixed(prefix, declare_parameter<std::string>("odom_frame", "odom"));
    base_frame_ = prefixed(prefix, declare_parameter<std::string>("base_frame", "base_footprint"));
    publish_tf_ = declare_parameter("publish_tf", false);
    const bool publish_ticks = declare_parameter("publish_ticks", true);

    odometry_ = std::make_unique<WheelOdometry>(p);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", rclcpp::QoS(10));
    if (publish_ticks) {
      ticks_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "wheel_odom/ticks", rclcpp::QoS(10));
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    // 위치 기반이라 드롭에 강하다 → best-effort 로 받아 신뢰성 발행자/최선 발행자 모두와 호환
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {onJointState(*msg);});
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "wheel_odom/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        odometry_->resetPose();
        res->success = true;
        res->message = "wheel odometry pose reset to origin";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    RCLCPP_INFO(
      get_logger(),
      "wheel odometry: r_L=%.4f r_R=%.4f b=%.4f N=%d sigma_s=%.4f rate=%.1f Hz "
      "integration=%s joints=[%s, %s] frames=%s->%s",
      p.geometry.left_wheel_radius, p.geometry.right_wheel_radius, p.geometry.wheel_separation,
      p.encoder.ticks_per_revolution, p.encoder.slip_noise_stddev, rate,
      toString(p.integration).c_str(), left_joint_.c_str(), right_joint_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  bool findJoints(const sensor_msgs::msg::JointState & msg)
  {
    if (left_index_ >= 0 && right_index_ >= 0 &&
      static_cast<std::size_t>(std::max(left_index_, right_index_)) < msg.name.size() &&
      isLeft(msg.name[left_index_]) && isRight(msg.name[right_index_]))
    {
      return true;
    }
    left_index_ = right_index_ = -1;
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (isLeft(msg.name[i])) {
        left_index_ = static_cast<int>(i);
      } else if (isRight(msg.name[i])) {
        right_index_ = static_cast<int>(i);
      }
    }
    return left_index_ >= 0 && right_index_ >= 0;
  }

  bool isLeft(const std::string & name) const
  {
    return name == left_joint_ || name == left_joint_prefixed_;
  }

  bool isRight(const std::string & name) const
  {
    return name == right_joint_ || name == right_joint_prefixed_;
  }

  void onJointState(const sensor_msgs::msg::JointState & msg)
  {
    if (!findJoints(msg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "joint_states has no '%s'/'%s'",
        left_joint_.c_str(), right_joint_.c_str());
      return;
    }
    if (msg.position.size() != msg.name.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "joint_states without positions");
      return;
    }
    rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      stamp = now();
    }
    // 조인트 속도는 선택 (Gazebo JointStatePublisher 는 채운다): 감긴 입력의 2π 분기 힌트
    const bool has_velocity = msg.velocity.size() == msg.name.size();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    WheelOdometryOutput out;
    if (!odometry_->update(
        stamp.seconds(), msg.position[left_index_], msg.position[right_index_], out,
        has_velocity ? msg.velocity[left_index_] : nan,
        has_velocity ? msg.velocity[right_index_] : nan))
    {
      return;
    }
    if (out.ambiguous_steps > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "wheel angle branch ambiguous or impossible jump (%d samples, %d total): "
        "twist variance inflated by one wheel revolution",
        out.ambiguous_steps, static_cast<int>(odometry_->ambiguousSteps()));
    }
    publish(stamp, out);
  }

  void publish(const rclcpp::Time & stamp, const WheelOdometryOutput & out)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = out.pose.x;
    odom.pose.pose.position.y = out.pose.y;
    yawToQuaternion(normalizeAngle(out.pose.theta), odom.pose.pose.orientation);

    // 6x6 행 우선 (x, y, z, roll, pitch, yaw). 평면 밖 성분은 추정하지 않음 → unfused_variance.
    auto & pc = odom.pose.covariance;
    pc.fill(0.0);
    const std::array<int, 3> idx = {0, 1, 5};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        pc[idx[r] * 6 + idx[c]] = out.pose_covariance(r, c);
      }
    }
    pc[2 * 6 + 2] = pc[3 * 6 + 3] = pc[4 * 6 + 4] = unfused_variance_;

    odom.twist.twist.linear.x = out.twist.v;
    odom.twist.twist.angular.z = out.twist.w;
    auto & tc = odom.twist.covariance;
    tc.fill(0.0);
    tc[0] = out.twist_covariance.var_v;
    tc[7] = out.twist_covariance.var_vy;
    tc[35] = out.twist_covariance.var_w;
    tc[5] = tc[30] = out.twist_covariance.cov_vw;
    tc[14] = tc[21] = tc[28] = unfused_variance_;
    odom_pub_->publish(odom);

    if (ticks_pub_) {
      sensor_msgs::msg::JointState ticks;
      ticks.header.stamp = stamp;
      ticks.name = {left_joint_, right_joint_};
      ticks.position = {
        static_cast<double>(out.ticks_left), static_cast<double>(out.ticks_right)};
      ticks.velocity = {out.tick_rate_left, out.tick_rate_right};
      ticks_pub_->publish(ticks);
    }
    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = out.pose.x;
      tf.transform.translation.y = out.pose.y;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  std::unique_ptr<WheelOdometry> odometry_;
  std::string left_joint_;
  std::string right_joint_;
  std::string left_joint_prefixed_;
  std::string right_joint_prefixed_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{false};
  double unfused_variance_{1e-3};
  int left_index_{-1};
  int right_index_{-1};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ticks_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace amr_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_localization::WheelOdometryNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
