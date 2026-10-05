// velocity_profiler_node — cmd_vel_nav → 저크 제한 S-curve 프로파일 + 모델 추종 PI →
//   cmd_vel_smoothed (50 Hz).
// Nav2 velocity_smoother 대체 (components.md §3.3, §4.1). 알고리즘은 core::VelocityProfiler.
//
// Sub  cmd_vel_nav        geometry_msgs/Twist     reliable depth 1
//      odometry/filtered  nav_msgs/Odometry       PI 피드백 (측정 v, ω)
//      payload/mass       std_msgs/Float32        latched (transient_local), 가속·저크 한계 스케일
// Pub  cmd_vel_smoothed   geometry_msgs/Twist     rate [Hz] (기본 50)
//      velocity_profiler/state  std_msgs/Float64MultiArray
//        [v_target, w_target, v_ref, w_ref, a_ref, v_out, w_out, v_meas, w_meas, jerk_ref,
//        resynced, jerk_out, a_out, correction, resync_count, meas_fresh]
//        (jerk_out: 최종 명령 v_out 의 2차 차분 — 출력 성형으로 ≤ limits.max_linear_jerk,
//         재동기화 순간 제외)
// 측정 신선도: odometry/filtered 헤더 시각이 직전 주기와 같으면 PI 를 갱신하지 않는다
//   (반복 표본이 kp 로 새는 것 방지).
// 첫 명령: 유휴(발행 중단) 상태에서 cmd_vel_nav 가 오면 다음 타이머를 기다리지 않고 바로 한 주기를
//   돌린다.
// 파라미터: config/robot_params.yaml 의 limits.*, robot.base_mass 와 config/velocity_profiler.yaml.
#ifndef AMR_NAVIGATION__VELOCITY_PROFILER_NODE_HPP_
#define AMR_NAVIGATION__VELOCITY_PROFILER_NODE_HPP_

#include <mutex>

#include "amr_navigation/core/velocity_profiler.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace amr_navigation
{

class VelocityProfilerNode : public rclcpp::Node
{
public:
  explicit VelocityProfilerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  const core::VelocityProfiler & profiler() const {return profiler_;}

private:
  void onCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onPayload(const std_msgs::msg::Float32::SharedPtr msg);
  void onTimer();
  void tickLocked();

  std::mutex mutex_;
  core::VelocityProfiler profiler_;
  double period_{0.02};
  double cmd_timeout_{0.5};
  double odom_timeout_{0.2};
  double idle_publish_time_{1.0};
  double robot_mass_{47.6};

  double v_target_{0.0};
  double w_target_{0.0};
  rclcpp::Time last_cmd_time_;
  bool has_cmd_{false};
  double v_meas_{0.0};
  double w_meas_{0.0};
  rclcpp::Time last_odom_time_;
  bool has_odom_{false};
  double odom_stamp_{-1.0};        // 최신 odometry 헤더 시각 [s]
  double used_odom_stamp_{-2.0};   // 직전 PI 갱신에 쓴 헤더 시각
  rclcpp::Time last_tick_;
  bool has_tick_{false};
  double idle_time_{0.0};
  double last_a_ref_{0.0};
  double last_v_out_{0.0};   // 최종 명령 저크 진단 (2차 차분)
  double last_a_out_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr payload_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__VELOCITY_PROFILER_NODE_HPP_
