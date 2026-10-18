// amr_nav_research::TimedController — 계산 시간 측정용 nav2_core::Controller 감싸개
// (docs/research/paper-research/prereg_r3c.md §3 E4).
//
// 안쪽 제어기 플러그인을 pluginlib 으로 불러 **같은 이름**으로 configure 한다. 그래서 안쪽 제어기는
// 감싸지 않았을 때와 똑같이 <name>.* 파라미터를 읽고 같은 토픽을 쓴다. 이 감싸개는 두 파라미터만
// 더 읽는다:
//   <name>.timed_inner_plugin (string, 필수): 안쪽 플러그인 형
//     (예 "nav2_mppi_controller::MPPIController")
//   <name>.timed_csv (string, "" = 기록 안 함): 주기 기록 CSV 경로
// computeVelocityCommands 한 번의 벽시계 시간(std::chrono::steady_clock)을 주기마다 메모리에 쌓고,
// cleanup() 또는 소멸자(먼저 오는 쪽)에서 CSV 로 한 번 쓴다. 측정 경계는 안쪽 호출 바로 앞뒤다.
// 다른 호출(configure·activate·deactivate·cleanup·setPlan·setSpeedLimit)은 그대로 넘긴다.
// CSV 열: t_ros (노드 시계 [s], 호출 직전), dt_us (안쪽 호출 벽시계 [µs]), plan_id (setPlan 순번),
//   ok (1 = 정상 반환, 0 = 예외 — 예외는 그대로 다시 던진다).
#ifndef AMR_NAV_RESEARCH__TIMED_CONTROLLER_HPP_
#define AMR_NAV_RESEARCH__TIMED_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace amr_nav_research
{

class TimedController : public nav2_core::Controller
{
public:
  TimedController();
  ~TimedController() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  struct Sample
  {
    double t_ros;
    std::int64_t dt_ns;
    int plan_id;
    int ok;
  };
  void writeCsv();

  // 선언 순서가 중요하다: inner_ 가 loader_ 보다 먼저 소멸해야 한다 (멤버는 역순으로 소멸).
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  nav2_core::Controller::Ptr inner_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("TimedController")};
  std::string name_;
  std::string inner_type_;
  std::string csv_path_;
  std::vector<Sample> samples_;
  int plan_id_{0};
  bool written_{false};
};

}  // namespace amr_nav_research

#endif  // AMR_NAV_RESEARCH__TIMED_CONTROLLER_HPP_
