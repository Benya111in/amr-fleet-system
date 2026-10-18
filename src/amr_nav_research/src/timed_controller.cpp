#include "amr_nav_research/timed_controller.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_nav_research
{

TimedController::TimedController() = default;

TimedController::~TimedController()
{
  writeCsv();
  inner_.reset();
}

void TimedController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("TimedController: 부모 노드가 없다");
  }
  name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".timed_inner_plugin", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".timed_csv", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name + ".timed_inner_plugin", inner_type_);
  node->get_parameter(name + ".timed_csv", csv_path_);
  if (inner_type_.empty()) {
    throw std::runtime_error("TimedController: " + name + ".timed_inner_plugin 이 비었다");
  }
  loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
    "nav2_core", "nav2_core::Controller");
  inner_ = loader_->createSharedInstance(inner_type_);
  samples_.clear();
  samples_.reserve(1U << 20);
  written_ = false;
  RCLCPP_INFO(
    logger_, "TimedController %s: 안쪽 %s, 기록 %s", name.c_str(), inner_type_.c_str(),
    csv_path_.empty() ? "(끔)" : csv_path_.c_str());
  inner_->configure(parent, name, tf, costmap_ros);
}

void TimedController::cleanup()
{
  if (inner_) {
    inner_->cleanup();
  }
  writeCsv();
}

void TimedController::activate()
{
  inner_->activate();
}

void TimedController::deactivate()
{
  inner_->deactivate();
}

void TimedController::setPlan(const nav_msgs::msg::Path & path)
{
  ++plan_id_;
  inner_->setPlan(path);
}

geometry_msgs::msg::TwistStamped TimedController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const double t_ros = clock_->now().seconds();
  const auto t0 = std::chrono::steady_clock::now();
  try {
    geometry_msgs::msg::TwistStamped out =
      inner_->computeVelocityCommands(pose, velocity, goal_checker);
    const auto t1 = std::chrono::steady_clock::now();
    samples_.push_back(
      {t_ros, std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count(), plan_id_, 1});
    return out;
  } catch (...) {
    const auto t1 = std::chrono::steady_clock::now();
    samples_.push_back(
      {t_ros, std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count(), plan_id_, 0});
    throw;
  }
}

void TimedController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  inner_->setSpeedLimit(speed_limit, percentage);
}

void TimedController::writeCsv()
{
  if (written_ || csv_path_.empty()) {
    return;
  }
  written_ = true;
  FILE * f = std::fopen(csv_path_.c_str(), "w");
  if (f == nullptr) {
    RCLCPP_WARN(logger_, "TimedController: %s 를 열 수 없다", csv_path_.c_str());
    return;
  }
  std::fprintf(f, "# controller=%s inner=%s\n", name_.c_str(), inner_type_.c_str());
  std::fprintf(f, "t_ros,dt_us,plan_id,ok\n");
  for (const auto & s : samples_) {
    std::fprintf(
      f, "%.6f,%.3f,%d,%d\n", s.t_ros, 1e-3 * static_cast<double>(s.dt_ns), s.plan_id, s.ok);
  }
  std::fclose(f);
}

}  // namespace amr_nav_research

PLUGINLIB_EXPORT_CLASS(amr_nav_research::TimedController, nav2_core::Controller)
