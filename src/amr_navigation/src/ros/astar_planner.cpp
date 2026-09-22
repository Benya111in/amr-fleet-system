#include "amr_navigation/astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "amr_navigation/ros_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_navigation
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  costmap_ros_ = std::move(costmap_ros);
  global_frame_ = costmap_ros_->getGlobalFrameID();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("AStarPlanner: parent node expired");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  loadParameters();
  RCLCPP_INFO(
    logger_, "AStarPlanner '%s' configured: cost_weight %.2f, tolerance %.2f m, allow_unknown %d",
    name_.c_str(), astar_.config().cost_weight, tolerance_, astar_.config().allow_unknown);
}

void AStarPlanner::loadParameters()
{
  auto node = node_.lock();
  const std::string p = name_ + ".";
  core::AStarConfig a;
  a.cost_weight = param(node, p + "cost_weight", 2.0);
  a.allow_unknown = param(node, p + "allow_unknown", true);
  a.unknown_cost = static_cast<uint8_t>(
    std::clamp(param(node, p + "unknown_cost", 252), 0, static_cast<int>(core::kMaxNonObstacle)));
  a.allow_corner_cutting = param(node, p + "allow_corner_cutting", false);
  a.allow_start_in_inscribed = param(node, p + "allow_start_in_inscribed", true);
  a.max_iterations = static_cast<std::size_t>(std::max(0, param(node, p + "max_iterations", 0)));
  tolerance_ = param(node, p + "tolerance", 0.25);
  max_planning_time_ = param(node, p + "max_planning_time", 0.4);
  use_final_approach_orientation_ = param(node, p + "use_final_approach_orientation", false);

  const std::string s = p + "smoother.";
  core::SmootherConfig sc;
  sc.enable_shortcut = param(node, s + "enable_shortcut", true);
  sc.shortcut_max_length = param(node, s + "shortcut_max_length", 10.0);
  sc.shortcut_cost_ratio = param(node, s + "shortcut_cost_ratio", 0.05);
  sc.clearance_cost_margin = param(node, s + "clearance_cost_margin", 10);
  sc.output_spacing = param(node, s + "output_spacing", 0.05);
  sc.enable_smoothing = param(node, s + "enable_smoothing", true);
  sc.w_data = param(node, s + "w_data", 0.2);
  sc.w_smooth = param(node, s + "w_smooth", 0.3);
  sc.w_clearance = param(node, s + "w_clearance", 0.3);
  sc.clearance_cost_threshold = static_cast<uint8_t>(
    std::clamp(param(node, s + "clearance_cost_threshold", 100), 0, 252));
  sc.max_iterations = param(node, s + "max_iterations", 100);
  sc.tolerance = param(node, s + "tolerance", 1e-4);
  sc.allow_unknown = a.allow_unknown;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    astar_.setConfig(a);
    smoother_.setConfig(sc);
  }
  if (!param_handle_) {
    param_handle_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return onParameters(params);});
  }
}

rcl_interfaces::msg::SetParametersResult AStarPlanner::onParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  // 동적 재설정: 이 플러그인 접두어의 파라미터만 반영 (재빌드·재시작 없이 튜닝 — 명세 9장)
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::lock_guard<std::mutex> lock(mutex_);
  core::AStarConfig a = astar_.config();
  core::SmootherConfig sc = smoother_.config();
  const std::string p = name_ + ".";
  for (const auto & prm : params) {
    const std::string & n = prm.get_name();
    if (n.rfind(p, 0) != 0) {
      continue;
    }
    const std::string key = n.substr(p.size());
    if (key == "cost_weight") {
      a.cost_weight = prm.as_double();
    } else if (key == "allow_unknown") {
      a.allow_unknown = prm.as_bool();
      sc.allow_unknown = a.allow_unknown;
    } else if (key == "tolerance") {
      tolerance_ = prm.as_double();
    } else if (key == "max_planning_time") {
      max_planning_time_ = prm.as_double();
    } else if (key == "smoother.w_data") {
      sc.w_data = prm.as_double();
    } else if (key == "smoother.w_smooth") {
      sc.w_smooth = prm.as_double();
    } else if (key == "smoother.w_clearance") {
      sc.w_clearance = prm.as_double();
    } else if (key == "smoother.enable_smoothing") {
      sc.enable_smoothing = prm.as_bool();
    } else if (key == "smoother.enable_shortcut") {
      sc.enable_shortcut = prm.as_bool();
    }
  }
  astar_.setConfig(a);
  smoother_.setConfig(sc);
  return result;
}

void AStarPlanner::cleanup() {param_handle_.reset();}
void AStarPlanner::activate() {}
void AStarPlanner::deactivate() {}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  const auto t0 = std::chrono::steady_clock::now();
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "AStarPlanner: start/goal must be in the costmap frame '" + global_frame_ + "'");
  }
  std::lock_guard<std::mutex> plock(mutex_);
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> clock(*costmap->getMutex());
  const core::CostGrid grid = costmapView(*costmap);

  core::Cell s;
  core::Cell g;
  if (!grid.worldToMap(start.pose.position.x, start.pose.position.y, s.x, s.y)) {
    throw nav2_core::PlannerException("AStarPlanner: start is outside the costmap");
  }
  if (!grid.worldToMap(goal.pose.position.x, goal.pose.position.y, g.x, g.y)) {
    throw nav2_core::PlannerException("AStarPlanner: goal is outside the costmap");
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  const core::Point2D sp{start.pose.position.x, start.pose.position.y};
  const core::Point2D gp{goal.pose.position.x, goal.pose.position.y};
  const double goal_yaw = yawOf(goal.pose);
  if (s == g) {
    path.poses.push_back(start);
    path.poses.back().header = path.header;
    path.poses.push_back(goal);
    path.poses.back().header = path.header;
    return path;
  }

  const int tol_cells = static_cast<int>(std::ceil(tolerance_ / grid.resolution));
  const auto deadline = core::AStar::Clock::now() +
    std::chrono::microseconds(static_cast<int64_t>(std::max(0.0, max_planning_time_) * 1e6));
  const core::AStarResult r = astar_.plan(grid, s, g, tol_cells, deadline);
  if (!r.ok()) {
    throw nav2_core::PlannerException(
            "AStarPlanner: " + core::toString(r.status) + " (" + std::to_string(r.expansions) +
            " expansions)");
  }
  const auto t_search = std::chrono::steady_clock::now();

  // 허용 오차로 목표가 바뀌었으면 도달 셀 중심으로 끝낸다
  const bool exact_goal = r.reached_goal == g;
  core::Point2D end = gp;
  if (!exact_goal) {
    grid.mapToWorld(r.reached_goal.x, r.reached_goal.y, end.x, end.y);
  }
  core::SmootherStats st;
  const double * yaw_ptr = use_final_approach_orientation_ ? nullptr : &goal_yaw;
  std::vector<core::Pose2D> poses = smoother_.process(
    grid, r.path, astar_.traversalTable(), &sp, &end, yaw_ptr, &st);
  clock.unlock();

  path = toPathMsg(poses, path.header);
  const auto t1 = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(
    logger_,
    "AStar: %zu expansions, search %.1f ms, total %.1f ms, raw %.2f m -> %.2f m, %zu poses",
    r.expansions, std::chrono::duration<double, std::milli>(t_search - t0).count(),
    std::chrono::duration<double, std::milli>(t1 - t0).count(), st.raw_length, st.smoothed_length,
    path.poses.size());
  return path;
}

}  // namespace amr_navigation

PLUGINLIB_EXPORT_CLASS(amr_navigation::AStarPlanner, nav2_core::GlobalPlanner)
