#include "amr_navigation/ros_utils.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/inflation_layer.hpp"
#include "tf2/utils.h"

namespace amr_navigation
{

core::CostGrid costmapView(const nav2_costmap_2d::Costmap2D & costmap)
{
  core::CostGrid g;
  g.data = costmap.getCharMap();
  g.width = static_cast<int>(costmap.getSizeInCellsX());
  g.height = static_cast<int>(costmap.getSizeInCellsY());
  g.resolution = costmap.getResolution();
  g.origin_x = costmap.getOriginX();
  g.origin_y = costmap.getOriginY();
  return g;
}

double yawOf(const geometry_msgs::msg::Pose & pose)
{
  return tf2::getYaw(pose.orientation);
}

core::Pose2D toPose2D(const geometry_msgs::msg::Pose & pose)
{
  return {pose.position.x, pose.position.y, yawOf(pose)};
}

geometry_msgs::msg::Pose toPoseMsg(const core::Pose2D & pose)
{
  geometry_msgs::msg::Pose p;
  p.position.x = pose.x;
  p.position.y = pose.y;
  p.orientation.z = std::sin(0.5 * pose.theta);
  p.orientation.w = std::cos(0.5 * pose.theta);
  return p;
}

std::vector<core::Pose2D> toPoses2D(const nav_msgs::msg::Path & path)
{
  std::vector<core::Pose2D> out;
  out.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    out.push_back(toPose2D(ps.pose));
  }
  return out;
}

nav_msgs::msg::Path toPathMsg(
  const std::vector<core::Pose2D> & poses, const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(poses.size());
  for (const auto & p : poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = header;
    ps.pose = toPoseMsg(p);
    path.poses.push_back(ps);
  }
  return path;
}

std::vector<core::Point2D> footprintOf(nav2_costmap_2d::Costmap2DROS & costmap_ros)
{
  std::vector<core::Point2D> fp;
  if (costmap_ros.getUseRadius()) {
    const double r = costmap_ros.getLayeredCostmap()->getCircumscribedRadius();
    constexpr int kSides = 16;
    for (int i = 0; i < kSides; ++i) {
      const double a = 2.0 * core::kPi * i / kSides;
      fp.push_back({r * std::cos(a), r * std::sin(a)});
    }
    return fp;
  }
  for (const auto & p : costmap_ros.getRobotFootprint()) {
    fp.push_back({p.x, p.y});
  }
  return fp;
}

uint8_t circumscribedCost(nav2_costmap_2d::Costmap2DROS & costmap_ros)
{
  auto * layered = costmap_ros.getLayeredCostmap();
  const double r = layered->getCircumscribedRadius();
  const double res = layered->getCostmap()->getResolution();
  for (const auto & layer : *layered->getPlugins()) {
    auto inflation = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(layer);
    if (inflation) {
      return inflation->computeCost(r / res);
    }
  }
  return 0;
}

bool lookupTransform2D(
  const tf2_ros::Buffer & tf, const std::string & target, const std::string & source,
  double timeout_s, core::Pose2D & out)
{
  if (target == source) {
    out = {};
    return true;
  }
  try {
    const auto t = tf.lookupTransform(
      target, source, tf2::TimePointZero, tf2::durationFromSec(timeout_s));
    out.x = t.transform.translation.x;
    out.y = t.transform.translation.y;
    out.theta = tf2::getYaw(t.transform.rotation);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

}  // namespace amr_navigation
