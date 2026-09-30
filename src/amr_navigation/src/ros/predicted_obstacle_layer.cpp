#include "amr_navigation/predicted_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace amr_navigation
{

void PredictedObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"PredictedObstacleLayer: 노드가 없다"};
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("perception/predicted_obstacles")));
  declareParameter("cost", rclcpp::ParameterValue(229.0));
  declareParameter("radius", rclcpp::ParameterValue(0.30));
  declareParameter("timeout", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".cost", cost_);
  node->get_parameter(name_ + ".radius", radius_);
  node->get_parameter(name_ + ".timeout", timeout_);
  // 253(INSCRIBED) 이상이면 "막힘" 이 되어 이 레이어의 취지(기피)가 사라진다.
  cost_ = std::clamp(cost_, 1.0, 252.0);
  stamp_ = node->now();
  sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      std::vector<Pt> pts;
      pts.reserve(msg->width * msg->height);
      sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y");
      for (; ix != ix.end(); ++ix, ++iy) {
        if (std::isfinite(*ix) && std::isfinite(*iy)) {
          pts.push_back({static_cast<double>(*ix), static_cast<double>(*iy)});
        }
      }
      std::lock_guard<std::mutex> lock(mutex_);
      pts_ = std::move(pts);
      stamp_ = msg->header.stamp;
      have_ = true;
    });
  current_ = true;
  RCLCPP_INFO(
    node->get_logger(),
    "%s: 예측 점유를 비용 %.0f (반경 %.2f m) 로 찍는다 <- %s",
    name_.c_str(), cost_, radius_, topic_.c_str());
}

void PredictedObstacleLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  // 지난 번 찍은 범위는 항상 다시 포함해야 예측이 사라졌을 때 비용이 지워진다.
  if (have_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_ || pts_.empty()) {
    return;
  }
  double lo_x = 1e9, lo_y = 1e9, hi_x = -1e9, hi_y = -1e9;
  for (const auto & p : pts_) {
    lo_x = std::min(lo_x, p.x - radius_);
    lo_y = std::min(lo_y, p.y - radius_);
    hi_x = std::max(hi_x, p.x + radius_);
    hi_y = std::max(hi_y, p.y + radius_);
  }
  *min_x = std::min(*min_x, lo_x);
  *min_y = std::min(*min_y, lo_y);
  *max_x = std::max(*max_x, hi_x);
  *max_y = std::max(*max_y, hi_y);
  last_min_x_ = lo_x;
  last_min_y_ = lo_y;
  last_max_x_ = hi_x;
  last_max_y_ = hi_y;
}

void PredictedObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  std::vector<Pt> pts;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto node = node_.lock();
    if (node && have_ && timeout_ > 0.0) {
      const double age = (node->now() - stamp_).seconds();
      if (age > timeout_ || age < -timeout_) {
        return;                       // 오래된 예측은 쓰지 않는다 (발행이 멈추면 만료)
      }
    }
    pts = pts_;
  }
  const unsigned char c = static_cast<unsigned char>(std::lround(cost_));
  const double res = master.getResolution();
  const int r_cells = std::max(0, static_cast<int>(std::ceil(radius_ / std::max(res, 1e-6))));
  for (const auto & p : pts) {
    unsigned int mx = 0, my = 0;
    if (!master.worldToMap(p.x, p.y, mx, my)) {
      continue;
    }
    const int ci = static_cast<int>(mx), cj = static_cast<int>(my);
    for (int dj = -r_cells; dj <= r_cells; ++dj) {
      for (int di = -r_cells; di <= r_cells; ++di) {
        if (di * di + dj * dj > r_cells * r_cells) {
          continue;
        }
        const int i = ci + di, j = cj + dj;
        if (i < min_i || i >= max_i || j < min_j || j >= max_j) {
          continue;
        }
        const unsigned char old = master.getCost(
          static_cast<unsigned int>(i),
          static_cast<unsigned int>(j));
        // 더 비싼 값(정적 장애물·팽창)은 덮지 않는다 — 기피는 올리기만 한다.
        if (old == nav2_costmap_2d::NO_INFORMATION || old < c) {
          master.setCost(static_cast<unsigned int>(i), static_cast<unsigned int>(j), c);
        }
      }
    }
  }
}

void PredictedObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pts_.clear();
  have_ = false;
}

}  // namespace amr_navigation

PLUGINLIB_EXPORT_CLASS(amr_navigation::PredictedObstacleLayer, nav2_costmap_2d::Layer)
