#include "amr_navigation/predicted_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "amr_navigation/core/predicted_occupancy.hpp"
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
  // 로봇 자기 자리에는 치명을 찍지 않는다 — 찍으면 A* start 가 막혀 제어가 통째로 멈춘다
  // (연구 브리프 §11, core::mayStampLethal 주석). 외접원 0.361 + 여유 0.10.
  declareParameter("lethal_keepout", rclcpp::ParameterValue(0.46));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".cost", cost_);
  node->get_parameter(name_ + ".radius", radius_);
  node->get_parameter(name_ + ".timeout", timeout_);
  node->get_parameter(name_ + ".lethal_keepout", lethal_keepout_);
  // 253(INSCRIBED) 이상이면 계획기가 "막힘" 으로 본다. 실측이 둘 다 필요함을 보였다:
  //   229(기피): A* 가 그냥 통과해 재계획이 22 % -> 7 % 로 죽는다 (logs/CB1a·b)
  //   254(차단): 재계획은 22 % 로 살지만 우회가 커 이탈이 명세를 넘는다 (logs/RP3a·b)
  // 그래서 값을 열어 두고, 차단을 쓸 때는 창(predict_window)을 좁혀 우회를 제한한다.
  cost_ = std::clamp(cost_, 1.0, 254.0);
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
  double robot_x, double robot_y, double, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  // 자세는 updateCosts 에서 치명 금지 반경 판정에 쓴다. 레이어 API 가 여기서만 주므로
  // 받아 둔다 (같은 주기에 updateBounds -> updateCosts 순서로 불린다).
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  // 지난 번 찍은 범위를 **한 번** 다시 포함해야 예측이 사라졌을 때 비용이 지워진다.
  // 계속 포함하면 안 된다 — 1200x806 전역 지도에서 매 주기 큰 영역을 갱신하게 되어
  // 시뮬레이션 실시간 배율이 떨어진다 (실측 CB1a·b: 시행 sim 시간은 같은데 라운드
  // 벽시계가 530 -> 1420~1515 s). 포함한 뒤 비운다.
  if (has_last_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
    has_last_ = false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_ || pts_.empty()) {
    return;
  }
  std::vector<std::pair<double, double>> xy;
  xy.reserve(pts_.size());
  for (const auto & p : pts_) {
    xy.emplace_back(p.x, p.y);
  }
  double lo_x = 0.0, lo_y = 0.0, hi_x = 0.0, hi_y = 0.0;
  if (!core::pointsBounds(xy, radius_, &lo_x, &lo_y, &hi_x, &hi_y)) {
    return;
  }
  *min_x = std::min(*min_x, lo_x);
  *min_y = std::min(*min_y, lo_y);
  *max_x = std::max(*max_x, hi_x);
  *max_y = std::max(*max_y, hi_y);
  last_min_x_ = lo_x;
  last_min_y_ = lo_y;
  last_max_x_ = hi_x;
  last_max_y_ = hi_y;
  has_last_ = true;
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
      if (!core::predictionFresh(age, timeout_)) {
        return;                       // 오래된 예측은 쓰지 않는다 (발행이 멈추면 만료)
      }
    }
    pts = pts_;
  }
  const unsigned char c = static_cast<unsigned char>(std::lround(cost_));
  const double res = master.getResolution();
  const double ox = master.getOriginX(), oy = master.getOriginY();
  for (const auto & p : pts) {
    const auto cells = core::discCells(
      p.x, p.y, radius_, c, ox, oy, res,
      static_cast<int>(master.getSizeInCellsX()),
      static_cast<int>(master.getSizeInCellsY()));
    for (const auto & cell : cells) {
      if (cell.i < min_i || cell.i >= max_i || cell.j < min_j || cell.j >= max_j) {
        continue;
      }
      // 로봇 자기 자리에는 치명을 찍지 않는다 (§11). 기피로 강등해 계획기가 통과할 수
      // 있게 둔다 — 그 자리의 실제 안전은 안전 게이트가 맡는다.
      const double cx = ox + (cell.i + 0.5) * res, cy = oy + (cell.j + 0.5) * res;
      const unsigned char want = core::stampCost(
        cell.cost, cx, cy, robot_x_, robot_y_, lethal_keepout_,
        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, kDemotedCost);
      const auto ui = static_cast<unsigned int>(cell.i), uj = static_cast<unsigned int>(cell.j);
      // 더 비싼 값(정적 장애물·팽창)은 덮지 않는다 — 기피는 올리기만 한다.
      if (core::shouldRaise(master.getCost(ui, uj), want, nav2_costmap_2d::NO_INFORMATION)) {
        master.setCost(ui, uj, want);
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
