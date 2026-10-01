// 예측 동적 점유를 **중간 비용**으로 찍는 코스트맵 레이어 (명세 4장 7절 / 9장).
//
// 왜 ObstacleLayer 로는 안 되는가
// ------------------------------
// ObstacleLayer 는 관측 점을 항상 LETHAL(254) 로 찍는다. 그러면 A* 가 예측 점유를
// **"지나갈 수 없는 벽"** 으로 보고 통째로 돌아간다. 실측(logs/RP3a·b, 60 시행):
//   재계획한 시행 이탈 중앙 0.574 m vs 안 한 시행 0.162 m
//   명세 이탈 1.0 m 초과가 **재계획한 시행에서만** 3/13, 안 한 시행 0/47
//   대조군(재계획 끔, 60 시행)은 초과 0/60 — 순수한 회귀다.
// 그런데 그 자리는 벽이 아니라 **곧 비워질 자리**다. 보행자는 2~3 초 뒤 지나간다.
// 필요한 것은 "막기" 가 아니라 "기피" 이므로 중간 비용을 쓴다.
//
// 비용값
// ------
// 기본 229 는 nav2 의 점유→비용 변환 round(90·254/100) 과 같은 값이다 (보행 차선 마스크가
// 쓰던 것과 같은 수준). 253(INSCRIBED) 미만이므로 계획기가 **지나갈 수는 있고**, 다른
// 경로가 조금이라도 싸면 그쪽을 고른다. 254 면 막힌 것으로 본다.
//
// 앞서 실패한 보행 차선 밴드와 무엇이 다른가
// -----------------------------------------
// 그때는 **정적 차선 전체**에 상시 비용을 얹었고 자유측도가 5.7 % 인 좌표에 벌점만 더해
// 이탈·복귀·소요가 나빠졌다. 지금은 **계획 충돌 트랙의 만남 지점만 한시적으로** 얹는다
// (TTC 로 걸러 admit_ttc 안, 창 predict_window 안). 대상과 지속이 다르다.
#ifndef AMR_NAVIGATION__PREDICTED_OBSTACLE_LAYER_HPP_
#define AMR_NAVIGATION__PREDICTED_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"

namespace amr_navigation
{

class PredictedObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}   // 예측은 소거하지 않는다

private:
  struct Pt
  {
    double x, y;
  };

  /// 치명 금지 반경 안에서 대신 찍는 값. 253(INSCRIBED) 미만이어야 계획기가 통과한다.
  /// 229 는 nav2 의 점유->비용 변환 round(90*254/100) 과 같은 수준이다.
  static constexpr unsigned char kDemotedCost = 229;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::mutex mutex_;
  std::vector<Pt> pts_;            ///< 전역 프레임 점 (마지막 수신)
  rclcpp::Time stamp_;
  std::string topic_;
  double cost_{254.0};             ///< 찍을 비용. 253 미만 = 기피, 254 = 차단
  double radius_{0.30};            ///< 점마다 이 반경을 채운다 [m]
  double timeout_{0.5};            ///< [s] 이보다 오래된 구름은 쓰지 않는다
  double lethal_keepout_{0.46};    ///< [m] 이 반경 안에는 치명을 찍지 않는다 (§11)
  double robot_x_{0.0}, robot_y_{0.0};   ///< updateBounds 가 준 자세 (치명 금지 판정용)
  bool have_{false};
  bool has_last_{false};   ///< 지난 범위를 아직 한 번 더 포함해야 하는가
  double last_min_x_{0.0}, last_min_y_{0.0}, last_max_x_{0.0}, last_max_y_{0.0};
};

}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__PREDICTED_OBSTACLE_LAYER_HPP_
