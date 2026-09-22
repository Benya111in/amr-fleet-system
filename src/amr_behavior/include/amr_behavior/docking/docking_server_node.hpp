// docking_server_node: amr_msgs/action/Dock 서버 (components.md §5.5).
//
// ActS dock (amr_msgs/action/Dock)  goal dock_id / approach_pose / max_retries
//                                   → 결과 success, 최종 오차, attempts_used
// Sub  perception/dock_marker_pose  PoseStamped, base_link 권장 (다른 프레임이면 tf2 로 변환)
// Pub  cmd_vel_nav (Twist)          제어 주기(control_rate_hz, 기본 20 Hz), 도킹 goal 실행 중에만
// 제어는 DockingController(ROS 비의존)에 위임한다. goal 은 한 번에 하나만 받는다 (실행 중이면
// 거절). detector_node 가 지정되면 goal 동안만 그 노드의 enabled 파라미터를 true 로 둔다
// (aruco_detector_node).
#ifndef AMR_BEHAVIOR__DOCKING__DOCKING_SERVER_NODE_HPP_
#define AMR_BEHAVIOR__DOCKING__DOCKING_SERVER_NODE_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>

#include "amr_behavior/docking/docking_controller.hpp"
#include "amr_msgs/action/dock.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_behavior
{
namespace docking
{

/// 마커 자세(base_link) → 관측. normal_axis = 마커 면 바깥 법선에 해당하는 마커 축 ("z", "-z",
/// "x", "-x").
std::optional<MarkerObservation> markerToObservation(
  const geometry_msgs::msg::Pose & pose, const std::string & normal_axis);

class DockingServerNode : public rclcpp::Node
{
public:
  using Dock = amr_msgs::action::Dock;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Dock>;

  explicit DockingServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// 도크 id 의 standoff [m] (docks.<id>.standoff, 없으면 기본값).
  double standoffFor(const std::string & dock_id) const;

private:
  rclcpp_action::GoalResponse onGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Dock::Goal> goal);
  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandle> handle);
  void onAccepted(const std::shared_ptr<GoalHandle> handle);
  void onMarker(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void controlStep();
  void finish(bool canceled);
  void publishCommand(const Command & cmd);
  void setDetectorEnabled(bool enabled);

  Params base_params_;
  std::map<std::string, double> standoffs_;
  int default_max_attempts_{3};
  double control_rate_hz_{20.0};
  std::string base_frame_;
  std::string normal_axis_;
  std::string detector_node_;

  std::unique_ptr<DockingController> controller_;
  Phase last_phase_{Phase::kIdle};   ///< 시도 실패(→ backup) 전이 로그용
  std::shared_ptr<GoalHandle> active_;
  std::optional<MarkerObservation> pending_obs_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Server<Dock>::SharedPtr server_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::AsyncParametersClient::SharedPtr detector_client_;
};

}  // namespace docking
}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__DOCKING__DOCKING_SERVER_NODE_HPP_
