// docking_server_node: amr_msgs/action/Dock 서버 (components.md §5.5).
//
// ActS dock (amr_msgs/action/Dock)  goal dock_id / approach_pose / max_retries
//                                   → 결과 success, 최종 오차, attempts_used
// Sub  perception/dock_marker_pose  PoseStamped, base_link 권장 (다른 프레임이면 tf2 로 변환).
//                                   자세 = amr_perception aruco_detector_node 의 마커 모델 프레임
//                                   (+x = 판 바깥 법선, +z = 위) → marker_normal_axis 기본 "x"
// Sub  perception/dock_marker_id    Int32 (검출기가 자세와 같은 주기에 낸다).
//                                   docks.<id>.marker_id 가 있으면 다른 id 의 관측은 버린다
//                                   (이웃 도크 마커로 붙지 않도록)
// Pub  cmd_vel_nav (Twist)          제어 주기(control_rate_hz, 기본 20 Hz), 도킹 goal 실행 중에만
// Pub  safety/dock_exclusion        PolygonStamped (frame = base_frame), 제어 주기. 도킹 세션
//                                   (goal 수락 ~ goal 이 끝나고 판에서 standoff + release 밖으로
//                                   물러나거나 마커가 안 보일 때까지) 동안 마커 판·벽 둘레
//                                   사각형을 보내 safety_node 가 그 안의 점에
//                                   exclusion_stop_distance 를 쓰게 한다 (계약 C2: docked
//                                   standoff 가 근접 정지를 걸지 않도록).
// 제어는 DockingController(ROS 비의존)에 위임한다. goal 은 한 번에 하나만 받는다 (실행 중이면
// 거절, 취소 중인 goal 은 새 goal 이 선점). detector_enable_service 가 지정되면 세션 동안만
// aruco_detector_node 를 켠다 (std_srvs/SetBool, perception/aruco/enable).
#ifndef AMR_BEHAVIOR__DOCKING__DOCKING_SERVER_NODE_HPP_
#define AMR_BEHAVIOR__DOCKING__DOCKING_SERVER_NODE_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "amr_behavior/docking/docking_controller.hpp"
#include "amr_behavior/docking/marker_relocalization.hpp"
#include "amr_msgs/action/dock.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_behavior
{
namespace docking
{

/// 마커 자세(base_link) → 관측. normal_axis = 마커 면 바깥 법선에 해당하는 마커 축 ("x", "-x",
/// "z", "-z"). amr_perception aruco_detector_node 는 마커 모델 프레임(+x 법선)으로 낸다 → "x".
std::optional<MarkerObservation> markerToObservation(
  const geometry_msgs::msg::Pose & pose, const std::string & normal_axis);

/// 도킹 예외 사각형 (마커 프레임: x = 바깥 법선, y = 판 가로).
struct ExclusionBox
{
  double half_width{0.5};     ///< 판 중심에서 가로 ± [m]
  double depth{0.3};          ///< 판 면 뒤(벽 쪽) [m]
  double front_margin{0.12};  ///< 판 면 앞(로봇 쪽) [m] — LiDAR σ 3 cm 의 4배
};

/// 마커 관측 → base_link 기준 예외 사각형 꼭짓점 4개 (반시계).
std::vector<std::pair<double, double>> exclusionPolygon(
  const MarkerObservation & marker, const ExclusionBox & box);

/// 로봇(base_link 원점)과 마커 판 면 사이 거리 (법선 방향) [m].
double distanceFromPlate(const MarkerObservation & marker);

class DockingServerNode : public rclcpp::Node
{
public:
  using Dock = amr_msgs::action::Dock;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Dock>;

  explicit DockingServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// 도크 id 의 standoff [m] (docks.<id>.standoff, 없으면 기본값).
  double standoffFor(const std::string & dock_id) const;
  /// 도크 id 의 ArUco 마커 id (docks.<id>.marker_id, 없으면 −1 = 아무 마커).
  int markerIdFor(const std::string & dock_id) const;
  /// 도킹 세션(예외 사각형 발행) 중인지.
  bool exclusionSessionActive() const {return session_active_;}
  const std::string & normalAxis() const {return normal_axis_;}

private:
  rclcpp_action::GoalResponse onGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Dock::Goal> goal);
  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandle> handle);
  void onAccepted(const std::shared_ptr<GoalHandle> handle);
  void onMarker(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  /// 이 goal 의 도크 마커가 아니면 true: 검출기가 id 를 낸 적이 있는데 최근
  /// (marker_id_max_age) id 가 없거나 다르다. 도크 id 를 모르거나 id 를 받은 적이 없으면 false.
  bool wrongMarker(double now) const;
  /// 다른 도크의 마커를 봤을 때: 그 마커의 지도 자세로 로봇 자세를 역산해
  /// localization/marker_fix 로 낸다 (연속 관측 min_observations 회가 서로
  /// tolerance 안에 들어올 때만). 도킹 판정은 바꾸지 않는다.
  void publishMarkerFix(int observed_id, const MarkerObservation & obs);
  void controlStep();
  void goalStep(double now);
  void exclusionStep(double now);
  void endSession();
  void finish(bool canceled);
  void publishCommand(const Command & cmd);
  void setDetectorEnabled(bool enabled);

  Params base_params_;
  std::map<std::string, double> standoffs_;
  std::map<std::string, int> marker_ids_;
  std::map<int, MapPose2D> marker_poses_;     ///< ArUco id → 마커 지도 자세 (계약 C3: +x 바깥 법선)
  bool reloc_enabled_{true};
  double reloc_max_range_{3.0};               ///< [m] 이보다 먼 관측은 쓰지 않는다
  double reloc_tolerance_{0.30};              ///< [m] 연속 관측이 이 안에 들어와야 한다
  int reloc_min_observations_{3};
  double reloc_position_sigma_{0.05};         ///< [m] 발행 공분산
  double reloc_yaw_sigma_{0.05};              ///< [rad]
  int reloc_streak_{0};
  int reloc_streak_id_{-1};
  MapPose2D reloc_last_fix_;
  int expected_marker_id_{-1};       ///< 이번 세션의 도크 마커 id (−1 = 확인 안 함)
  int last_marker_id_{-1};
  double last_marker_id_time_{-1.0};
  double marker_id_max_age_{0.2};    ///< [s] 이보다 오래된 id 는 쓰지 않는다
  int default_max_attempts_{3};
  double control_rate_hz_{20.0};
  std::string base_frame_;
  std::string normal_axis_;
  std::string detector_service_;

  // 도킹 예외 (계약 C2)
  bool exclusion_enabled_{true};
  ExclusionBox exclusion_box_;
  double exclusion_release_distance_{0.5};
  double exclusion_marker_timeout_{0.5};
  bool session_active_{false};
  double session_standoff_{0.65};
  std::optional<MarkerObservation> last_obs_;   ///< 마지막 관측 (세션 중, goal 밖 포함)
  double last_obs_time_{-1.0};

  std::unique_ptr<DockingController> controller_;
  Phase last_phase_{Phase::kIdle};   ///< 시도 실패(→ backup) 전이 로그용
  std::shared_ptr<GoalHandle> active_;
  std::optional<MarkerObservation> pending_obs_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Server<Dock>::SharedPtr server_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr marker_id_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr exclusion_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr marker_fix_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr detector_client_;
};

}  // namespace docking
}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__DOCKING__DOCKING_SERVER_NODE_HPP_
