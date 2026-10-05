// task_executor_node: BehaviorTree.CPP v3 작업 실행기 (components.md §3.5, §5.5).
//
// SrvS  assign_task (amr_msgs/AssignTask)  IDLE 이면 수락 → 블랙보드, 아니면 success=false
//                                          (message "busy:<…>" 등 거절 사유)
// Pub   task_status (amr_msgs/Task)        상태 전이마다 (IN_PROGRESS → COMPLETED / FAILED)
// Pub   executor/phase (String, latched)   IDLE/MOVING/DOCKING/LOADING/UNLOADING/RETURNING/…
// SrvS  clear_payload (std_srvs/Trigger)      되돌려 놓지 못한 물품을 작업자가 내렸음 → 작업 재개
// Pub   payload/attach (String), payload/mass (Float32), charging/enable (Bool) — 모두 latched
// Pub/Sub /fleet/charger_claims (String JSON {"robot","charger","since"}, 1 Hz 심장박동)
//       충전소 점유
// Sub   battery_state, safety/estop_active(latched: 버튼·센서 고장 E-stop 만, 계약 C1),
//       traffic/hold, traffic/yield_pose, localization/lost(latched), perception/detected_objects,
//       perception/dock_marker_pose
// ActC  navigate_to_pose, dock, spin, backup, wait
// SrvC  {global,local}_costmap/clear_entirely_{global,local}_costmap
// Groot BT::PublisherZMQ (groot.publisher_port / groot.server_port, 다중 로봇은 bringup 이
//       로봇 i 에 1666 + 2i / 1667 + 2i 를 준다)
// 트리 tick 은 벽시계 타이머(tick_rate_hz)로 돌고, 서비스·구독·액션 콜백과 같은 단일 스레드
// 실행기에서 처리된다.
#ifndef AMR_BEHAVIOR__TASK_EXECUTOR_NODE_HPP_
#define AMR_BEHAVIOR__TASK_EXECUTOR_NODE_HPP_

#include <memory>
#include <string>

#include "amr_behavior/executor_context.hpp"
#include "amr_behavior/task_tree.hpp"
#include "amr_msgs/msg/detected_object_array.hpp"
#include "amr_msgs/msg/task.hpp"
#include "amr_msgs/srv/assign_task.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace amr_behavior
{

class TaskExecutorNode : public rclcpp::Node
{
public:
  explicit TaskExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TaskExecutorNode() override;

  /// 트리 생성·Groot·타이머 시작. shared_from_this 가 필요해 생성자 밖에서 호출한다.
  void init();

  /// 트리를 한 번 tick 한다 (타이머 콜백, 테스트에서 직접 호출 가능).
  void tickOnce();

  const std::shared_ptr<ExecutorContext> & context() const {return ctx_;}
  BT::Tree * tree() {return tree_.get();}
  bool grootActive() const {return static_cast<bool>(groot_);}

private:
  void declareAndLoadParameters();
  void createInterfaces();
  void onAssignTask(
    const std::shared_ptr<amr_msgs::srv::AssignTask::Request> request,
    std::shared_ptr<amr_msgs::srv::AssignTask::Response> response);
  void publishChargerClaim(const std::string & charger, double since);
  void onChargerClaim(const std::string & json);

  std::shared_ptr<ExecutorContext> ctx_;
  TaskTreeConfig tree_config_;
  std::string robot_id_;
  std::string bt_xml_;
  double tick_rate_hz_{50.0};
  bool groot_enabled_{true};
  int groot_publisher_port_{1666};
  int groot_server_port_{1667};
  int groot_max_msg_per_second_{25};
  std::string bt_log_file_;
  bool publish_payload_mass_{true};
  int server_wait_timeout_ms_{5000};
  int goal_response_timeout_ms_{2000};

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<BT::PublisherZMQ> groot_;
  std::unique_ptr<BT::FileLogger> file_logger_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_pub_;
  rclcpp::Publisher<amr_msgs::msg::Task>::SharedPtr task_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr payload_attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr payload_mass_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charging_pub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr traffic_hold_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr yield_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lost_sub_;
  rclcpp::Subscription<amr_msgs::msg::DetectedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_sub_;
  rclcpp::Service<amr_msgs::srv::AssignTask>::SharedPtr assign_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_payload_srv_;
  std::string charger_claims_topic_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr claim_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr claim_sub_;
  rclcpp::TimerBase::SharedPtr claim_timer_;
};

/// robot_id 끝 숫자 → 0 기반 번호 (amr_01 → 0, amr_05 → 4, 숫자가 없으면 0). 충전소 선호 순서용.
int robotIndexFromId(const std::string & robot_id);

/// 충전소 점유 메시지 (JSON 한 줄) 만들기·읽기. 읽기 실패면 false.
std::string encodeChargerClaim(
  const std::string & robot, const std::string & charger,
  double since);
bool decodeChargerClaim(
  const std::string & json, std::string & robot, std::string & charger,
  double & since);

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__TASK_EXECUTOR_NODE_HPP_
