// task_executor_node 구현 (task_executor_node.hpp 설명 참고).
#include "amr_behavior/task_executor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "amr_behavior/bt_nodes/bt_conversions.hpp"
#include "amr_behavior/bt_registry.hpp"

namespace amr_behavior
{

namespace
{
/// 네임스페이스 "/amr_01" → "amr_01" (다중 로봇: robot_id 기본값).
std::string robotIdFromNamespace(const std::string & ns)
{
  std::string id = ns;
  id.erase(0, id.find_first_not_of('/'));
  const auto pos = id.find('/');
  return pos == std::string::npos ? id : id.substr(0, pos);
}

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}
}  // namespace

TaskExecutorNode::TaskExecutorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("task_executor_node", options)
{
  ctx_ = std::make_shared<ExecutorContext>([this]() {return now().seconds();});
  declareAndLoadParameters();
  createInterfaces();
}

TaskExecutorNode::~TaskExecutorNode()
{
  tick_timer_.reset();
  groot_.reset();
  file_logger_.reset();
  if (tree_) {
    tree_->haltTree();
  }
}

void TaskExecutorNode::declareAndLoadParameters()
{
  robot_id_ = declare_parameter<std::string>("robot_id", "");
  if (robot_id_.empty()) {
    robot_id_ = robotIdFromNamespace(get_namespace());
  }
  ctx_->setRobotId(robot_id_);

  bt_xml_ = declare_parameter<std::string>("bt_xml", "");
  if (bt_xml_.empty()) {
    bt_xml_ = ament_index_cpp::get_package_share_directory("amr_behavior") +
      "/behavior_trees/task_executor.xml";
  }
  tick_rate_hz_ = declare_parameter<double>("tick_rate_hz", 50.0);
  groot_enabled_ = declare_parameter<bool>("groot.enabled", true);
  groot_publisher_port_ = declare_parameter<int>("groot.publisher_port", 1666);
  groot_server_port_ = declare_parameter<int>("groot.server_port", 1667);
  groot_max_msg_per_second_ = declare_parameter<int>("groot.max_msg_per_second", 25);
  bt_log_file_ = declare_parameter<std::string>("bt_log_file", "");
  publish_payload_mass_ = declare_parameter<bool>("publish_payload_mass", true);
  server_wait_timeout_ms_ = declare_parameter<int>("server_wait_timeout_ms", 5000);
  goal_response_timeout_ms_ = declare_parameter<int>("goal_response_timeout_ms", 2000);

  // 작업 수락 정책
  AcceptPolicy policy;
  policy.battery_low_percent = declare_parameter<double>(
    "accept_battery_min_percent", policy.battery_low_percent);
  policy.accept_while_returning = declare_parameter<bool>(
    "accept_while_returning", policy.accept_while_returning);
  ctx_->setAcceptPolicy(policy);

  // 물품 표 (config/robot_params.yaml payload.<type>, 명세 8장 표가 기본값)
  const std::map<std::string, PayloadSpec> defaults = {
    {"small", {2.0, 5.0}}, {"medium", {10.0, 10.0}}, {"large", {25.0, 15.0}}};
  const auto types = declare_parameter<std::vector<std::string>>(
    "payload_types", std::vector<std::string>{"small", "medium", "large"});
  std::map<std::string, PayloadSpec> table;
  for (const auto & type : types) {
    const auto it = defaults.find(type);
    const PayloadSpec d = it == defaults.end() ? PayloadSpec{} : it->second;
    PayloadSpec spec;
    spec.mass = declare_parameter<double>("payload." + type + ".mass", d.mass);
    spec.load_time = declare_parameter<double>("payload." + type + ".load_time", d.load_time);
    table[type] = spec;
  }
  ctx_->setPayloadTable(table);

  // 도크 표 (behavior.yaml docks.*): staging 자세 [x, y, yaw] (map)
  const std::string map_frame = declare_parameter<std::string>("map_frame", "map");
  const auto dock_ids = declare_parameter<std::vector<std::string>>(
    "docks.ids", std::vector<std::string>{});
  std::vector<DockSpec> docks;
  for (const auto & id : dock_ids) {
    const auto staging = declare_parameter<std::vector<double>>(
      "docks." + id + ".staging", std::vector<double>{});
    if (staging.size() != 3U) {
      RCLCPP_WARN(get_logger(), "docks.%s.staging 은 [x, y, yaw] 여야 한다 → 무시", id.c_str());
      continue;
    }
    DockSpec d;
    d.id = id;
    d.frame_id = map_frame;
    d.x = staging[0];
    d.y = staging[1];
    d.yaw = staging[2];
    docks.push_back(d);
  }
  ctx_->setDocks(
    docks, declare_parameter<double>("dock_match_radius", 1.6),
    declare_parameter<double>("dock_match_yaw_tolerance", M_PI));

  // 트리 설정 키 (TaskTreeConfig, behavior_tree.md 표)
  TaskTreeConfig & c = tree_config_;
  c.nav_attempts = declare_parameter<int>("nav_attempts", c.nav_attempts);
  c.perception_attempts = declare_parameter<int>("perception_attempts", c.perception_attempts);
  c.dock_attempts = declare_parameter<int>("dock_attempts", c.dock_attempts);
  c.dock_max_retries = static_cast<unsigned>(std::max<int64_t>(
      0, declare_parameter<int>("dock_max_retries", static_cast<int>(c.dock_max_retries))));
  auto ms = [this](const std::string & name, unsigned fallback) {
      return static_cast<unsigned>(std::max<int64_t>(
               0, declare_parameter<int>(name, static_cast<int>(fallback))));
    };
  c.perception_timeout_ms = ms("perception_timeout_ms", c.perception_timeout_ms);
  c.relocalization_timeout_ms = ms("relocalization_timeout_ms", c.relocalization_timeout_ms);
  c.charge_timeout_ms = ms("charge_timeout_ms", c.charge_timeout_ms);
  c.charge_retry_delay_ms = ms("charge_retry_delay_ms", c.charge_retry_delay_ms);
  c.perception_class = declare_parameter<std::string>("perception_class", c.perception_class);
  c.perception_max_distance = declare_parameter<double>(
    "perception_max_distance", c.perception_max_distance);
  c.battery_low_percent = declare_parameter<double>("battery_low_percent", c.battery_low_percent);
  c.battery_resume_percent = declare_parameter<double>(
    "battery_resume_percent", c.battery_resume_percent);
  c.recovery_spin_angle = declare_parameter<double>("recovery_spin_angle", c.recovery_spin_angle);
  c.recovery_wait_s = declare_parameter<double>("recovery_wait_s", c.recovery_wait_s);
  c.recovery_backup_dist = declare_parameter<double>(
    "recovery_backup_dist", c.recovery_backup_dist);
  c.perception_spin_angle = declare_parameter<double>(
    "perception_spin_angle", c.perception_spin_angle);
  c.dock_backup_dist = declare_parameter<double>("dock_backup_dist", c.dock_backup_dist);
  c.marker_max_age = declare_parameter<double>("marker_max_age", c.marker_max_age);
  c.undock_dist = declare_parameter<double>("undock_dist", c.undock_dist);
  const auto waiting = declare_parameter<std::vector<double>>(
    "waiting_pose", std::vector<double>{22.0, -16.0, 0.0});
  if (waiting.size() == 3U) {
    c.waiting_pose = makePose(map_frame, waiting[0], waiting[1], waiting[2]);
  } else {
    RCLCPP_WARN(get_logger(), "waiting_pose 는 [x, y, yaw] 여야 한다 → 복귀 생략");
  }
  c.charger_dock_id = declare_parameter<std::string>("charger_dock_id", "");
  if (const auto dock = ctx_->dock(c.charger_dock_id)) {
    c.charger_goal = makePose(dock->frame_id, dock->x, dock->y, dock->yaw);
  } else if (!c.charger_dock_id.empty()) {
    RCLCPP_WARN(
      get_logger(), "charger_dock_id '%s' 가 docks 표에 없다 → 충전 비활성",
      c.charger_dock_id.c_str());
    c.charger_dock_id.clear();
  }
  if (c.charger_dock_id.empty()) {
    // 충전소가 없으면 Charge 서브트리가 실패만 반복하므로 자동 충전을 끈다 (수락 정책의 배터리
    // 거절은 유지)
    c.battery_low_percent = -1.0;
    RCLCPP_INFO(get_logger(), "charger_dock_id 없음 → 자동 충전 비활성");
  }
}

void TaskExecutorNode::createInterfaces()
{
  phase_pub_ = create_publisher<std_msgs::msg::String>("executor/phase", latchedQos());
  task_status_pub_ = create_publisher<amr_msgs::msg::Task>("task_status", rclcpp::QoS(10));
  payload_attach_pub_ = create_publisher<std_msgs::msg::String>("payload/attach", latchedQos());
  if (publish_payload_mass_) {
    payload_mass_pub_ = create_publisher<std_msgs::msg::Float32>("payload/mass", latchedQos());
  }
  charging_pub_ = create_publisher<std_msgs::msg::Bool>("charging/enable", latchedQos());

  ExecutorHooks hooks;
  hooks.publish_phase = [this](const std::string & phase) {
      std_msgs::msg::String msg;
      msg.data = phase;
      phase_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "phase → %s", phase.c_str());
    };
  hooks.publish_task_status = [this](const amr_msgs::msg::Task & task) {
      task_status_pub_->publish(task);
    };
  hooks.publish_payload_attach = [this](const std::string & item) {
      std_msgs::msg::String msg;
      msg.data = item;
      payload_attach_pub_->publish(msg);
    };
  hooks.publish_payload_mass = [this](double mass) {
      if (payload_mass_pub_) {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(mass);
        payload_mass_pub_->publish(msg);
      }
    };
  hooks.publish_charging = [this](bool enable) {
      std_msgs::msg::Bool msg;
      msg.data = enable;
      charging_pub_->publish(msg);
    };
  hooks.log_info = [this](const std::string & text) {
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
    };
  hooks.log_warn = [this](const std::string & text) {
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
    };
  ctx_->setHooks(hooks);

  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::QoS(10), [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      // REP: percentage 0~1, 미측정 NaN
      ctx_->updateBattery(static_cast<double>(msg->percentage) * 100.0);
    });
  // safety/estop_active, localization/lost 는 latched 로 발행된다 (components.md §5.4,
  // kidnap_monitor)
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "safety/estop_active", latchedQos(), [this](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data != ctx_->estopActive()) {
        RCLCPP_WARN(get_logger(), "E-stop %s", msg->data ? "활성 → 작업 일시 중단" : "해제 → 재개");
      }
      ctx_->updateEstop(msg->data);
    });
  lost_sub_ = create_subscription<std_msgs::msg::Bool>(
    "localization/lost", latchedQos(), [this](const std_msgs::msg::Bool::SharedPtr msg) {
      ctx_->updateLocalizationLost(msg->data);
    });
  // traffic/hold 는 QoS 미정 → volatile 구독 (latched/volatile 발행자 모두와 호환)
  traffic_hold_sub_ = create_subscription<std_msgs::msg::Bool>(
    "traffic/hold", rclcpp::QoS(10), [this](const std_msgs::msg::Bool::SharedPtr msg) {
      ctx_->updateTrafficHold(msg->data);
    });
  yield_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "traffic/yield_pose", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {ctx_->updateYieldPose(*msg);});
  objects_sub_ = create_subscription<amr_msgs::msg::DetectedObjectArray>(
    "perception/detected_objects", rclcpp::SensorDataQoS(),
    [this](const amr_msgs::msg::DetectedObjectArray::SharedPtr msg) {
      ctx_->updateDetectedObjects(*msg);
    });
  marker_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "perception/dock_marker_pose", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {ctx_->updateDockMarker(*msg);});

  assign_srv_ = create_service<amr_msgs::srv::AssignTask>(
    "assign_task",
    [this](
      const std::shared_ptr<amr_msgs::srv::AssignTask::Request> request,
      std::shared_ptr<amr_msgs::srv::AssignTask::Response> response) {
      onAssignTask(request, response);
    });
}

void TaskExecutorNode::init()
{
  RosNodeParams ros = RosNodeParams::make(shared_from_this());
  ros.server_timeout = std::chrono::milliseconds(goal_response_timeout_ms_);
  ros.wait_for_server_timeout = std::chrono::milliseconds(server_wait_timeout_ms_);
  registerCoreNodes(factory_, ctx_);
  registerRosNodes(factory_, ros);

  tree_ = std::make_unique<BT::Tree>(createTaskTree(factory_, bt_xml_, tree_config_));
  RCLCPP_INFO(
    get_logger(), "BT 로드: %s (노드 %zu 개, 자체 노드 타입 %zu 종)", bt_xml_.c_str(),
    tree_->nodes.size(), customNodeIds(factory_).size());

  if (groot_enabled_) {
    try {
      groot_ = std::make_unique<BT::PublisherZMQ>(
        *tree_, static_cast<unsigned>(groot_max_msg_per_second_),
        static_cast<unsigned>(groot_publisher_port_), static_cast<unsigned>(groot_server_port_));
      RCLCPP_INFO(
        get_logger(), "Groot ZMQ: publisher :%d, server :%d", groot_publisher_port_,
        groot_server_port_);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "Groot ZMQ 퍼블리셔를 열 수 없다 (%s) → 시각화 없이 계속", e.what());
    }
  }
  if (!bt_log_file_.empty()) {
    file_logger_ = std::make_unique<BT::FileLogger>(*tree_, bt_log_file_.c_str());
  }

  ctx_->setPhase(phase::kIdle);
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, tick_rate_hz_));
  tick_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() {tickOnce();});
}

void TaskExecutorNode::tickOnce()
{
  if (!tree_) {
    return;
  }
  try {
    const BT::NodeStatus status = tree_->tickRoot();
    if (status != BT::NodeStatus::RUNNING) {
      // 메인 루프는 끝나지 않도록 설계했다. 끝났다면 되돌리고 다시 시작한다.
      RCLCPP_WARN(get_logger(), "루트가 %s 로 끝남 → 재시작", BT::toStr(status, false).c_str());
      tree_->haltTree();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "BT tick 예외: %s → 트리 정지 후 재시작", e.what());
    tree_->haltTree();
    ctx_->setPhase(phase::kError);
  }
}

void TaskExecutorNode::onAssignTask(
  const std::shared_ptr<amr_msgs::srv::AssignTask::Request> request,
  std::shared_ptr<amr_msgs::srv::AssignTask::Response> response)
{
  const AcceptDecision decision = ctx_->acceptTask(request->task);
  response->success = decision.accepted;
  response->message = decision.message;
  response->robot_id = robot_id_;
  if (decision.accepted && tree_) {
    resetTaskProgress(*tree_->rootBlackboard());
    const auto & stamp = request->task.header.stamp;
    if (stamp.sec != 0 || stamp.nanosec != 0U) {
      const double latency_ms = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type()))
        .seconds() * 1e3;
      RCLCPP_INFO(
        get_logger(), "assign_task 수락 %s (디스패치→수신 %.1f ms)",
        request->task.task_id.c_str(), latency_ms);
    }
  } else if (!decision.accepted) {
    RCLCPP_INFO(
      get_logger(), "assign_task 거절 %s: %s", request->task.task_id.c_str(),
      decision.message.c_str());
  }
}

}  // namespace amr_behavior
