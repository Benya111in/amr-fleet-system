#include "amr_rl_controller/rl_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "amr_navigation/ros_utils.hpp"
#include "amr_rl_controller/core/geometry.hpp"
#include "amr_rl_controller/core/kinematics.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace amr_rl_controller
{

namespace C = contract;
namespace nav = amr_navigation;

namespace
{
// rl_controller/stats 의 필드 순서 (Float64MultiArray.data)
constexpr const char * kStatsFields =
  "0 cycle_ms, 1 obs_ms, 2 mask_ms, 3 infer_ms, 4 source(0 policy 1 fallback_cell 2 dwa 3 align), "
  "5 dwa_reason(0 none 1 state 2 policy_error 3 timeout 4 nonfinite), 6 action, 7 model_action, "
  "8 n_valid, 9 saturated, 10 v, 11 w, 12 new_decision, 13 decisions, 14 dwa_ticks, "
  "15 fallback_cell, 16 invalid_action, 17 timeouts, 18 scan_age, 19 track_age, 20 n_tracks, "
  "21 v_cap, 22 dev_in, 23 dev_peak";

double toDouble(const std::string & s, double dflt)
{
  try {
    return std::stod(s);
  } catch (const std::exception &) {
    return dflt;
  }
}
}  // namespace

void RLController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RLController: parent node expired");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  const std::string p = name_ + ".rl.";

  // 폴백 DWA: 같은 플러그인 이름으로 configure → 기존 <name>.* DWA 파라미터를 그대로 읽는다
  dwa_fallback_ = nav::param(node, p + "dwa_fallback", true);
  if (dwa_fallback_) {
    dwa_ = std::make_shared<nav::DWAController>();
    dwa_->configure(parent, name_, tf_, costmap_ros_);
  }

  const std::string model_path = nav::param(node, p + "model_path", std::string(""));
  const std::string mode = nav::param(node, p + "mode", std::string("C"));
  if (mode != "B" && mode != "C") {
    throw std::runtime_error("RLController: " + p + "mode 는 B 또는 C: '" + mode + "'");
  }
  if (model_path.empty()) {
    throw std::runtime_error("RLController: " + p + "model_path 가 비어 있다");
  }
  OnnxPolicyConfig pc;
  pc.model_path = model_path;
  pc.expected_arm = mode == "B" ? Arm::kB : Arm::kC;
  pc.intra_op_threads = nav::param(node, p + "intra_op_threads", 1);
  pc.require_metadata = nav::param(node, p + "require_metadata", true);
  policy_ = std::make_unique<OnnxPolicy>(pc);   // 계약 위반이면 여기서 예외 (onnx_io.md)

  double controller_frequency = C::kControllerHz;
  if (node->has_parameter("controller_frequency")) {
    node->get_parameter("controller_frequency", controller_frequency);
  }
  EngineConfig ec;
  ec.control_period = 1.0 / std::max(1.0, controller_frequency);
  ec.max_inference_ms = nav::param(node, p + "max_inference_ms", 25.0);
  ec.hysteresis_beta = nav::param(node, p + "hysteresis_beta", 0.0);
  ec.yaw_tol = nav::param(node, p + "yaw_goal_tolerance", 0.05);
  engine_ = std::make_unique<DecisionEngine>(ec, policy_.get());

  v_cap_op_ = nav::param(node, name_ + ".max_vel_x", C::kVMaxOp);
  goal_reset_distance_ = nav::param(node, p + "goal_reset_distance", 0.25);
  transform_tolerance_ = nav::param(node, p + "transform_tolerance", 0.2);
  scan_timeout_ = nav::param(node, p + "scan_timeout", 0.3);
  track_timeout_ = nav::param(node, p + "track_timeout", 0.5);
  profiler_timeout_ = nav::param(node, p + "profiler_timeout", 0.2);
  profiler_dt_ = 1.0 / std::max(1.0, nav::param(node, p + "profiler_rate", 50.0));

  // 마스크는 계약 상수로 DWA 판정을 재현한다 — 배포 DWA 파라미터가 다르면 알린다
  const std::vector<std::pair<std::string, double>> dwa_consts{
    {"sim_dt", C::kSimDt}, {"sim_time_min", C::kSimTimeMin}, {"sim_time_max", C::kSimTimeMax},
    {"vo_time_horizon", C::kVoTimeHorizon}, {"vo_margin", C::kVoMargin},
    {"vo_max_range", C::kVoMaxRange}, {"dynamic_speed_threshold", C::kDynSpeedThreshold},
    {"vo_time_tie", C::kVoTimeTie}, {"goal_overshoot_margin", C::kGoalOvershootMargin},
    {"dynamic_fast_speed", C::kDynFastSpeed}};
  for (const auto & kv : dwa_consts) {
    const double v = nav::param(node, name_ + "." + kv.first, kv.second);
    if (std::abs(v - kv.second) > 1e-9) {
      RCLCPP_WARN(
        logger_, "RLController: %s.%s = %g 이 계약 값 %g 과 다르다 (마스크는 계약 값을 쓴다)",
        name_.c_str(), kv.first.c_str(), v, kv.second);
    }
  }

  const std::string scan_topic = nav::param(node, p + "scan_topic", std::string("scan_filtered"));
  const std::string tracks_topic =
    nav::param(node, p + "tracks_topic", std::string("perception/tracked_obstacles"));
  const std::string zone_topic = nav::param(node, p + "zone_topic", std::string("safety/zone"));
  const std::string diag_topic = nav::param(
    node, p + "diagnostics_topic", std::string(
      "diagnostics"));
  const std::string prof_topic =
    nav::param(node, p + "profiler_state_topic", std::string("velocity_profiler/state"));
  const std::string stats_topic =
    nav::param(node, p + "stats_topic", std::string("rl_controller/stats"));

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {onScan(m);});
  tracks_sub_ = node->create_subscription<amr_msgs::msg::TrackedObstacleArray>(
    tracks_topic, rclcpp::QoS(5),
    [this](const amr_msgs::msg::TrackedObstacleArray::SharedPtr m) {onTracks(m);});
  zone_sub_ = node->create_subscription<std_msgs::msg::UInt8>(
    zone_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::UInt8::SharedPtr m) {onZone(m);});
  diag_sub_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    diag_topic, rclcpp::QoS(10),
    [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) {onDiagnostics(m);});
  profiler_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    prof_topic, rclcpp::QoS(10),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {onProfilerState(m);});
  stats_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(stats_topic, 10);

  RCLCPP_INFO(
    logger_,
    "RLController '%s' 로드: 모델 %s (%s) | 모드 %s, 결정 %.2f s = 틱 %.3f s x %d, 추론 상한 "
    "%.1f ms, β %.2f, v_cap %.2f m/s, DWA 폴백 %s | 스캔 %s",
    name_.c_str(), model_path.c_str(), policy_->describe().c_str(), mode.c_str(),
    ec.decision_period, ec.control_period, engine_->ticksPerDecision(), ec.max_inference_ms,
    ec.hysteresis_beta, v_cap_op_, dwa_fallback_ ? "on" : "off", scan_topic.c_str());
  RCLCPP_INFO(logger_, "RLController stats (%s): %s", stats_topic.c_str(), kStatsFields);
}

void RLController::cleanup()
{
  if (dwa_) {
    dwa_->cleanup();
  }
  scan_sub_.reset();
  tracks_sub_.reset();
  zone_sub_.reset();
  diag_sub_.reset();
  profiler_sub_.reset();
  stats_pub_.reset();
  engine_.reset();
  policy_.reset();
  dwa_.reset();
}

void RLController::activate()
{
  stats_pub_->on_activate();
  if (dwa_) {
    dwa_->activate();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  engine_->reset();
  have_goal_ = false;
}

void RLController::deactivate()
{
  stats_pub_->on_deactivate();
  if (dwa_) {
    dwa_->deactivate();
  }
}

void RLController::setPlan(const nav_msgs::msg::Path & path)
{
  if (dwa_) {
    dwa_->setPlan(path);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  plan_ = path;
  if (path.poses.empty()) {
    return;
  }
  const auto & g = path.poses.back().pose.position;
  const bool same = have_goal_ && goal_frame_ == path.header.frame_id &&
    std::hypot(g.x - goal_x_, g.y - goal_y_) <= goal_reset_distance_;
  if (!same) {
    // 새 목표 = 에피소드 경계: 은닉 상태·이탈 상태 0 (속도 명령은 이어 간다)
    engine_->resetEpisode();
    have_goal_ = true;
    goal_x_ = g.x;
    goal_y_ = g.y;
    goal_frame_ = path.header.frame_id;
  }
}

void RLController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (dwa_) {
    dwa_->setSpeedLimit(speed_limit, percentage);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (speed_limit <= 0.0 || (percentage && speed_limit >= 100.0 - 1e-9)) {
    speed_limit_ = kInf;   // nav2_costmap_2d::NO_SPEED_LIMIT (0) → 제한 없음
  } else {
    speed_limit_ = percentage ? v_cap_op_ * speed_limit / 100.0 : speed_limit;
  }
}

void RLController::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  scan_ = msg;
}

void RLController::onTracks(const amr_msgs::msg::TrackedObstacleArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  tracks_ = msg;
}

void RLController::onZone(const std_msgs::msg::UInt8::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  safety_.zone = std::min<int>(msg->data, C::kZoneStop);
}

void RLController::onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  static const std::string kSuffix = "/safety_node";
  for (const auto & st : msg->status) {
    if (st.name.size() < kSuffix.size() ||
      st.name.compare(st.name.size() - kSuffix.size(), kSuffix.size(), kSuffix) != 0)
    {
      continue;
    }
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    for (const auto & kv : st.values) {
      if (kv.key == "speed_limit_mps") {
        const double v = toDouble(kv.value, kInf);
        safety_.speed_limit = std::isfinite(v) ? v : kInf;
      } else if (kv.key == "min_ttc_s") {
        const double v = toDouble(kv.value, kInf);
        safety_.min_ttc = std::isfinite(v) && v >= 0.0 ? v : kInf;
      } else if (kv.key == "proximity_stop") {
        safety_.proximity_stop = kv.value == "true";
      }
    }
  }
}

void RLController::onProfilerState(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 5) {
    return;
  }
  std::lock_guard<std::mutex> lock(sensor_mutex_);
  const double w_ref = msg->data[3];
  profiler_a_v_ = msg->data[4];   // a_ref (실행 기준 가속도)
  // ω 는 가속도 필드가 없어 50 Hz 기준 ω_ref 의 차분으로 (프로파일러 고정 주기)
  profiler_a_w_ = profiler_have_w_ ? (w_ref - profiler_w_ref_) / profiler_dt_ : 0.0;
  profiler_w_ref_ = w_ref;
  profiler_have_w_ = true;
  profiler_stamp_ = clock_->now();
}

void RLController::buildInput(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  TickInput & in)
{
  const rclcpp::Time now = clock_->now();
  in.now = now.seconds();
  ContractState & s = in.state;
  const std::string frame = pose.header.frame_id;
  const nav::core::Pose2D robot = nav::toPose2D(pose.pose);
  s.pose = robot;
  s.v_meas = velocity.linear.x;
  s.w_meas = velocity.angular.z;
  s.v_cap = std::min(v_cap_op_, speed_limit_);

  auto invalid = [&in](const std::string & why) {
      if (in.state_ok) {
        in.state_ok = false;
        in.state_error = why;
      }
    };

  // 경로 (plan 프레임 → 자세 프레임)
  nav::core::Pose2D T;
  if (!nav::lookupTransform2D(*tf_, frame, plan_.header.frame_id, transform_tolerance_, T)) {
    invalid("경로 변환 실패 " + plan_.header.frame_id + " → " + frame);
  } else {
    s.path.reserve(plan_.poses.size());
    for (const auto & ps : plan_.poses) {
      const auto q = nav::core::compose(T, nav::toPose2D(ps.pose));
      s.path.push_back({q.x, q.y});
    }
    const auto g = nav::core::compose(T, nav::toPose2D(plan_.poses.back().pose));
    in.dist_to_goal = std::hypot(g.x - robot.x, g.y - robot.y);
    in.yaw_error_to_goal = wrapAngle(g.theta - robot.theta);
  }

  sensor_msgs::msg::LaserScan::SharedPtr scan;
  amr_msgs::msg::TrackedObstacleArray::SharedPtr tracks;
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    scan = scan_;
    tracks = tracks_;
    s.safety = safety_;
    if ((now - profiler_stamp_).seconds() <= profiler_timeout_) {
      s.a_v_meas = profiler_a_v_;
      s.a_w_meas = profiler_a_w_;
    }
  }

  // 스캔: 720 빔, 계약 각도, 나이 ≤ scan_timeout
  if (!scan) {
    invalid("스캔 없음");
  } else {
    s.scan_age = (now - rclcpp::Time(scan->header.stamp, now.get_clock_type())).seconds();
    const bool angles_ok = std::abs(scan->angle_min - C::kLidarAngleMin) <= 1e-3 &&
      std::abs(scan->angle_increment - C::kLidarAngleInc) <= 1e-5;
    if (static_cast<int>(scan->ranges.size()) != C::kLidarSamples) {
      invalid("스캔 빔 수 " + std::to_string(scan->ranges.size()));
    } else if (!angles_ok) {
      invalid("스캔 각도가 계약과 다르다");
    } else if (s.scan_age > scan_timeout_ || s.scan_age < -1.0) {
      invalid("스캔 나이 " + std::to_string(s.scan_age) + " s");
    }
    s.scan_age = std::max(0.0, s.scan_age);
    s.scan_ranges.assign(scan->ranges.begin(), scan->ranges.end());
  }

  // 트랙: 나이 ≤ track_timeout 이면 자세 프레임으로 (외삽은 계약 함수가 track_age 로 한다)
  if (tracks) {
    const double age = (now - rclcpp::Time(tracks->header.stamp, now.get_clock_type())).seconds();
    s.track_age = std::max(0.0, age);
    nav::core::Pose2D Tt;
    if (age <= track_timeout_ && age >= -1.0 &&
      nav::lookupTransform2D(*tf_, frame, tracks->header.frame_id, 0.0, Tt))
    {
      const double c = std::cos(Tt.theta);
      const double sn = std::sin(Tt.theta);
      for (const auto & o : tracks->obstacles) {
        Track t;
        t.id = static_cast<int64_t>(o.track_id);
        t.x = Tt.x + c * o.position.x - sn * o.position.y;
        t.y = Tt.y + sn * o.position.x + c * o.position.y;
        t.vx = c * o.velocity.x - sn * o.velocity.y;
        t.vy = sn * o.velocity.x + c * o.velocity.y;
        t.is_dynamic = o.is_dynamic;
        s.tracks.push_back(t);
      }
    }
  }
}

geometry_msgs::msg::TwistStamped RLController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto t0 = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (plan_.poses.empty()) {
    throw nav2_core::PlannerException("RLController: no plan");
  }
  TickInput in;
  buildInput(pose, velocity, in);
  TickResult r = engine_->tick(in);
  double v = r.v;
  double w = r.w;
  if (r.need_dwa) {
    if (dwa_) {
      const auto cmd = dwa_->computeVelocityCommands(pose, velocity, goal_checker);
      v = cmd.twist.linear.x;
      w = cmd.twist.angular.z;
    } else {
      // DWA 폴백을 끈 경우: 계약 폴백 규칙(최대 감속, ω 를 0 쪽으로)을 한 틱 만큼
      const double dt = engine_->config().control_period;
      const double v0 = engine_->lastV();
      const double w0 = engine_->lastW();
      v = v0 > 0.0 ? std::max(0.0, v0 - C::kAMax * dt) : std::min(0.0, v0 + C::kAMax * dt);
      w = w0 - std::copysign(std::min(std::abs(w0), C::kAlphaMax * dt), w0);
    }
    engine_->commitDwa(v, w);
  }

  ++counters_.ticks;
  if (r.new_decision) {
    ++counters_.decisions;
    if (r.source == Source::kDwa) {
      ++counters_.dwa[static_cast<std::size_t>(r.dwa_reason)];
    } else if (r.source == Source::kFallbackCell) {
      ++counters_.fallback_cell;
    } else if (r.source == Source::kAlign) {
      ++counters_.align;
    }
    if (r.invalid_action) {
      ++counters_.invalid_action;
    }
    if (r.source == Source::kDwa) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000, "RLController: DWA 폴백 (사유 %d): %s",
        static_cast<int>(r.dwa_reason), r.error.c_str());
    }
  }
  r.v = v;
  r.w = w;
  last_ = r;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.header.stamp = clock_->now();
  cmd.twist.linear.x = v;
  cmd.twist.angular.z = w;

  const double cycle_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  publishStats(r, cycle_ms, in);
  const rclcpp::Time now = clock_->now();
  if ((now - last_summary_).seconds() >= 10.0) {
    last_summary_ = now;
    const auto & c = counters_;
    RCLCPP_INFO(
      logger_,
      "RLController: 틱 %zu, 결정 %zu, DWA 폴백(입력 %zu, 오류 %zu, 시간초과 %zu, 비유한 %zu), "
      "계약 폴백 셀 %zu, 마스크 밖 %zu, 정렬 %zu",
      c.ticks, c.decisions, c.dwa[1], c.dwa[2], c.dwa[3], c.dwa[4], c.fallback_cell,
      c.invalid_action, c.align);
  }
  return cmd;
}

void RLController::publishStats(const TickResult & r, double cycle_ms, const TickInput & in)
{
  if (!stats_pub_ || !stats_pub_->is_activated()) {
    return;
  }
  std_msgs::msg::Float64MultiArray st;
  const auto & c = counters_;
  std::size_t dwa_total = 0;
  for (auto n : c.dwa) {
    dwa_total += n;
  }
  st.data = {cycle_ms, r.obs_ms, r.mask_ms, r.infer_ms, static_cast<double>(r.source),
    static_cast<double>(r.dwa_reason), static_cast<double>(r.action),
    static_cast<double>(r.model_action), static_cast<double>(r.n_valid),
    r.saturated ? 1.0 : 0.0, r.v, r.w, r.new_decision ? 1.0 : 0.0,
    static_cast<double>(c.decisions), static_cast<double>(dwa_total),
    static_cast<double>(c.fallback_cell), static_cast<double>(c.invalid_action),
    static_cast<double>(c.dwa[static_cast<std::size_t>(DwaReason::kTimeout)]),
    in.state.scan_age, in.state.track_age, static_cast<double>(in.state.tracks.size()),
    in.state.v_cap, engine_->dev().in_dev ? 1.0 : 0.0, engine_->dev().peak};
  stats_pub_->publish(st);
}

RLController::Counters RLController::counters() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return counters_;
}

TickResult RLController::lastResult() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_;
}

}  // namespace amr_rl_controller

PLUGINLIB_EXPORT_CLASS(amr_rl_controller::RLController, nav2_core::Controller)
