// 작업 실행기 공유 상태 구현 (executor_context.hpp 설명 참고).
#include "amr_behavior/executor_context.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace amr_behavior
{

namespace phase
{
bool isKnown(const std::string & name)
{
  static const std::vector<std::string> kAll = {
    kIdle, kMoving, kPerceiving, kDocking, kLoading, kUnloading, kReturning, kCharging, kError,
    kRecovering, kUndocking};
  return std::find(kAll.begin(), kAll.end(), name) != kAll.end();
}
}  // namespace phase

namespace
{
std::string toLower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

bool finitePose(const geometry_msgs::msg::PoseStamped & p)
{
  const auto & q = p.pose.orientation;
  return std::isfinite(p.pose.position.x) && std::isfinite(p.pose.position.y) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

/// 양보 자세가 hold 보다 조금 먼저 올 수 있어(교통 관리자 발행 순서) 이만큼은 hold 이전 것도
/// 인정한다.
constexpr double kYieldPoseGrace = 2.0;   // [s]
}  // namespace

ExecutorContext::ExecutorContext(Clock clock, ExecutorHooks hooks)
: clock_(std::move(clock)), hooks_(std::move(hooks))
{
}

double ExecutorContext::now() const
{
  return clock_ ? clock_() : 0.0;
}

void ExecutorContext::setHooks(ExecutorHooks hooks)
{
  std::lock_guard<std::mutex> lock(mutex_);
  hooks_ = std::move(hooks);
}

// ------------------------------------------------------------------ 설정
void ExecutorContext::setRobotId(const std::string & robot_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  robot_id_ = robot_id;
}

std::string ExecutorContext::robotId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_id_;
}

void ExecutorContext::setAcceptPolicy(const AcceptPolicy & policy)
{
  std::lock_guard<std::mutex> lock(mutex_);
  policy_ = policy;
}

void ExecutorContext::setPayloadTable(const std::map<std::string, PayloadSpec> & table)
{
  std::lock_guard<std::mutex> lock(mutex_);
  payloads_ = table;
}

std::optional<PayloadSpec> ExecutorContext::payloadSpec(const std::string & item_type) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = payloads_.find(toLower(item_type));
  if (it == payloads_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void ExecutorContext::setDocks(
  const std::vector<DockSpec> & docks, double position_tolerance,
  double yaw_tolerance)
{
  std::lock_guard<std::mutex> lock(mutex_);
  docks_ = docks;
  dock_pos_tol_ = position_tolerance;
  dock_yaw_tol_ = yaw_tolerance;
}

double ExecutorContext::yawOf(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string ExecutorContext::resolveDock(const geometry_msgs::msg::PoseStamped & pose) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return resolveDockLocked(pose);
}

std::string ExecutorContext::resolveDockLocked(const geometry_msgs::msg::PoseStamped & pose) const
{
  const double yaw = yawOf(pose);
  std::string best;
  double best_dist = std::numeric_limits<double>::infinity();
  for (const auto & d : docks_) {
    const double dist = std::hypot(pose.pose.position.x - d.x, pose.pose.position.y - d.y);
    const double dyaw = std::fabs(wrapAngle(yaw - d.yaw));
    const bool yaw_ok = dock_yaw_tol_ >= M_PI || dyaw <= dock_yaw_tol_;
    if (dist <= dock_pos_tol_ && yaw_ok && dist < best_dist) {
      best = d.id;
      best_dist = dist;
    }
  }
  return best;
}

std::optional<DockSpec> ExecutorContext::dock(const std::string & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & d : docks_) {
    if (d.id == id) {
      return d;
    }
  }
  return std::nullopt;
}

// ------------------------------------------------------------------ 입력 상태
void ExecutorContext::updateBattery(double percent)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (std::isfinite(percent)) {
    battery_ = std::clamp(percent, 0.0, 100.0);
  } else {
    battery_.reset();
  }
}

std::optional<double> ExecutorContext::battery() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return battery_;
}

void ExecutorContext::updateEstop(bool active)
{
  std::lock_guard<std::mutex> lock(mutex_);
  estop_ = active;
}

bool ExecutorContext::estopActive() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return estop_;
}

void ExecutorContext::updateTrafficHold(bool hold)
{
  const double t = now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (hold && !traffic_hold_) {
    hold_since_ = t;
  }
  if (!hold) {
    yield_pose_.reset();   // 해제되면 양보 자세도 무효
  }
  traffic_hold_ = hold;
}

bool ExecutorContext::trafficHold() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return traffic_hold_;
}

void ExecutorContext::updateYieldPose(const geometry_msgs::msg::PoseStamped & pose)
{
  const double t = now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (pose.header.frame_id.empty() || !finitePose(pose)) {
    return;
  }
  yield_pose_ = pose;
  yield_stamp_ = t;
}

std::optional<geometry_msgs::msg::PoseStamped> ExecutorContext::yieldPose() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!traffic_hold_ || !yield_pose_ || yield_stamp_ < hold_since_ - kYieldPoseGrace) {
    return std::nullopt;
  }
  return yield_pose_;
}

void ExecutorContext::updateLocalizationLost(bool lost)
{
  std::lock_guard<std::mutex> lock(mutex_);
  localization_lost_ = lost;
}

bool ExecutorContext::localizationLost() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return localization_lost_;
}

void ExecutorContext::updateDetectedObjects(const amr_msgs::msg::DetectedObjectArray & objects)
{
  const double t = now();
  std::lock_guard<std::mutex> lock(mutex_);
  objects_ = objects;
  objects_stamp_ = t;
}

bool ExecutorContext::objectDetected(const ObjectQuery & query) const
{
  const double t = now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (objects_stamp_ < 0.0 || t - objects_stamp_ > query.max_age) {
    return false;
  }
  const std::string wanted = toLower(query.class_name);
  for (const auto & o : objects_.objects) {
    const bool class_ok = wanted.empty() || toLower(o.class_name) == wanted;
    const bool conf_ok = o.confidence >= query.min_confidence;
    const bool dist_ok = std::isfinite(o.distance) && o.distance <= query.max_distance;
    if (class_ok && conf_ok && dist_ok) {
      return true;
    }
  }
  return false;
}

void ExecutorContext::updateDockMarker(const geometry_msgs::msg::PoseStamped & pose)
{
  const double t = now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (finitePose(pose)) {
    marker_stamp_ = t;
  }
}

double ExecutorContext::dockMarkerAge() const
{
  const double t = now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (marker_stamp_ < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return t - marker_stamp_;
}

// ------------------------------------------------------------------ 작업
AcceptDecision ExecutorContext::evaluateTask(const amr_msgs::msg::Task & task) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (estop_) {
    return {false, "estop"};
  }
  if (localization_lost_) {
    return {false, "lost"};
  }
  if (task_) {
    return {false, "busy:" + task_->task_id};
  }
  if (!payload_.empty()) {
    // 하역 실패로 물품이 실린 채면 새 물품을 위에 싣지 않는다
    // (되돌려 놓거나 clear_payload 로 비운 뒤에 받는다)
    return {false, "blocked:payload"};
  }
  const bool idle = phase_.empty() || phase_ == phase::kIdle;
  const bool returning = phase_ == phase::kReturning && policy_.accept_while_returning;
  if (!idle && !returning) {
    return {false, "busy:" + toLower(phase_)};
  }
  if (battery_ && *battery_ < policy_.battery_low_percent) {
    return {false, "battery_low"};
  }
  if (task.task_id.empty()) {
    return {false, "invalid:task_id"};
  }
  if (!finitePose(task.pickup_pose) || !finitePose(task.dropoff_pose)) {
    return {false, "invalid:pose"};
  }
  if (!payloads_.empty() && payloads_.find(toLower(task.item_type)) == payloads_.end()) {
    return {false, "invalid:item_type"};
  }
  if (!policy_.allow_undocked_tasks) {
    // 등록 도크와 맞지 않는 자세를 조용히 도킹 생략으로 처리하지 않는다 (명세: 도킹 완료 후 적재)
    if (resolveDockLocked(task.pickup_pose).empty()) {
      return {false, "invalid:no_dock:pickup"};
    }
    if (resolveDockLocked(task.dropoff_pose).empty()) {
      return {false, "invalid:no_dock:dropoff"};
    }
  }
  return {true, "accepted"};
}

AcceptDecision ExecutorContext::acceptTask(const amr_msgs::msg::Task & task)
{
  AcceptDecision decision = evaluateTask(task);
  if (!decision.accepted) {
    return decision;
  }
  amr_msgs::msg::Task accepted = task;
  std::function<void(const amr_msgs::msg::Task &)> publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!robot_id_.empty()) {
      accepted.robot_id = robot_id_;
    }
    for (auto * p : {&accepted.pickup_pose, &accepted.dropoff_pose}) {
      if (p->header.frame_id.empty()) {
        p->header.frame_id = "map";
      }
    }
    accepted.status = amr_msgs::msg::Task::STATUS_IN_PROGRESS;
    task_ = accepted;
    last_failure_reason_.clear();
    publish = hooks_.publish_task_status;
  }
  if (publish) {
    publish(accepted);
  }
  logInfo("작업 수락: " + accepted.task_id + " (" + accepted.item_type + ")");
  return decision;
}

bool ExecutorContext::hasTask() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return task_.has_value();
}

std::optional<amr_msgs::msg::Task> ExecutorContext::currentTask() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return task_;
}

bool ExecutorContext::reportStatus(uint8_t status, const std::string & reason)
{
  amr_msgs::msg::Task msg;
  std::function<void(const amr_msgs::msg::Task &)> publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!task_) {
      return false;
    }
    task_->status = status;
    msg = *task_;
    publish = hooks_.publish_task_status;
    const bool terminal = status == amr_msgs::msg::Task::STATUS_COMPLETED ||
      status == amr_msgs::msg::Task::STATUS_FAILED;
    if (terminal) {
      last_failure_reason_ = status == amr_msgs::msg::Task::STATUS_FAILED ? reason : "";
      task_.reset();
      return_pending_ = true;
    }
  }
  if (publish) {
    publish(msg);
  }
  if (status == amr_msgs::msg::Task::STATUS_FAILED) {
    logWarn("작업 실패: " + msg.task_id + " (" + reason + ")");
  } else if (status == amr_msgs::msg::Task::STATUS_COMPLETED) {
    logInfo("작업 완료: " + msg.task_id);
  }
  return true;
}

std::string ExecutorContext::lastFailureReason() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_failure_reason_;
}

bool ExecutorContext::returnPending() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return return_pending_;
}

void ExecutorContext::clearReturnPending()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return_pending_ = false;
}

// ------------------------------------------------------------------ 출력
void ExecutorContext::setPhase(const std::string & name)
{
  std::function<void(const std::string &)> publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (name == phase_) {
      return;
    }
    phase_ = name;
    publish = hooks_.publish_phase;
  }
  if (publish) {
    publish(name);
  }
}

std::string ExecutorContext::phase() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return phase_;
}

void ExecutorContext::attachPayload(
  const std::string & item_type, double mass,
  const PayloadOrigin & origin)
{
  ExecutorHooks hooks;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    payload_ = item_type;
    payload_origin_ = origin;
    payload_origin_.item_type = item_type;
    payload_return_attempted_ = false;
    hooks = hooks_;
  }
  if (hooks.publish_payload_attach) {
    hooks.publish_payload_attach(item_type);
  }
  if (hooks.publish_payload_mass) {
    hooks.publish_payload_mass(mass);
  }
}

void ExecutorContext::detachPayload()
{
  attachPayload("", 0.0);
}

std::string ExecutorContext::attachedPayload() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return payload_;
}

std::optional<PayloadOrigin> ExecutorContext::strandedPayload() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (payload_.empty() || task_ || payload_return_attempted_) {
    return std::nullopt;
  }
  return payload_origin_;
}

void ExecutorContext::markPayloadReturnAttempted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  payload_return_attempted_ = !payload_.empty();
}

// ------------------------------------------------------------------ 충전소 할당
void ExecutorContext::setChargers(
  const std::vector<std::string> & ids, int robot_index,
  double claim_timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  charger_preference_.clear();
  const int n = static_cast<int>(ids.size());
  for (int i = 0; i < n; ++i) {
    charger_preference_.push_back(ids[static_cast<size_t>(((robot_index % n) + n + i) % n)]);
  }
  claim_timeout_ = claim_timeout;
}

std::vector<std::string> ExecutorContext::chargerPreference() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return charger_preference_;
}

void ExecutorContext::updateChargerClaim(
  const std::string & robot, const std::string & charger,
  double since)
{
  const double t = now();
  std::lock_guard<std::mutex> lock(mutex_);
  if (robot.empty() || robot == robot_id_) {
    return;
  }
  if (charger.empty()) {
    claims_.erase(robot);
    return;
  }
  claims_[robot] = ChargerClaim{charger, since, t};
}

std::optional<DockSpec> ExecutorContext::selectCharger(bool keep_current)
{
  const double t = now();
  std::function<void(const std::string &, double)> publish;
  std::optional<DockSpec> chosen;
  std::string claim;
  double since = 0.0;
  bool announce = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string previous = charger_claim_;
    // 다른 로봇이 (유효하게) 먼저 점유한 충전소인지
    auto taken = [&](const std::string & id, double my_since) {
        for (const auto & kv : claims_) {
          const ChargerClaim & c = kv.second;
          if (c.charger != id || t - c.heard > claim_timeout_) {
            continue;
          }
          if (c.since < my_since || (c.since == my_since && kv.first < robot_id_)) {
            return true;
          }
        }
        return false;
      };
    auto spec = [&](const std::string & id) -> std::optional<DockSpec> {
        for (const auto & d : docks_) {
          if (d.id == id) {
            return d;
          }
        }
        return std::nullopt;
      };
    const bool keep = !charger_claim_.empty() &&
      (keep_current || !taken(charger_claim_, charger_claim_since_));
    if (keep) {
      chosen = spec(charger_claim_);   // 자기 점유 유지 (E-stop 뒤 재개 포함)
    } else {
      charger_claim_.clear();
      for (const auto & id : charger_preference_) {
        if (taken(id, t)) {
          continue;
        }
        chosen = spec(id);
        if (chosen) {
          charger_claim_ = id;
          charger_claim_since_ = t;
          break;
        }
      }
    }
    claim = charger_claim_;
    since = charger_claim_since_;
    announce = claim != previous;   // 새 점유(또는 잃은 점유의 해제)를 바로 알린다
    publish = hooks_.publish_charger_claim;
  }
  if (publish && announce) {
    publish(claim, since);
  }
  return chosen;
}

void ExecutorContext::releaseCharger()
{
  std::function<void(const std::string &, double)> publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (charger_claim_.empty()) {
      return;
    }
    charger_claim_.clear();
    publish = hooks_.publish_charger_claim;
  }
  if (publish) {
    publish("", 0.0);
  }
}

std::string ExecutorContext::chargerClaim() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return charger_claim_;
}

double ExecutorContext::chargerClaimSince() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return charger_claim_since_;
}

void ExecutorContext::setCharging(bool enable)
{
  std::function<void(bool)> publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enable == charging_) {
      return;
    }
    charging_ = enable;
    publish = hooks_.publish_charging;
  }
  if (publish) {
    publish(enable);
  }
}

bool ExecutorContext::charging() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return charging_;
}

void ExecutorContext::logInfo(const std::string & msg) const
{
  std::function<void(const std::string &)> log;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    log = hooks_.log_info;
  }
  if (log) {
    log(msg);
  }
}

void ExecutorContext::logWarn(const std::string & msg) const
{
  std::function<void(const std::string &)> log;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    log = hooks_.log_warn;
  }
  if (log) {
    log(msg);
  }
}

}  // namespace amr_behavior
