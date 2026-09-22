// 마커 기반 정밀 도킹 제어기 구현 (docking_controller.hpp 설명, docs/algorithms/docking.md 유도).
#include "amr_behavior/docking/docking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

namespace amr_behavior
{
namespace docking
{

namespace
{
double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double clampAbs(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}
}  // namespace

const char * phaseName(Phase phase)
{
  switch (phase) {
    case Phase::kSearch: return "search";
    case Phase::kAlign: return "align";
    case Phase::kApproach: return "approach";
    case Phase::kFinal: return "final";
    case Phase::kBackup: return "backup";
    case Phase::kSucceeded: return "succeeded";
    case Phase::kFailed: return "failed";
    case Phase::kIdle: break;
  }
  return "idle";
}

std::optional<ControlLaw> parseControlLaw(const std::string & name)
{
  if (name == "proportional") {
    return ControlLaw::kProportional;
  }
  if (name == "graceful") {
    return ControlLaw::kGraceful;
  }
  return std::nullopt;
}

// ---------------------------------------------------------------- 오차
double DockErrors::position() const
{
  return std::hypot(longitudinal, lateral);
}

bool DockErrors::valid() const
{
  return std::isfinite(longitudinal) && std::isfinite(lateral) && std::isfinite(heading);
}

DockErrors computeErrors(const MarkerObservation & m, double standoff)
{
  // 목표 base_link 자세 (로봇 현재 프레임): 마커 중심에서 바깥 법선으로 standoff, 방위는 마커를
  // 마주 봄
  const double tx = m.x + standoff * std::cos(m.normal_yaw);
  const double ty = m.y + standoff * std::sin(m.normal_yaw);
  const double target_yaw = wrap(m.normal_yaw + M_PI);
  // 로봇(원점, 방위 0)을 도크 프레임 D 로: r_D = R(−target_yaw)·(0 − t)
  const double c = std::cos(target_yaw);
  const double s = std::sin(target_yaw);
  const double x_r = -tx * c - ty * s;
  const double y_r = tx * s - ty * c;
  DockErrors e;
  e.longitudinal = -x_r;
  e.lateral = y_r;
  e.heading = wrap(-target_yaw);
  return e;
}

// ---------------------------------------------------------------- 추적
void MarkerTracker::reset()
{
  has_ = false;
  last_obs_ = -1.0;
}

void MarkerTracker::correct(const MarkerObservation & obs, double now, double filter_coef)
{
  if (!has_ || filter_coef >= 1.0) {
    est_ = obs;
  } else {
    const double a = std::clamp(filter_coef, 0.0, 1.0);
    est_.x += a * (obs.x - est_.x);
    est_.y += a * (obs.y - est_.y);
    est_.normal_yaw = wrap(est_.normal_yaw + a * wrap(obs.normal_yaw - est_.normal_yaw));
  }
  has_ = true;
  last_obs_ = now;
}

void MarkerTracker::predict(double linear, double angular, double dt)
{
  if (!has_ || !(dt > 0.0)) {
    return;
  }
  // 로봇 이동 (중점 적분): Δθ = ω·dt, Δp = v·dt·(cos Δθ/2, sin Δθ/2)
  const double dth = angular * dt;
  const double dx = linear * dt * std::cos(0.5 * dth);
  const double dy = linear * dt * std::sin(0.5 * dth);
  // 고정된 마커를 새 로봇 프레임으로: m' = R(−Δθ)·(m − Δp)
  const double px = est_.x - dx;
  const double py = est_.y - dy;
  const double c = std::cos(dth);
  const double s = std::sin(dth);
  est_.x = c * px + s * py;
  est_.y = -s * px + c * py;
  est_.normal_yaw = wrap(est_.normal_yaw - dth);
}

double MarkerTracker::age(double now) const
{
  return last_obs_ < 0.0 ? std::numeric_limits<double>::infinity() : now - last_obs_;
}

// ---------------------------------------------------------------- 제어기
DockingController::DockingController(const Params & params)
: params_(params)
{
}

void DockingController::start(double now, int max_attempts)
{
  max_attempts_ = std::max(1, max_attempts);
  attempt_ = 1;
  tracker_.reset();
  errors_ = DockErrors{};
  failure_reason_.clear();
  attempt_start_ = now;
  last_update_ = -1.0;
  last_cmd_ = Command{};
  settle_count_ = 0;
  lateral_stall_count_ = 0;
  arrived_ = false;
  heading_arrived_ = false;
  enterPhase(Phase::kSearch, now);
}

void DockingController::cancel()
{
  failure_reason_ = "canceled";
  phase_ = Phase::kFailed;
  last_cmd_ = Command{};
}

void DockingController::enterPhase(Phase phase, double now)
{
  phase_ = phase;
  phase_start_ = now;
}

Command DockingController::stop()
{
  last_cmd_ = Command{};
  return last_cmd_;
}

void DockingController::failAttempt(double now, const std::string & reason)
{
  failure_reason_ = reason;
  settle_count_ = 0;
  lateral_stall_count_ = 0;
  arrived_ = false;
  heading_arrived_ = false;
  enterPhase(attempt_ < max_attempts_ ? Phase::kBackup : Phase::kFailed, now);
}

bool DockingController::inTolerance(const DockErrors & e) const
{
  return e.valid() && e.position() <= params_.position_tolerance &&
         std::fabs(e.heading) <= params_.angle_tolerance;
}

Command DockingController::proportional(const DockErrors & e, double speed_cap) const
{
  Command cmd;
  // 주시점: 목표점을 직접 겨눈다(L = e_x → 횡오차가 남은 거리에 비례해 0 으로). L 하한은 특이점
  // 회피용. 도달 후 남는 방위 오차는 final 단계의 제자리 회전(finalAlign)이 없앤다.
  const double lookahead = std::max(e.longitudinal, params_.lookahead);
  const double heading_ref = std::atan2(-e.lateral, lookahead);
  const double heading_err = wrap(heading_ref - e.heading);
  if (std::fabs(heading_err) > params_.angular_deadband) {
    cmd.angular = clampAbs(params_.k_heading * heading_err, params_.max_angular_speed);
  }
  if (std::fabs(e.longitudinal) > params_.linear_deadband) {
    const double v = clampAbs(params_.k_distance * e.longitudinal, speed_cap);
    cmd.linear = v * std::max(0.0, std::cos(heading_err));
  }
  return cmd;
}

Command DockingController::graceful(const DockErrors & e, double speed_cap) const
{
  // Park & Kuipers (2011): 목표를 로봇 중심 극좌표 (r, φ, δ) 로 두고 곡률 κ 를 정한다.
  const double dx = e.longitudinal;
  const double dy = -e.lateral;
  const double r = std::hypot(dx, dy);
  if (r < 1e-6) {
    return Command{};
  }
  const double los = std::atan2(dy, dx);
  const double phi = wrap(-los);
  const double delta = wrap(e.heading - los);
  const double kphi_phi = params_.k_phi * phi;
  const double kappa = -1.0 / r *
    (params_.k_delta * (delta - std::atan(-kphi_phi)) +
    (1.0 + params_.k_phi / (1.0 + kphi_phi * kphi_phi)) * std::sin(delta));
  double v = speed_cap / (1.0 + params_.beta * std::pow(std::fabs(kappa), params_.lambda));
  v = std::min(speed_cap * r / params_.slowdown_radius, v);
  v = std::clamp(v, std::min(params_.min_linear_speed, speed_cap), speed_cap);
  const double w = std::clamp(kappa * v, -params_.max_angular_speed, params_.max_angular_speed);
  if (kappa != 0.0) {
    v = w / kappa;   // ω 포화 시에도 곡률을 지킨다 (Nav2 graceful 과 같음)
  }
  return Command{v, w};
}

Command DockingController::control(const DockErrors & e, double speed_cap) const
{
  return params_.law == ControlLaw::kGraceful ? graceful(e, speed_cap) : proportional(e, speed_cap);
}

Command DockingController::finalAlign(const DockErrors & e) const
{
  Command cmd;
  if (std::fabs(e.heading) > params_.angular_deadband) {
    cmd.angular = clampAbs(-params_.k_heading * e.heading, params_.max_angular_speed);
  }
  return cmd;
}

Command DockingController::update(double now, const std::optional<MarkerObservation> & obs)
{
  if (phase_ == Phase::kIdle || finished()) {
    return stop();
  }
  const double dt = last_update_ < 0.0 ? 0.0 : std::clamp(now - last_update_, 0.0, 0.5);
  last_update_ = now;
  tracker_.predict(last_cmd_.linear, last_cmd_.angular, dt);
  if (obs) {
    tracker_.correct(*obs, now, params_.filter_coef);
  }
  if (tracker_.hasEstimate()) {
    errors_ = computeErrors(tracker_.estimate(), params_.standoff);
  }

  if (phase_ == Phase::kBackup) {
    if (now - phase_start_ >= params_.backup_distance / params_.backup_speed) {
      ++attempt_;
      attempt_start_ = now;
      enterPhase(Phase::kSearch, now);
      return stop();
    }
    last_cmd_ = Command{-params_.backup_speed, 0.0};
    return last_cmd_;
  }

  if (now - attempt_start_ > params_.attempt_timeout) {
    failAttempt(now, "attempt_timeout");
    return stop();
  }

  const bool fresh = obs.has_value();
  if (phase_ == Phase::kSearch) {
    if (!fresh) {
      if (now - phase_start_ > params_.search_timeout) {
        failAttempt(now, "search_timeout");
        return stop();
      }
      // ±search_sweep 삼각파 탐색 (시작 방향 +)
      const double half = params_.search_sweep / params_.search_angular_speed;
      const double tau = std::fmod(now - phase_start_, 4.0 * half);
      const double dir = (tau < half || tau >= 3.0 * half) ? 1.0 : -1.0;
      last_cmd_ = Command{0.0, dir * params_.search_angular_speed};
      return last_cmd_;
    }
    enterPhase(Phase::kApproach, now);
  }

  if (tracker_.age(now) > params_.marker_timeout || !errors_.valid()) {
    failAttempt(now, "marker_lost");
    return stop();
  }
  const DockErrors & e = errors_;
  if (e.longitudinal < -params_.max_overshoot) {
    failAttempt(now, "overshoot");
    return stop();
  }

  // 종방향 도달 (히스테리시스): 허용오차 경계(2 cm)에서 멈추지 않고 목표점 근처까지 들어간다
  if (std::fabs(e.longitudinal) <= params_.stop_distance) {
    arrived_ = true;
  } else if (std::fabs(e.longitudinal) > params_.stop_distance + 2.0 * params_.linear_deadband) {
    arrived_ = false;
  }

  // 방위 도달 (히스테리시스, 종방향 도달 뒤에만): 1° 경계가 아니라 heading_stop_tolerance 까지
  // 제자리 정렬한 뒤 판정한다 (접근 중 우연히 작았던 방위로 도달 처리하지 않는다)
  if (!arrived_) {
    heading_arrived_ = false;
  } else if (std::fabs(e.heading) <= params_.heading_stop_tolerance) {
    heading_arrived_ = true;
  } else if (std::fabs(e.heading) > params_.angle_tolerance) {
    heading_arrived_ = false;
  }

  // 판정: 도달 후 허용오차 안에서 신선한 관측이 settle_frames 연속이면 성공 (그동안 정지 유지)
  if (arrived_ && heading_arrived_ && inTolerance(e)) {
    lateral_stall_count_ = 0;
    if (tracker_.age(now) <= 2.0 * params_.control_period) {
      ++settle_count_;
    }
    if (settle_count_ >= params_.settle_frames) {
      enterPhase(Phase::kSucceeded, now);
    } else if (phase_ != Phase::kFinal) {
      enterPhase(Phase::kFinal, now);
    }
    return stop();
  }
  settle_count_ = 0;

  // 진행 방향 오차가 크면 제자리 정렬 (히스테리시스 절반에서 해제)
  const double lookahead = std::max(e.longitudinal, params_.lookahead);
  const double heading_err = wrap(std::atan2(-e.lateral, lookahead) - e.heading);
  const bool far = e.longitudinal > params_.final_distance;
  if (far) {
    if (phase_ == Phase::kAlign) {
      if (std::fabs(heading_err) < 0.5 * params_.align_threshold) {
        enterPhase(Phase::kApproach, now);
      }
    } else if (std::fabs(heading_err) > params_.align_threshold) {
      enterPhase(Phase::kAlign, now);
    }
    if (phase_ == Phase::kAlign) {
      last_cmd_ =
        Command{0.0, clampAbs(params_.k_heading * heading_err, params_.max_angular_speed)};
      return last_cmd_;
    }
    if (phase_ != Phase::kApproach) {
      enterPhase(Phase::kApproach, now);
    }
    last_cmd_ = control(e, params_.max_linear_speed);
    return last_cmd_;
  }

  if (phase_ != Phase::kFinal) {
    enterPhase(Phase::kFinal, now);
  }
  if (arrived_) {
    // 종방향 도달: 제자리에서 방위만 맞춘다. 횡오차가 남아 있으면 이 자세로는 고칠 수 없다 →
    // 재시도.
    if (e.position() > params_.position_tolerance) {
      if (++lateral_stall_count_ > 2 * params_.settle_frames) {
        failAttempt(now, "lateral");
        return stop();
      }
    }
    last_cmd_ = finalAlign(e);
    return last_cmd_;
  }
  lateral_stall_count_ = 0;
  if (e.longitudinal < 0.0) {
    // 목표를 지나침: 방위 유지하며 천천히 후진
    last_cmd_ = Command{
      clampAbs(params_.k_distance * e.longitudinal, params_.final_linear_speed), 0.0};
    return last_cmd_;
  }
  last_cmd_ = control(e, params_.final_linear_speed);
  return last_cmd_;
}

}  // namespace docking
}  // namespace amr_behavior
