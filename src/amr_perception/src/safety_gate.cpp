// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 안전 게이트 순수 로직 구현.

#include "amr_perception/safety_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace amr_perception
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

/// 등속 원호(유니사이클) 운동 후 자세 (base_link 기준 상대 변위)
Pose2D unicycleDelta(double v, double w, double t)
{
  if (std::abs(w) < 1e-6) {
    return Pose2D{v * t, 0.0, 0.0};
  }
  const double th = w * t;
  return Pose2D{v / w * std::sin(th), v / w * (1.0 - std::cos(th)), th};
}
}  // namespace

const char * zoneName(SafetyZone zone)
{
  switch (zone) {
    case SafetyZone::kClear:
      return "CLEAR";
    case SafetyZone::kWarning:
      return "WARNING";
    case SafetyZone::kCritical:
      return "CRITICAL";
    case SafetyZone::kStop:
      return "STOP";
  }
  return "UNKNOWN";
}

SafetyGate::SafetyGate(
  const SafetyParams & params, std::vector<SensorWatch> sensors,
  double start_time)
: params_(params), sensors_(std::move(sensors))
{
  // 기동 직후에는 아직 한 번도 안 온 센서를 "기동 시각에 봤다" 고 두어 타임아웃만큼 유예한다.
  for (const auto & s : sensors_) {
    last_seen_[s.name] = start_time;
  }
}

double SafetyGate::footprintDistance(const Vec2 & p) const
{
  return distanceToRectangle(p, params_.footprint_length, params_.footprint_width);
}

double SafetyGate::circumscribedRadius() const
{
  return 0.5 * std::hypot(params_.footprint_length, params_.footprint_width);
}

double SafetyGate::clearanceSpeedLimit(double distance, const SafetyParams & p)
{
  if (!std::isfinite(distance)) {
    return kInf;
  }
  // d(v) = v·t + v²/(2a) <= D - d_stop 의 양의 근
  const double margin = std::max(distance - p.emergency_stop_distance, 0.0);
  const double at = p.max_deceleration * p.reaction_latency;
  return -at + std::sqrt(at * at + 2.0 * p.max_deceleration * margin);
}

SafetyZone SafetyGate::classifyZone(double distance, const SafetyParams & p)
{
  if (distance <= p.emergency_stop_distance) {
    return SafetyZone::kStop;
  }
  if (distance <= p.critical_zone_distance) {
    return SafetyZone::kCritical;
  }
  if (distance <= p.warning_zone_distance) {
    return SafetyZone::kWarning;
  }
  return SafetyZone::kClear;
}

void SafetyGate::setScanPoints(const std::vector<Vec2> & points, double stamp)
{
  scan_points_ = points;
  scan_stamp_ = stamp;
}

void SafetyGate::sensorHeartbeat(const std::string & name, double stamp)
{
  auto it = last_seen_.find(name);
  if (it != last_seen_.end()) {
    it->second = std::max(it->second, stamp);
  }
}

void SafetyGate::setCommand(double linear, double angular, double stamp)
{
  input_.linear = std::isfinite(linear) ? linear : 0.0;
  input_.angular = std::isfinite(angular) ? angular : 0.0;
  input_stamp_ = stamp;
}

void SafetyGate::setMinTtc(double ttc, double stamp)
{
  min_ttc_ = std::isnan(ttc) ? kInf : ttc;
  ttc_stamp_ = stamp;
}

bool SafetyGate::allSourcesClear() const
{
  return std::none_of(
    estop_sources_.begin(), estop_sources_.end(),
    [](const auto & kv) {return kv.second;});
}

void SafetyGate::releaseIfArmed(double now)
{
  if (!estop_latched_ || !allSourcesClear()) {
    return;
  }
  if (!params_.estop_release_requires_reset || now <= reset_armed_until_) {
    estop_latched_ = false;
    reset_armed_until_ = -1.0;
  }
}

void SafetyGate::setEstopSource(const std::string & source, bool active, double now)
{
  estop_sources_[source] = active;
  if (active) {
    estop_latched_ = true;
    reset_armed_until_ = -1.0;  // 해제 대기 중이던 reset 은 새 E-stop 으로 무효
    return;
  }
  releaseIfArmed(now);
}

ResetResult SafetyGate::requestReset(double now)
{
  if (!estop_latched_) {
    return ResetResult{true, "E-stop 래치 없음"};
  }
  if (allSourcesClear()) {
    estop_latched_ = false;
    reset_armed_until_ = -1.0;
    return ResetResult{true, "E-stop 래치 해제"};
  }
  // 대시보드는 false 발행 직후 reset 을 부른다 — 토픽보다 서비스가 먼저 도착하는 경우를 위해
  // reset_grace 동안 false 를 기다렸다가 해제한다.
  reset_armed_until_ = now + params_.reset_grace;
  std::string active;
  for (const auto & kv : estop_sources_) {
    if (kv.second) {
      active += (active.empty() ? "" : ", ") + kv.first;
    }
  }
  return ResetResult{
    false, "E-stop 입력이 아직 true (" + active + ") — " +
    std::to_string(params_.reset_grace).substr(0, 4) + " s 안에 false 가 오면 해제"};
}

void SafetyGate::setExclusionPolygon(const std::vector<Vec2> & polygon, double stamp)
{
  exclusion_ = polygon;
  exclusion_stamp_ = stamp;
}

SafetyGate::Distances SafetyGate::computeDistances(double now, double v, double w) const
{
  Distances d;
  const bool excl_valid = exclusion_.size() >= 3 && exclusion_stamp_ >= 0.0 &&
    now - exclusion_stamp_ <= params_.exclusion_timeout;
  const double inner_l = params_.footprint_length - 2.0 * params_.self_filter_margin;
  const double inner_w = params_.footprint_width - 2.0 * params_.self_filter_margin;
  const bool moving_linear = std::abs(v) > params_.motion_epsilon_v;
  const bool rotating = !moving_linear && std::abs(w) > params_.motion_epsilon_w;
  const double half_l = 0.5 * params_.footprint_length;
  const double r_circ = circumscribedRadius();

  for (const auto & p : scan_points_) {
    if (std::abs(p.x()) < 0.5 * inner_l && std::abs(p.y()) < 0.5 * inner_w) {
      continue;  // 자기 차체(풋프린트 안쪽) 반사
    }
    const double dist = footprintDistance(p);
    if (excl_valid && pointInPolygon(p, exclusion_)) {
      d.exclusion = std::min(d.exclusion, dist);
      d.exclusion_present = true;
      continue;
    }
    if (params_.zone_region == ZoneRegion::kOmni) {
      d.zone = std::min(d.zone, dist);
      continue;
    }
    // kMotion: 진행 방향 보호 영역
    if (moving_linear) {
      const double ahead = (v > 0.0 ? p.x() : -p.x());
      if (ahead > half_l) {
        d.zone = std::min(d.zone, dist);
      } else {
        d.lateral = std::min(d.lateral, dist);
      }
    } else if (rotating) {
      // 제자리 회전: 꼭짓점이 쓸고 지나가는 외접원 기준
      d.zone = std::min(d.zone, std::max(p.norm() - r_circ, 0.0));
    } else {
      d.zone = std::min(d.zone, dist);
    }
  }
  return d;
}

double SafetyGate::minDistanceAfterMotion(double v, double w, double horizon) const
{
  // 명령을 horizon 동안 유지했을 때(중간 시각 포함) 풋프린트-장애물 최소 거리
  double best = kInf;
  const int steps = 5;
  for (int i = 1; i <= steps; ++i) {
    const Pose2D inv = unicycleDelta(v, w, horizon * i / steps).inverse();
    for (const auto & p : scan_points_) {
      best = std::min(best, footprintDistance(inv.apply(p)));
    }
  }
  return best;
}

SafetyStatus SafetyGate::evaluate(double now)
{
  SafetyStatus st;
  releaseIfArmed(now);
  st.estop_latched = estop_latched_;

  // 센서 타임아웃
  for (const auto & s : sensors_) {
    const double age = now - last_seen_[s.name];
    if (age > s.timeout) {
      st.failed_sensors.push_back(s.name);
      if (s.action == SensorFailureAction::kStop) {
        st.sensor_stop = true;
      } else {
        st.degraded = true;
      }
    }
  }

  // 입력 명령
  const bool cmd_fresh = input_stamp_ >= 0.0 && now - input_stamp_ <= params_.command_timeout;
  st.command_stale = !cmd_fresh;
  const double v_in = cmd_fresh ? std::clamp(
    input_.linear, params_.min_linear_velocity, params_.max_linear_velocity) : 0.0;
  const double w_in = cmd_fresh ? std::clamp(
    input_.angular, -params_.max_angular_velocity, params_.max_angular_velocity) : 0.0;

  // 거리와 근접 정지 (히스테리시스)
  const Distances d = computeDistances(now, v_in, w_in);
  st.min_distance = d.zone;
  const bool trigger = d.zone <= params_.emergency_stop_distance ||
    d.lateral <= params_.lateral_stop_distance ||
    d.exclusion <= params_.exclusion_stop_distance;
  if (trigger) {
    proximity_stop_ = true;
  } else if (proximity_stop_) {
    const double h = params_.zone_hysteresis;
    if (d.zone > params_.stop_release_distance &&
      d.lateral > params_.lateral_stop_distance + h &&
      d.exclusion > params_.exclusion_stop_distance + h)
    {
      proximity_stop_ = false;
    }
  }
  st.proximity_stop = proximity_stop_;

  // 존 (완화 방향만 히스테리시스)
  if (proximity_stop_) {
    zone_ = SafetyZone::kStop;
  } else {
    const SafetyZone raw = classifyZone(d.zone, params_);
    if (raw == SafetyZone::kStop) {
      zone_ = SafetyZone::kCritical;  // 트리거 조건과 경계값 차이 방어 (도달 불가에 가깝다)
    } else if (static_cast<int>(raw) >= static_cast<int>(zone_) || zone_ == SafetyZone::kStop) {
      zone_ = raw;
    } else {
      const double h = params_.zone_hysteresis;
      const double boundary = zone_ == SafetyZone::kCritical ?
        params_.critical_zone_distance : params_.warning_zone_distance;
      if (d.zone > boundary + h) {
        zone_ = raw;
      }
    }
  }
  st.zone = zone_;

  // 거리 기반 상한 (선속도·꼭짓점 속도 공통)
  double v_dist = params_.max_linear_velocity;
  if (zone_ == SafetyZone::kWarning) {
    v_dist = std::min(v_dist, params_.warning_zone_max_speed);
  } else if (zone_ == SafetyZone::kCritical) {
    v_dist = std::min(v_dist, params_.critical_zone_max_speed);
  }
  if (params_.clearance_speed_limit_enabled) {
    v_dist = std::min(v_dist, clearanceSpeedLimit(d.zone, params_));
  }
  if (d.exclusion_present) {
    v_dist = std::min(v_dist, params_.critical_zone_max_speed);
  }
  if (st.degraded) {
    v_dist = std::min(v_dist, params_.degraded_mode_max_speed);
  }
  // TTC 기반 연속 감속 (선속도만)
  double v_lim = v_dist;
  const bool ttc_fresh = ttc_stamp_ >= 0.0 && now - ttc_stamp_ <= params_.ttc_max_age;
  st.min_ttc = ttc_fresh ? min_ttc_ : kInf;
  bool ttc_limited = false;
  if (params_.ttc_limit_enabled && ttc_fresh && min_ttc_ <= params_.ttc_critical) {
    const double v_ttc = std::max(
      0.0, params_.max_deceleration * (min_ttc_ - params_.reaction_latency));
    if (v_ttc < v_lim) {
      v_lim = v_ttc;
      ttc_limited = true;
    }
  }
  const double w_lim = std::min(params_.max_angular_velocity, v_dist / circumscribedRadius());
  st.speed_limit = v_lim;
  st.angular_limit = w_lim;

  // 명령 결정
  double v = v_in;
  double w = w_in;
  if (st.estop_latched) {
    st.reasons.emplace_back("estop_latched");
  }
  if (st.sensor_stop) {
    st.reasons.emplace_back("sensor_failure_stop");
  }
  if (st.degraded) {
    st.reasons.emplace_back("sensor_failure_degraded");
  }
  if (st.command_stale) {
    st.reasons.emplace_back("command_stale");
  }
  if (ttc_limited) {
    st.reasons.emplace_back("ttc_limit");
  }
  if (st.estop_latched || st.sensor_stop || st.command_stale) {
    v = 0.0;
    w = 0.0;
  } else if (proximity_stop_) {
    st.reasons.emplace_back("proximity_stop");
    const bool moving = std::abs(v_in) > params_.motion_epsilon_v ||
      std::abs(w_in) > params_.motion_epsilon_w;
    bool escape = false;
    if (params_.allow_escape && moving && params_.zone_region == ZoneRegion::kOmni) {
      const double cur = std::min({d.zone, d.lateral, d.exclusion});
      const double after = minDistanceAfterMotion(v_in, w_in, params_.escape_horizon);
      escape = after > cur + 1e-3;
    }
    if (escape) {
      st.escaping = true;
      st.reasons.emplace_back("escape");
      const double cap = params_.critical_zone_max_speed;
      const double s = std::min(
        {1.0, std::abs(v) > cap ? cap / std::abs(v) : 1.0,
          std::abs(w) * circumscribedRadius() > cap ? cap / (std::abs(w) * circumscribedRadius()) :
          1.0});
      v *= s;
      w *= s;
    } else {
      v = 0.0;
      w = 0.0;
    }
  } else {
    // 곡률을 유지하도록 같은 비율로 축소
    const double sv = std::abs(v) > v_lim ? v_lim / std::abs(v) : 1.0;
    const double sw = std::abs(w) > w_lim ? w_lim / std::abs(w) : 1.0;
    const double s = std::min(sv, sw);
    v *= s;
    w *= s;
  }
  if (zone_ == SafetyZone::kWarning) {
    st.reasons.emplace_back("warning_zone");
  } else if (zone_ == SafetyZone::kCritical) {
    st.reasons.emplace_back("critical_zone");
  }
  st.command.linear = v;
  st.command.angular = w;
  st.estop_active = st.estop_latched || proximity_stop_ || st.sensor_stop;
  return st;
}

}  // namespace amr_perception
