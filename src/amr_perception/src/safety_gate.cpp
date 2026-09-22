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
constexpr double kTwoPi = 2.0 * M_PI;
/// |R| 가 이보다 크면 직선으로 본다 (영역 3 m 에서 가로 편차 < 0.05 mm)
constexpr double kStraightRadius = 1.0e5;
/// 탈출 판정: 가드 거리(중앙값)가 이만큼 줄어도 "멀어짐" 으로 본다 (중앙값 잡음 σ ≈ 0.01 m)
constexpr double kEscapeTolerance = 0.01;
constexpr int kEscapeSteps = 5;

/// 등속 원호(유니사이클) 운동 후 자세 (base_link 기준 상대 변위)
Pose2D unicycleDelta(double v, double w, double t)
{
  if (std::abs(w) < 1e-9) {
    return Pose2D{v * t, 0.0, 0.0};
  }
  const double th = w * t;
  return Pose2D{v / w * std::sin(th), v / w * (1.0 - std::cos(th)), th};
}

void appendUnique(std::vector<std::string> & dst, const std::vector<std::string> & src)
{
  for (const auto & s : src) {
    if (std::find(dst.begin(), dst.end(), s) == dst.end()) {
      dst.push_back(s);
    }
  }
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

// ---------------------------------------------------------------- 스윕 풋프린트

SweptFootprint::SweptFootprint(
  double half_length, double half_width, double margin, const Motion & motion,
  double arc_travel, double total_travel, double epsilon_v, double epsilon_w)
: a_(half_length), b_(half_width + std::max(margin, 0.0)), b_real_(half_width), m_(motion),
  total_(total_travel)
{
  const bool linear = std::abs(motion.v) > epsilon_v;
  const bool rotating = std::abs(motion.w) > epsilon_w;
  const double u = maxPointSpeed(a_, b_, motion);
  if ((!linear && !rotating) || u <= 0.0 || total_travel <= 0.0) {
    empty_ = true;
    return;
  }
  empty_ = false;
  // 순간 회전 중심이 아주 멀면(거의 직선) 직선 공식이 수치적으로 안전하다
  straight_ = std::abs(motion.w) < 1e-12 || std::abs(motion.v / motion.w) > kStraightRadius;
  if (straight_) {
    m_.w = 0.0;
    return;
  }
  radius_ = motion.v / motion.w;
  // 원호 구간: 가장 빠른 점이 arc_travel 을 움직이는 시간. 선운동이 없으면 원호로 끝까지.
  const double arc_time = (linear ? std::min(arc_travel, total_travel) : total_travel) / u;
  arc_angle_ = std::min(std::abs(motion.w) * arc_time, kTwoPi);
  if (linear && arc_travel < total_travel) {
    has_tail_ = true;
    tail_pose_ = unicycleDelta(motion.v, motion.w, arc_time);
    tail_offset_ = std::abs(motion.v) * arc_time;
  }
}

double SweptFootprint::maxPointSpeed(
  double half_length, double half_width, const Motion & motion)
{
  // 강체 속도장 (v - w·y, w·x) 의 크기는 볼록 다각형에서 꼭짓점이 최대
  double best = 0.0;
  for (const double x : {-half_length, half_length}) {
    for (const double y : {-half_width, half_width}) {
      best = std::max(best, std::hypot(motion.v - motion.w * y, motion.w * x));
    }
  }
  return best;
}

double SweptFootprint::straightApproach(const Vec2 & p, double b) const
{
  if (std::abs(p.y()) > b) {
    return kInf;  // 스윕 통로 밖 (측면 벽 등)
  }
  const double ahead = (m_.v > 0.0 ? p.x() : -p.x()) - a_;
  return ahead >= 0.0 ? ahead : kInf;  // 음수 = 풋프린트 옆/뒤 (이미 지나왔다)
}

double SweptFootprint::arcApproach(const Vec2 & p, double b) const
{
  // 로봇 좌표에서 정지점은 순간 회전 중심 c = (0, R) 둘레를 각속도 -w 로 돈다.
  // 반지름 ρ 원과 사각형 변의 교점 중 운동 방향으로 가장 먼저 만나는 각 Δ 가 접촉 → 이동 거리 ρ·Δ.
  const double dx = p.x();
  const double dy = p.y() - radius_;
  const double rho = std::hypot(dx, dy);
  if (rho < 1e-9) {
    return kInf;  // 회전 중심 위의 점은 움직이지 않는다
  }
  const double th0 = std::atan2(dy, dx);
  const double dir = m_.w > 0.0 ? -1.0 : 1.0;
  double best = kInf;
  auto consider = [&](double x, double y) {
      double delta = std::fmod(dir * (std::atan2(y - radius_, x) - th0), kTwoPi);
      if (delta < 0.0) {
        delta += kTwoPi;
      }
      best = std::min(best, delta);
    };
  const double rho2 = rho * rho;
  for (const double sx : {-a_, a_}) {
    const double h2 = rho2 - sx * sx;
    if (h2 >= 0.0) {
      const double h = std::sqrt(h2);
      for (const double y : {radius_ + h, radius_ - h}) {
        if (std::abs(y) <= b) {
          consider(sx, y);
        }
      }
    }
  }
  for (const double sy : {-b, b}) {
    const double h2 = rho2 - (sy - radius_) * (sy - radius_);
    if (h2 >= 0.0) {
      const double h = std::sqrt(h2);
      for (const double x : {h, -h}) {
        if (std::abs(x) <= a_) {
          consider(x, sy);
        }
      }
    }
  }
  if (best > arc_angle_) {
    return kInf;
  }
  return rho * best;
}

double SweptFootprint::approach(const Vec2 & p) const
{
  if (empty_) {
    return kInf;
  }
  const bool beside = std::abs(p.x()) <= a_;
  if (beside && std::abs(p.y()) <= b_real_) {
    return 0.0;  // 풋프린트 안
  }
  // 이미 좌우 여유 안에 있는 점은 여유 없이 판정 (옆 벽이 "닿음" 이 되지 않게)
  const double b = beside && std::abs(p.y()) <= b_ ? b_real_ : b_;
  double s = kInf;
  if (straight_) {
    s = straightApproach(p, b);
  } else {
    s = arcApproach(p, b);
    if (!std::isfinite(s) && has_tail_) {
      // 원호 끝 자세에서 접선 직선으로 계속 (원호 중 닿지 않은 점만 여기 온다)
      const Vec2 q = tail_pose_.inverse().apply(p);
      const double ahead = straightApproach(q, b);
      if (std::isfinite(ahead)) {
        s = tail_offset_ + ahead;
      }
    }
  }
  return s <= total_ ? s : kInf;
}

// ---------------------------------------------------------------- 강건 통계

double kthSmallest(std::vector<double> values, int support)
{
  const int k = std::max(support, 1);
  if (static_cast<int>(values.size()) < k) {
    return kInf;
  }
  std::nth_element(values.begin(), values.begin() + (k - 1), values.end());
  return values[k - 1];
}

double windowedOrderStatistic(
  const std::vector<double> & values, int window, int support, bool circular)
{
  const int n = static_cast<int>(values.size());
  const int k = std::max(support, 1);
  if (n == 0) {
    return kInf;
  }
  const int win = std::max(window, k);
  if (n <= win) {
    return kthSmallest(values, k);
  }
  const int starts = circular ? n : n - win + 1;
  double best = kInf;
  std::vector<double> buf(win);
  for (int s = 0; s < starts; ++s) {
    // 창 안에 유한 값이 support 개 미만이면 건너뛴다 (대부분의 창이 여기서 끝난다)
    int finite = 0;
    for (int j = 0; j < win; ++j) {
      const double x = values[(s + j) % n];
      buf[j] = x;
      finite += std::isfinite(x) ? 1 : 0;
    }
    if (finite < k) {
      continue;
    }
    std::nth_element(buf.begin(), buf.begin() + (k - 1), buf.end());
    best = std::min(best, buf[k - 1]);
  }
  return best;
}

double physicalWindowStatistic(
  const std::vector<double> & values, const std::vector<double> & ranges, double angle_increment,
  double width, int min_window, double support_ratio, bool circular)
{
  const int n = static_cast<int>(values.size());
  if (n == 0 || ranges.size() != values.size()) {
    return kInf;
  }
  const int min_half = std::max(min_window, 1) / 2;
  const int max_half = std::max(min_half, (n - 1) / 2);
  const double dphi = std::max(std::abs(angle_increment), 1e-6);
  double best = kInf;
  std::vector<double> buf;
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(values[i]) || values[i] >= best) {
      continue;  // 창 중심은 유한 값 빔만. 이미 더 작은 결과가 있으면 볼 필요 없다
    }
    const double r = std::isfinite(ranges[i]) && ranges[i] > 0.0 ? ranges[i] : 1.0;
    const int half = std::clamp(
      static_cast<int>(std::ceil(0.5 * width / (r * dphi))), min_half, max_half);
    const int len = 2 * half + 1;
    const int k = std::max(1, static_cast<int>(std::ceil(support_ratio * len - 1e-9)));
    buf.clear();
    for (int j = i - half; j <= i + half; ++j) {
      if (circular) {
        buf.push_back(values[((j % n) + n) % n]);
      } else if (j >= 0 && j < n) {
        buf.push_back(values[j]);
      } else {
        buf.push_back(kInf);  // 스캔 밖 = 해당 없음
      }
    }
    std::nth_element(buf.begin(), buf.begin() + (k - 1), buf.end());
    best = std::min(best, buf[k - 1]);
  }
  return best;
}

// ---------------------------------------------------------------- 게이트

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

double SafetyGate::brakingDistance(double speed, const SafetyParams & p)
{
  const double u = std::abs(speed);
  return u * p.reaction_latency + u * u / (2.0 * p.max_deceleration);
}

double SafetyGate::clearanceSpeedLimit(
  double distance, double stop_distance, const SafetyParams & p)
{
  if (!std::isfinite(distance)) {
    return kInf;
  }
  // d(v) = v·t + v²/(2a) <= D - d_stop 의 양의 근
  const double margin = std::max(distance - stop_distance, 0.0);
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

SweptFootprint SafetyGate::sweptRegion(const Motion & motion) const
{
  const double a = 0.5 * params_.footprint_length;
  const double b = 0.5 * params_.footprint_width;
  const double u = SweptFootprint::maxPointSpeed(a, b + params_.swept_margin, motion);
  // 원호 = 정지 포락선 (지금 멈추기 시작해도 지나가는 거리 + 안전 거리),
  // 전체 = 경고 존(히스테리시스 포함)까지 — 포락선이 더 길면 포락선
  const double envelope = params_.emergency_stop_distance + brakingDistance(u, params_);
  const double total = std::max(params_.warning_zone_distance + params_.zone_hysteresis, envelope) +
    params_.region_margin;
  return SweptFootprint(
    a, b, params_.swept_margin, motion, envelope, total, params_.motion_epsilon_v,
    params_.motion_epsilon_w);
}

void SafetyGate::setScan(
  const std::vector<Vec2> & beams, const Vec2 & origin, double angle_increment, bool circular,
  double stamp)
{
  beams_ = beams;
  ranges_.assign(beams.size(), std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 0; i < beams.size(); ++i) {
    ranges_[i] = (beams[i] - origin).norm();
  }
  angle_increment_ = angle_increment;
  circular_ = circular;
  scan_stamp_ = stamp;
}

double SafetyGate::robustBeams(const std::vector<double> & values, bool guard) const
{
  return guard ?
         physicalWindowStatistic(
    values, ranges_, angle_increment_, params_.guard_window_width, params_.guard_window, 0.5,
    circular_) :
         physicalWindowStatistic(
    values, ranges_, angle_increment_, params_.beam_window_width, params_.beam_window,
    params_.beam_support, circular_);
}

void SafetyGate::setCloud(const std::vector<Vec2> & points, double capture, double received)
{
  cloud_ = points;
  cloud_capture_ = capture;
  cloud_stamp_ = received;
}

Pose2D SafetyGate::latencyCorrection(double stamp, double now) const
{
  const bool fresh = measured_stamp_ >= 0.0 &&
    now - measured_stamp_ <= params_.measured_velocity_timeout;
  if (!params_.latency_compensation || !fresh || stamp < 0.0) {
    return Pose2D{};
  }
  const double age = std::clamp(now - stamp, 0.0, params_.max_compensation_age);
  return unicycleDelta(measured_.v, measured_.w, age).inverse();
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

void SafetyGate::setMeasuredVelocity(double linear, double angular, double stamp)
{
  if (!std::isfinite(linear) || !std::isfinite(angular)) {
    return;
  }
  const double dt = measured_stamp_ >= 0.0 ? stamp - measured_stamp_ : -1.0;
  const double tau = params_.measured_velocity_time_constant;
  if (dt <= 0.0 || dt > params_.measured_velocity_timeout || tau <= 0.0) {
    measured_ = Motion{linear, angular};
  } else {
    const double alpha = dt / (tau + dt);
    measured_.v += alpha * (linear - measured_.v);
    measured_.w += alpha * (angular - measured_.w);
  }
  measured_stamp_ = stamp;
}

double minTrackTtc(const std::vector<TrackTtc> & tracks, bool only_dynamic)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & t : tracks) {
    if (std::isnan(t.ttc) || (only_dynamic && !t.is_dynamic)) {
      continue;
    }
    best = std::min(best, t.ttc);
  }
  return best;
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

void SafetyGate::setEstopSource(const std::string & source, bool active)
{
  estop_sources_[source] = active;
  if (active) {
    estop_latched_ = true;
  } else if (!params_.estop_release_requires_reset && allSourcesClear()) {
    estop_latched_ = false;
  }
}

ResetResult SafetyGate::requestReset()
{
  if (!estop_latched_) {
    return ResetResult{true, "E-stop 래치 없음"};
  }
  if (allSourcesClear()) {
    estop_latched_ = false;
    return ResetResult{true, "E-stop 래치 해제"};
  }
  // 거절된 reset 은 아무 상태도 남기지 않는다 — false 를 보낸 뒤 다시 reset 해야 한다
  std::string active;
  for (const auto & kv : estop_sources_) {
    if (kv.second) {
      active += (active.empty() ? "" : ", ") + kv.first;
    }
  }
  return ResetResult{false, "E-stop 입력이 아직 true (" + active + ") — false 뒤에 다시 reset"};
}

void SafetyGate::setExclusionPolygon(const std::vector<Vec2> & polygon, double stamp)
{
  exclusion_ = polygon;
  exclusion_stamp_ = stamp;
}

bool SafetyGate::isSelf(const Vec2 & p) const
{
  const double hl = 0.5 * params_.footprint_length - params_.self_filter_margin;
  const double hw = 0.5 * params_.footprint_width - params_.self_filter_margin;
  return std::abs(p.x()) < hl && std::abs(p.y()) < hw;
}

bool SafetyGate::excluded(const Vec2 & p, double now) const
{
  return exclusion_.size() >= 3 && exclusion_stamp_ >= 0.0 &&
         now - exclusion_stamp_ <= params_.exclusion_timeout && pointInPolygon(p, exclusion_);
}

std::vector<Motion> SafetyGate::motionHypotheses(
  const SafetyCommand & cmd, bool cmd_fresh, double now)
{
  std::vector<Motion> out;
  auto moving = [this](const Motion & m) {
      return std::abs(m.v) > params_.motion_epsilon_v || std::abs(m.w) > params_.motion_epsilon_w;
    };
  const Motion c{cmd_fresh ? cmd.linear : 0.0, cmd_fresh ? cmd.angular : 0.0};
  if (moving(c)) {
    out.push_back(c);
    intent_ = c;
    have_intent_ = true;
  }
  if (measured_stamp_ >= 0.0 && now - measured_stamp_ <= params_.measured_velocity_timeout &&
    moving(measured_))
  {
    out.push_back(measured_);
  }
  // 멈춰 있으면 마지막 명령 운동 방향을 유지한다 — 정지 원인 앞에서 STOP/존이 깜박이지 않게
  if (out.empty() && params_.hold_motion_intent && have_intent_) {
    out.push_back(intent_);
  }
  return out;
}

bool SafetyGate::hasCause(const std::string & cause) const
{
  return std::find(stop_causes_.begin(), stop_causes_.end(), cause) != stop_causes_.end();
}

void SafetyGate::updateChannel(Channel & ch, double value, double threshold, bool new_frame)
{
  ch.value = value;
  if (new_frame) {
    ch.count = value <= threshold ? ch.count + 1 : 0;
  }
}

bool SafetyGate::confirmed(const Channel & ch, double threshold) const
{
  if (ch.value > threshold) {
    return false;
  }
  return ch.count >= params_.stop_confirm_frames ||
         ch.value <= threshold - params_.immediate_stop_margin;
}

bool SafetyGate::escapeAllowed(double v, double w, const Pose2D & scan_correction) const
{
  // 접촉 가드 점들까지의 강건 거리(창 중앙값)가 운동 중 줄지 않으면 "멀어짐"
  std::vector<Vec2> pts(beams_.size(), Vec2(kInf, kInf));
  std::vector<double> now_d(beams_.size(), kInf);
  for (std::size_t i = 0; i < beams_.size(); ++i) {
    const Vec2 & p = beams_[i];
    if (std::isfinite(p.x()) && std::isfinite(p.y()) && !isSelf(p)) {
      pts[i] = scan_correction.apply(p);
      now_d[i] = footprintDistance(pts[i]);
    }
  }
  const double g_now = robustBeams(now_d, true);
  std::vector<double> after(beams_.size(), kInf);
  for (int step = 1; step <= kEscapeSteps; ++step) {
    const Pose2D inv =
      unicycleDelta(v, w, params_.escape_horizon * step / kEscapeSteps).inverse();
    for (std::size_t i = 0; i < beams_.size(); ++i) {
      after[i] = std::isfinite(now_d[i]) ? footprintDistance(inv.apply(pts[i])) : kInf;
    }
    const double g = robustBeams(after, true);
    if (g < g_now - kEscapeTolerance) {
      return false;
    }
  }
  return true;
}

SafetyStatus SafetyGate::evaluate(double now)
{
  SafetyStatus st;
  const SafetyParams & p = params_;
  const double h = p.zone_hysteresis;
  if (!p.estop_release_requires_reset && estop_latched_ && allSourcesClear()) {
    estop_latched_ = false;
  }
  st.estop_latched = estop_latched_;

  // 센서: 지연(경고) / 고장(대응)
  for (const auto & s : sensors_) {
    const double age = now - last_seen_[s.name];
    if (age > s.fault_timeout) {
      st.failed_sensors.push_back(s.name);
      if (s.action == SensorFailureAction::kStop) {
        st.sensor_stop = true;
      } else {
        st.degraded = true;
      }
    } else if (age > s.late_timeout) {
      st.late_sensors.push_back(s.name);
    }
  }

  // 입력 명령
  const bool cmd_fresh = input_stamp_ >= 0.0 && now - input_stamp_ <= p.command_timeout;
  st.command_stale = !cmd_fresh;
  const double v_in = cmd_fresh ? std::clamp(
    input_.linear, p.min_linear_velocity, p.max_linear_velocity) : 0.0;
  const double w_in = cmd_fresh ? std::clamp(
    input_.angular, -p.max_angular_velocity, p.max_angular_velocity) : 0.0;

  // 운동 가설 → 접근 영역
  std::vector<SweptFootprint> regions;
  for (const auto & m : motionHypotheses(SafetyCommand{v_in, w_in}, cmd_fresh, now)) {
    SweptFootprint r = sweptRegion(m);
    if (!r.empty()) {
      regions.push_back(r);
    }
  }
  auto approach = [&regions](const Vec2 & q) {
      double s = kInf;
      for (const auto & r : regions) {
        s = std::min(s, r.approach(q));
      }
      return s;
    };

  // LiDAR: 빔별 접근 거리(본/예외)와 접촉 거리 → 빔 창 순위 통계.
  // 촬영 후 로봇이 움직인 만큼 점을 옮긴다 (처리·전송 지연 동안의 접근을 놓치지 않게)
  const Pose2D scan_corr = latencyCorrection(scan_stamp_, now);
  const std::size_t nb = beams_.size();
  std::vector<double> main_s(nb, kInf);
  std::vector<double> excl_s(nb, kInf);
  std::vector<double> guard_d(nb, kInf);
  for (std::size_t i = 0; i < nb; ++i) {
    const Vec2 & raw = beams_[i];
    if (!std::isfinite(raw.x()) || !std::isfinite(raw.y()) || isSelf(raw)) {
      continue;  // 무효 빔, 자기 차체 반사는 어떤 판정에도 쓰지 않는다
    }
    const Vec2 q = scan_corr.apply(raw);
    guard_d[i] = footprintDistance(q);
    (excluded(q, now) ? excl_s[i] : main_s[i]) = approach(q);
  }
  const double lidar_main = robustBeams(main_s, false);
  const double lidar_excl = robustBeams(excl_s, false);
  const double guard = robustBeams(guard_d, true);

  // 깊이 점군 (전방, LiDAR 평면 아래 물체)
  double cloud_main = kInf;
  double cloud_excl = kInf;
  const bool cloud_fresh = cloud_stamp_ >= 0.0 && now - cloud_stamp_ <= p.cloud_timeout &&
    now - cloud_capture_ <= p.cloud_max_age;
  st.cloud_stale = p.cloud_required && !cloud_fresh;
  if (cloud_fresh) {
    const Pose2D cloud_corr = latencyCorrection(cloud_capture_, now);
    std::vector<double> cm;
    std::vector<double> ce;
    for (const auto & raw : cloud_) {
      if (!std::isfinite(raw.x()) || !std::isfinite(raw.y()) || isSelf(raw)) {
        continue;
      }
      const Vec2 q = cloud_corr.apply(raw);
      const double s = approach(q);
      if (std::isfinite(s)) {
        (excluded(q, now) ? ce : cm).push_back(s);
      }
    }
    cloud_main = kthSmallest(cm, p.cloud_min_points);
    cloud_excl = kthSmallest(ce, p.cloud_min_points);
  }

  // 시간 일관성: 새 프레임마다 연속 계수
  const bool new_scan = scan_stamp_ >= 0.0 && scan_stamp_ != counted_scan_stamp_;
  if (new_scan) {
    counted_scan_stamp_ = scan_stamp_;
  }
  const bool new_cloud = cloud_fresh && cloud_stamp_ != counted_cloud_stamp_;
  if (new_cloud) {
    counted_cloud_stamp_ = cloud_stamp_;
  }
  updateChannel(lidar_main_, lidar_main, p.emergency_stop_distance, new_scan);
  updateChannel(lidar_excl_, lidar_excl, p.exclusion_stop_distance, new_scan);
  updateChannel(guard_, guard, p.contact_guard_distance, new_scan);
  updateChannel(cloud_main_, cloud_main, p.emergency_stop_distance, new_cloud);
  updateChannel(cloud_excl_, cloud_excl, p.exclusion_stop_distance, new_cloud);

  const double d_main = std::min(lidar_main, cloud_main);
  const double d_excl = std::min(lidar_excl, cloud_excl);
  st.min_distance = d_main;
  st.lidar_distance = lidar_main;
  st.cloud_distance = cloud_main;
  st.contact_distance = guard;
  st.exclusion_distance = d_excl;

  // STOP 래치
  std::vector<std::string> causes;
  if (confirmed(lidar_main_, p.emergency_stop_distance)) {
    causes.emplace_back("lidar");
  }
  if (confirmed(cloud_main_, p.emergency_stop_distance)) {
    causes.emplace_back("depth");
  }
  if (confirmed(guard_, p.contact_guard_distance)) {
    causes.emplace_back("contact");
  }
  if (confirmed(lidar_excl_, p.exclusion_stop_distance) ||
    confirmed(cloud_excl_, p.exclusion_stop_distance))
  {
    causes.emplace_back("dock_exclusion");
  }
  if (!causes.empty()) {
    if (!proximity_stop_) {
      stop_causes_.clear();
    }
    appendUnique(stop_causes_, causes);
    proximity_stop_ = true;
  } else if (proximity_stop_) {
    // 원인별 해제: 현재 운동으로 더 이상 다가가지 않는다(장애물이 떠났거나 운동이 바뀌었다),
    // 접촉 가드는 떨어졌다. 원인이 아닌 채널(예: 통로 옆 벽의 가드 거리 0.07 m)은
    // 해제를 막지 않는다. **원인이 된 센서가 다시 보여 줘야** 푼다: 깊이 점군이 끊기면 그 값은
    // +inf 라 합친 거리로는 "치웠다" 로 보인다 — 깊이로 선 STOP 이 점군이 끊긴 사이 풀려
    // 장애물이 그대로 있는데 다시 움직였다 (통합 시나리오 09 실측).
    const bool lidar_clear = !hasCause("lidar") || lidar_main > p.stop_release_distance;
    const bool depth_clear = !hasCause("depth") ||
      (cloud_fresh && cloud_main > p.stop_release_distance);
    const bool main_clear = lidar_clear && depth_clear;
    const bool excl_clear = !hasCause("dock_exclusion") ||
      d_excl > p.exclusion_stop_distance + h;
    const bool guard_clear = !hasCause("contact") || guard > p.contact_guard_distance + h;
    if (main_clear && excl_clear && guard_clear) {
      proximity_stop_ = false;
      stop_causes_.clear();
    }
  }
  st.proximity_stop = proximity_stop_;
  st.stop_causes = stop_causes_;

  // 존 (완화 방향만 히스테리시스)
  if (proximity_stop_) {
    zone_ = SafetyZone::kStop;
  } else {
    SafetyZone raw = classifyZone(d_main, p);
    if (raw == SafetyZone::kStop) {
      // 확정 전 (연속 프레임 대기) — 속도는 여유 거리 제한이 이미 0 으로 묶는다
      raw = SafetyZone::kCritical;
    }
    if (static_cast<int>(raw) >= static_cast<int>(zone_) || zone_ == SafetyZone::kStop) {
      zone_ = raw;
    } else {
      const double boundary = zone_ == SafetyZone::kCritical ?
        p.critical_zone_distance : p.warning_zone_distance;
      if (d_main > boundary + h) {
        zone_ = raw;
      }
    }
  }
  st.zone = zone_;

  // 거리 기반 점 속도 상한 (풋프린트 최고 점 속도에 적용 — 곡률 유지)
  double u_cap = kInf;
  if (zone_ == SafetyZone::kWarning) {
    u_cap = std::min(u_cap, p.warning_zone_max_speed);
  } else if (zone_ == SafetyZone::kCritical) {
    u_cap = std::min(u_cap, p.critical_zone_max_speed);
  }
  if (p.clearance_speed_limit_enabled) {
    u_cap = std::min(u_cap, clearanceSpeedLimit(d_main, p.emergency_stop_distance, p));
  }
  if (std::isfinite(d_excl)) {
    u_cap = std::min(u_cap, p.critical_zone_max_speed);
    if (p.clearance_speed_limit_enabled) {
      u_cap = std::min(u_cap, clearanceSpeedLimit(d_excl, p.exclusion_stop_distance, p));
    }
  }
  if (st.degraded) {
    u_cap = std::min(u_cap, p.degraded_mode_max_speed);
  }
  // 깊이 점군이 없으면 전방 저상 물체를 못 본다 → 전진만 저속 (후진·회전은 카메라 밖)
  double v_fwd = kInf;
  if (st.cloud_stale) {
    v_fwd = p.degraded_mode_max_speed;
  }
  // TTC 기반 연속 감속 (선속도만)
  double v_ttc = kInf;
  const bool ttc_fresh = ttc_stamp_ >= 0.0 && now - ttc_stamp_ <= p.ttc_max_age;
  st.min_ttc = ttc_fresh ? min_ttc_ : kInf;
  bool ttc_limited = false;
  if (p.ttc_limit_enabled && ttc_fresh && min_ttc_ <= p.ttc_critical) {
    v_ttc = std::max(0.0, p.max_deceleration * (min_ttc_ - p.reaction_latency));
  }
  st.speed_limit = std::min({u_cap, v_ttc, v_fwd, p.max_linear_velocity});
  st.angular_limit = std::min(p.max_angular_velocity, u_cap / circumscribedRadius());

  const double half_l = 0.5 * p.footprint_length;
  const double half_w = 0.5 * p.footprint_width;
  auto scale = [&](double & v, double & w, double u_lim, double v_lim) {
      double s = 1.0;
      const double u = SweptFootprint::maxPointSpeed(half_l, half_w, Motion{v, w});
      if (u > u_lim) {
        s = std::min(s, u_lim / u);
      }
      if (std::abs(v) > v_lim) {
        s = std::min(s, v_lim / std::abs(v));
      }
      v *= s;
      w *= s;
    };

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
  if (!st.late_sensors.empty()) {
    st.reasons.emplace_back("sensor_late");
  }
  if (st.cloud_stale) {
    st.reasons.emplace_back("depth_cloud_stale");
  }
  if (st.command_stale) {
    st.reasons.emplace_back("command_stale");
  }
  if (st.estop_latched || st.sensor_stop || st.command_stale) {
    v = 0.0;
    w = 0.0;
  } else if (proximity_stop_) {
    st.reasons.emplace_back("proximity_stop");
    for (const auto & c : stop_causes_) {
      st.reasons.push_back("stop_" + c);
    }
    // 현재 운동이 아직 정지 원인에 다가가면 0. 접촉 가드만 남았으면 멀어지는 명령만 저속 통과.
    // 깊이로 선 STOP 인데 점군이 끊겼으면 "멀어진다" 를 증명할 수 없다 → 다가가는 것으로 본다
    // (탈출 판정은 LiDAR 빔만 보므로 평면 아래 물체를 못 본다, 통합 시나리오 09).
    const bool depth_blind = hasCause("depth") && !cloud_fresh;
    const bool approaching = depth_blind ||
      ((hasCause("lidar") || hasCause("depth")) && d_main <= p.stop_release_distance) ||
      (hasCause("dock_exclusion") && d_excl <= p.exclusion_stop_distance + h) ||
      d_main <= p.emergency_stop_distance || d_excl <= p.exclusion_stop_distance;
    const bool moving = std::abs(v_in) > p.motion_epsilon_v ||
      std::abs(w_in) > p.motion_epsilon_w;
    if (!approaching && p.allow_escape && moving && escapeAllowed(v_in, w_in, scan_corr)) {
      st.escaping = true;
      st.reasons.emplace_back("escape");
      scale(v, w, std::min(p.escape_max_speed, u_cap), v > 0.0 ? std::min(v_ttc, v_fwd) : v_ttc);
    } else {
      v = 0.0;
      w = 0.0;
    }
  } else {
    scale(v, w, u_cap, v > 0.0 ? std::min(v_ttc, v_fwd) : v_ttc);
    ttc_limited = std::isfinite(v_ttc) && std::abs(v_in) > v_ttc && v_ttc <= u_cap;
  }
  if (ttc_limited) {
    st.reasons.emplace_back("ttc_limit");
  }
  if (zone_ == SafetyZone::kWarning) {
    st.reasons.emplace_back("warning_zone");
  } else if (zone_ == SafetyZone::kCritical) {
    st.reasons.emplace_back("critical_zone");
  }
  if (std::isfinite(d_excl)) {
    st.reasons.emplace_back("dock_exclusion");
  }
  st.command.linear = v;
  st.command.angular = w;
  st.estop_active = st.estop_latched || st.sensor_stop;
  return st;
}

}  // namespace amr_perception
