#include "amr_rl_controller/core/decision_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>

#include "amr_navigation/core/speed_profile.hpp"
#include "amr_rl_controller/core/dev_state.hpp"
#include "amr_rl_controller/core/geometry.hpp"

namespace amr_rl_controller
{

namespace C = contract;

namespace
{
double msSince(const std::chrono::steady_clock::time_point & t0)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

double clamp(double x, double lo, double hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}

/// [s] dt_est 하한 — /clock 해상도(Gazebo 물리 1 ms) 아래로는 추정하지 않는다
constexpr double kMinTickDt = 1e-3;

// 제자리 정렬의 정지각 [rad] (A2-5): 지연 t_c 동안 지금 각가속도 a0(회전 방향 성분, 음수면 0 으로
// 보수적)를 유지 → 저크 j 로 각가속도를 0 까지 → 저크 제한 정지 (SpeedProfile::stoppingDistance).
// 지금 가속 중인 것을 빼면 하류 각저크 필터가 가속도를 +α 에서 뒤집는 동안 더 돈다 — 오프라인
// 폐루프 [계산]: 그것만 빼면 초기 오차 1.0 rad 에서 0.118 rad 지나쳤다.
double alignStopAngle(double w, double a0, double t_c)
{
  const double j = C::kAngJerkMax;
  a0 = std::max(0.0, a0);
  double d = w * t_c + 0.5 * a0 * t_c * t_c;
  const double w1 = w + a0 * t_c;
  const double t0 = a0 / j;
  d += w1 * t0 + 0.5 * a0 * t0 * t0 - j * t0 * t0 * t0 / 6.0;
  const double w2 = w1 + a0 * a0 / (2.0 * j);
  return d + amr_navigation::core::SpeedProfile::stoppingDistance(w2, C::kAlphaMax, j, 0.0);
}

/// alignStopAngle(ω, a0, t_c) ≤ e 인 최대 ω ∈ [0, ω_max] (단조 증가 → 이분법).
double alignStopSpeed(double e, double a0, double t_c)
{
  if (alignStopAngle(C::kWMax, a0, t_c) <= e) {
    return C::kWMax;
  }
  double lo = 0.0;
  double hi = C::kWMax;
  for (int i = 0; i < 50; ++i) {
    const double m = 0.5 * (lo + hi);
    (alignStopAngle(m, a0, t_c) <= e ? lo : hi) = m;
  }
  return lo;
}
}  // namespace

DecisionEngine::DecisionEngine(const EngineConfig & cfg, PolicyInterface * policy)
: cfg_(cfg), policy_(policy)
{
  ticks_per_decision_ = std::max(
    1, static_cast<int>(std::lround(cfg_.decision_period / std::max(1e-6, cfg_.control_period))));
}

void DecisionEngine::reset()
{
  synced_ = false;
  kind_ = Kind::kNone;
  in_handover_ = false;
  last_period_ = 0.0;
  prev_ = {};
  prev_cell_ = -1;
  dev_ = DevState{};
  if (policy_) {
    policy_->resetHidden();
  }
}

void DecisionEngine::resetEpisode()
{
  dev_ = DevState{};
  prev_cell_ = -1;
  in_handover_ = false;
  if (policy_) {
    policy_->resetHidden();
  }
}

void DecisionEngine::closePeriod(double elapsed)
{
  if (kind_ == Kind::kNone) {
    return;
  }
  Accel a;
  if (kind_ == Kind::kDwa) {
    // 실행 가속도 = 그 주기 동안 실제로 바뀐 명령 / 실제 sim 경과 (A5-1: 공칭 0.1 이 아니다)
    const double e = elapsed > 1e-6 ? elapsed : cfg_.decision_period;
    a = {(v_out_ - v_cmd_) / e, (w_out_ - w_cmd_) / e};
    v_cmd_ = v_out_;
    w_cmd_ = w_out_;
  } else {
    a = {target_.a_v, target_.a_w};
    const Command c = rampAt(elapsed);
    v_cmd_ = c.v;
    w_cmd_ = c.w;
  }
  for (std::size_t k = prev_.size() - 1; k > 0; --k) {
    prev_[k] = prev_[k - 1];
  }
  prev_[0] = a;
  kind_ = Kind::kNone;
}

int DecisionEngine::selectC(const PolicyOutput & out, const MaskResult & m, TickResult & res)
{
  res.model_action = static_cast<int>(out.c_action);
  int sel = res.model_action;
  if (cfg_.hysteresis_beta > 0.0) {
    // 직전 셀 가산점 뒤 마스크 안 argmax, 동률이면 작은 인덱스
    double best = -kInf;
    sel = C::kFallbackIndex;
    for (int i = 0; i < C::kNActionsC; ++i) {
      if (!m.mask[static_cast<std::size_t>(i)]) {
        continue;
      }
      double l = out.logits[static_cast<std::size_t>(i)];
      if (i == prev_cell_) {
        l += cfg_.hysteresis_beta;
      }
      if (l > best) {
        best = l;
        sel = i;
      }
    }
  }
  if (sel < 0 || sel >= C::kNActionsC || !m.mask[static_cast<std::size_t>(sel)]) {
    res.invalid_action = true;   // onnx_io.md: mask[action] 을 다시 확인하고 아니면 폴백(99)
    sel = C::kFallbackIndex;
  }
  return sel;
}

void DecisionEngine::startPeriod(TickInput & in, TickResult & res, double dev_dt)
{
  res.new_decision = true;
  ContractState & s = in.state;
  v_hi_ = std::min(s.v_cap, C::kVMaxHw);
  s.v_cmd = v_cmd_;
  s.w_cmd = w_cmd_;
  s.prev_actions = prev_;

  // 이탈 상태: s' 관측에는 갱신 뒤 값 (contract.md §4)
  if (in.state_ok && !s.path.empty()) {
    try {
      const double e = ContractPath(s.path).project({s.pose.x, s.pose.y}).dist;
      dev_ = devUpdate(dev_, e, dev_dt);
    } catch (const std::exception & ex) {
      in.state_ok = false;
      in.state_error = ex.what();
    }
  }
  s.dev = dev_;

  auto dwa = [&](DwaReason why, const std::string & err) {
      kind_ = Kind::kDwa;
      res.source = Source::kDwa;
      res.dwa_reason = why;
      res.error = err;
      prev_cell_ = -1;
    };

  // 근접 인계: 목표 직선거리 ≤ r_h 이면 DWA 가 끝(정렬 포함)까지 몬다. 풀 때는 r_h + 히스테리시스.
  if (cfg_.handover_radius > 0.0) {
    if (in_handover_) {
      in_handover_ = !(in.dist_to_goal > cfg_.handover_radius + cfg_.handover_hysteresis);
    } else {
      in_handover_ = in.dist_to_goal <= cfg_.handover_radius;
    }
    res.handover = in_handover_;
    if (in_handover_) {
      dwa(DwaReason::kHandover, "근접 인계 (목표 " + std::to_string(in.dist_to_goal) + " m)");
      return;
    }
  }

  // 목표 성공 조건 → 제자리 yaw 정렬 (ω 만, α 한계)
  if (in.dist_to_goal <= cfg_.goal_xy_tol && std::abs(s.v_meas) <= cfg_.goal_stop_speed) {
    kind_ = Kind::kAlign;
    res.source = Source::kAlign;
    const double dp = cfg_.decision_period;
    const double e = in.yaw_error_to_goal;
    double w_des = 0.0;
    if (std::abs(e) > 0.5 * cfg_.yaw_tol) {
      // 정지 가능 속도: 하류 velocity_profiler 의 각저크 필터(램프 α/j)와 명령→실속도 지연
      // (align_latency), 지금 각가속도(a_w_meas = 프로파일러 기준값)를 넣은 정지각이 |e| 이하인
      // 최대 ω — DWA 의 선속도 목표 접근(dwa.cpp:650-653, SpeedProfile::maxSpeedForStop)과 같은
      // 생각에 가속 상태를 더했다. 1차 물결의 사다리꼴 √(2α|e|) 는 지나쳤다 (A2 Gazebo 실측: goal
      // checker 가 성공을 낸 뒤 0.16–0.26 rad 더 돌았다).
      const double a0 = std::copysign(1.0, e) * s.a_w_meas;
      const double w_stop = alignStopSpeed(std::abs(e), a0, cfg_.align_latency);
      w_des = std::copysign(std::min({C::kWMax, w_stop, cfg_.align_gain * std::abs(e)}), e);
    }
    target_.v = std::max(0.0, v_cmd_ - C::kAMax * dp);
    target_.w = clamp(w_des, w_cmd_ - C::kAlphaMax * dp, w_cmd_ + C::kAlphaMax * dp);
    target_.a_v = (target_.v - v_cmd_) / dp;
    target_.a_w = (target_.w - w_cmd_) / dp;
    return;
  }

  if (!in.state_ok || policy_ == nullptr) {
    dwa(DwaReason::kStateInvalid, policy_ ? in.state_error : std::string("정책 없음"));
    return;
  }

  Observation obs;
  MaskResult m;
  std::array<float, C::kNActionsC> maskf{};
  try {
    auto t0 = std::chrono::steady_clock::now();
    obs = buildObservation(s);
    res.obs_ms = msSince(t0);
    if (policy_->arm() == Arm::kC) {
      t0 = std::chrono::steady_clock::now();
      m = mask_builder_.compute(s);
      res.mask_ms = msSince(t0);
      for (std::size_t i = 0; i < maskf.size(); ++i) {
        maskf[i] = m.mask[i] ? 1.0f : 0.0f;
      }
      res.n_valid = m.n_valid;
      res.saturated = m.saturated;
    }
  } catch (const std::exception & ex) {
    dwa(DwaReason::kStateInvalid, ex.what());
    return;
  }

  const PolicyOutput out = policy_->run(obs.data(), maskf.data());
  res.infer_ms = out.infer_ms;
  if (!out.ok) {
    dwa(DwaReason::kPolicyError, out.error);
    return;
  }
  if (out.infer_ms > cfg_.max_inference_ms) {
    dwa(DwaReason::kTimeout, "추론 " + std::to_string(out.infer_ms) + " ms > 상한");
    return;
  }
  if (policy_->arm() == Arm::kB) {
    if (!std::isfinite(out.b_action[0]) || !std::isfinite(out.b_action[1])) {
      dwa(DwaReason::kNonFinite, "B action 비유한");
      return;
    }
    target_ = integrateB(out.b_action[0], out.b_action[1], v_cmd_, w_cmd_, s.v_cap);
    res.source = Source::kPolicy;
  } else {
    if (out.logits.size() != static_cast<std::size_t>(C::kNActionsC)) {
      dwa(DwaReason::kPolicyError, "logits 길이 " + std::to_string(out.logits.size()));
      return;
    }
    for (std::size_t i = 0; i < out.logits.size(); ++i) {
      if (m.mask[i] && !std::isfinite(out.logits[i])) {
        dwa(DwaReason::kNonFinite, "유효 셀 logit 비유한");
        return;
      }
    }
    const int sel = selectC(out, m, res);
    res.action = sel;
    prev_cell_ = sel;
    target_ = integrateC(sel, v_cmd_, w_cmd_);
    res.source = sel == C::kFallbackIndex ? Source::kFallbackCell : Source::kPolicy;
  }
  kind_ = Kind::kPolicy;
}

TickResult DecisionEngine::tick(TickInput & in)
{
  const double dp = cfg_.decision_period;
  const bool resync = !synced_ || in.now - last_tick_ > cfg_.reset_gap || in.now < last_tick_;
  double tick_dt = 0.0;
  if (resync) {
    // 재동기: 측정 속도에서 이어 간다 (정지 상태 시작이면 0)
    const double v_hi = std::min(in.state.v_cap, C::kVMaxHw);
    v_cmd_ = clamp(in.state.v_meas, C::kVMin, std::max(C::kVMin, v_hi));
    w_cmd_ = clamp(in.state.w_meas, -C::kWMax, C::kWMax);
    v_out_ = v_cmd_;
    w_out_ = w_cmd_;
    if (synced_) {
      prev_ = {};
    }
    kind_ = Kind::kNone;
    dt_est_ = cfg_.control_period;
    synced_ = true;
  } else {
    tick_dt = in.now - last_tick_;
    if (tick_dt > 0.0) {
      // sim 시계가 멈춘 틱(간격 0)은 추정을 바꾸지 않는다
      dt_est_ = clamp(tick_dt, kMinTickDt, dp);
    }
  }
  last_tick_ = in.now;
  v_cap_ = in.state.v_cap;

  TickResult res;
  const bool due = kind_ == Kind::kNone || in.now >= next_boundary_ - 0.5 * dt_est_;
  if (due) {
    const bool fresh = kind_ == Kind::kNone;
    const double elapsed = in.now - period_start_;
    closePeriod(elapsed);
    last_period_ = fresh ? 0.0 : elapsed;
    // 이탈 타이머: 실제 경과. 재동기 뒤 첫 결정은 공백을 세지 않는다 (예전처럼 한 주기)
    startPeriod(in, res, fresh ? dp : clamp(elapsed, 0.0, cfg_.reset_gap));
    // 경계 격자: 오차가 쌓이지 않게 직전 경계 + 0.1. 새로 시작했거나 그 경계도 이미 지났으면 지금
    // + 0.1
    double nb = next_boundary_ + dp;
    if (fresh || in.now >= nb - 0.5 * dt_est_) {
      nb = in.now + dp;
    }
    next_boundary_ = nb;
    period_start_ = in.now;
    period_info_ = res;
  } else {
    res = period_info_;
    res.new_decision = false;
    res.obs_ms = res.mask_ms = res.infer_ms = 0.0;
  }
  res.sim_now = in.now;
  res.tick_dt = tick_dt;
  res.last_period = last_period_;
  res.dt_est = dt_est_;
  res.handover = in_handover_;

  if (kind_ == Kind::kDwa) {
    res.need_dwa = true;
    return res;
  }
  // ZOH 램프: 이 틱이 다음 틱까지 붙잡을 값 = 결정 뒤 sim 경과 + 다음 틱까지의 추정
  const Command c = rampAt(in.now - period_start_ + dt_est_);
  res.v = c.v;
  res.w = c.w;
  v_out_ = res.v;
  w_out_ = res.w;
  return res;
}

Command DecisionEngine::rampAt(double t) const
{
  const double dp = cfg_.decision_period;
  Command c = target_;
  if (std::abs(t - dp) <= 1e-9) {
    return c;   // 0.1 s 정각 (20 Hz·RTF 1): 계약 적분의 끝 명령 그대로
  }
  if (t < dp || kind_ == Kind::kAlign) {
    // 정렬은 되먹임 제어라 0.1 뒤에는 끝 명령을 붙잡는다 (목표 ω 를 넘지 않게)
    const double tt = std::min(t, dp);
    c.v = v_cmd_ + target_.a_v * tt;
    c.w = w_cmd_ + target_.a_w * tt;
    return c;
  }
  // 정책 주기가 0.1 보다 길다 (결정 격자 + 이산 틱): 가속도를 다음 결정까지 붙잡는다 (ZOH). 끝
  // 명령에서 멈추면
  // 긴 주기마다 a·(e − 0.1) 를 잃어 평균 가속이 a 보다 작아진다 — 격자 평균 주기는 0.1 이라 이렇게
  // 해야
  // 명령이 a·t 를 따른다. 속도 상자(ray 클램프와 같은 한계) 밖으로는 나가지 않는다.
  c.v = clamp(v_cmd_ + target_.a_v * t, C::kVMin, std::max(C::kVMin, v_hi_));
  c.w = clamp(w_cmd_ + target_.a_w * t, -C::kWMax, C::kWMax);
  return c;
}

FallbackCommand DecisionEngine::brakeStep() const
{
  const double dv = C::kAMax * cfg_.control_period;
  const double dw = C::kAlphaMax * cfg_.control_period;
  FallbackCommand c;
  c.v = v_out_ > 0.0 ? std::max(0.0, v_out_ - dv) : std::min(0.0, v_out_ + dv);
  c.w = w_out_ - std::copysign(std::min(std::abs(w_out_), dw), w_out_);
  return c;
}

FallbackCommand DecisionEngine::commitDwa(double v, double w)
{
  // (리뷰 반영) 내부 DWA 의 창 중심이 실제 직전 명령과 다르면 (정책 구간 뒤 낡은 v_last, 재설정
  // 뒤 측정 속도와 명령의 추종 지연) 그 창에서 고른 명령은 한 틱에 a_max·Δt_c 를 넘게 뛸 수 있다
  // (수정 전 0.30 → 0.15 m/s, 속도 제한 0.20 아래에서 0.35 로 재가속). 그래서 매 DWA 틱 명령을
  // 실제로 낸 명령 기준 계약 동적 창에 다시 넣는다 — 첫 틱만 자르면 다음 틱에 다시 뛴다.
  FallbackCommand c;
  if (!std::isfinite(v) || !std::isfinite(w)) {
    c = brakeStep();
    c.limited = true;
  } else {
    const double dv = C::kAMax * cfg_.control_period;
    const double dw = C::kAlphaMax * cfg_.control_period;
    const double v_lo = v_out_ - dv;
    const double v_hi = std::max(v_lo, std::min(v_cap_, v_out_ + dv));
    c.v = clamp(v, v_lo, v_hi);
    c.w = clamp(clamp(w, w_out_ - dw, w_out_ + dw), -C::kWMax, C::kWMax);
    c.limited = c.v != v || c.w != w;
  }
  v_out_ = c.v;
  w_out_ = c.w;
  return c;
}

}  // namespace amr_rl_controller
