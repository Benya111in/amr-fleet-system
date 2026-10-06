#include "amr_rl_controller/core/decision_engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>

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
  tick_in_period_ = 0;
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
  if (policy_) {
    policy_->resetHidden();
  }
}

void DecisionEngine::closePeriod()
{
  if (kind_ == Kind::kNone) {
    return;
  }
  Accel a;
  if (kind_ == Kind::kDwa) {
    a = {(v_out_ - v_cmd_) / cfg_.decision_period, (w_out_ - w_cmd_) / cfg_.decision_period};
    v_cmd_ = v_out_;
    w_cmd_ = w_out_;
  } else {
    a = {target_.a_v, target_.a_w};
    v_cmd_ = target_.v;
    w_cmd_ = target_.w;
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

void DecisionEngine::startPeriod(TickInput & in, TickResult & res)
{
  res.new_decision = true;
  ContractState & s = in.state;
  s.v_cmd = v_cmd_;
  s.w_cmd = w_cmd_;
  s.prev_actions = prev_;

  // 이탈 상태: s' 관측에는 갱신 뒤 값 (contract.md §4)
  if (in.state_ok && !s.path.empty()) {
    try {
      const double e = ContractPath(s.path).project({s.pose.x, s.pose.y}).dist;
      dev_ = devUpdate(dev_, e, cfg_.decision_period);
    } catch (const std::exception & ex) {
      in.state_ok = false;
      in.state_error = ex.what();
    }
  }
  s.dev = dev_;

  // 목표 성공 조건 → 제자리 yaw 정렬 (ω 만, α 한계)
  if (in.dist_to_goal <= cfg_.goal_xy_tol && std::abs(s.v_meas) <= cfg_.goal_stop_speed) {
    kind_ = Kind::kAlign;
    res.source = Source::kAlign;
    const double dp = cfg_.decision_period;
    const double e = in.yaw_error_to_goal;
    double w_des = 0.0;
    if (std::abs(e) > 0.5 * cfg_.yaw_tol) {
      w_des = std::copysign(
        std::min({C::kWMax, std::sqrt(2.0 * C::kAlphaMax * std::abs(e)), 2.0 * std::abs(e)}), e);
    }
    target_.v = std::max(0.0, v_cmd_ - C::kAMax * dp);
    target_.w = clamp(w_des, w_cmd_ - C::kAlphaMax * dp, w_cmd_ + C::kAlphaMax * dp);
    target_.a_v = (target_.v - v_cmd_) / dp;
    target_.a_w = (target_.w - w_cmd_) / dp;
    return;
  }

  auto dwa = [&](DwaReason why, const std::string & err) {
      kind_ = Kind::kDwa;
      res.source = Source::kDwa;
      res.dwa_reason = why;
      res.error = err;
      prev_cell_ = -1;
    };
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
  if (!synced_ || in.now - last_tick_ > cfg_.reset_gap || in.now < last_tick_) {
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
    tick_in_period_ = ticks_per_decision_;
    synced_ = true;
  }
  last_tick_ = in.now;
  v_cap_ = in.state.v_cap;

  TickResult res;
  if (kind_ == Kind::kNone || tick_in_period_ >= ticks_per_decision_) {
    closePeriod();
    tick_in_period_ = 0;
    startPeriod(in, res);
    period_info_ = res;
  } else {
    res = period_info_;
    res.new_decision = false;
    res.obs_ms = res.mask_ms = res.infer_ms = 0.0;
  }
  ++tick_in_period_;

  if (kind_ == Kind::kDwa) {
    res.need_dwa = true;
    return res;
  }
  if (tick_in_period_ >= ticks_per_decision_) {
    res.v = target_.v;
    res.w = target_.w;
  } else {
    const double t = std::min(cfg_.decision_period, tick_in_period_ * cfg_.control_period);
    res.v = v_cmd_ + target_.a_v * t;
    res.w = w_cmd_ + target_.a_w * t;
  }
  v_out_ = res.v;
  w_out_ = res.w;
  return res;
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
