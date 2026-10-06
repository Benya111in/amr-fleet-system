// DecisionEngine 시험 (가짜 정책, ROS·ORT 비의존).
//  20 Hz 틱 → 10 Hz 결정 (ZOH 가속도), prev_actions 갱신, DWA 폴백 네 사유, 계약 폴백(99),
//  히스테리시스, 목표 정렬, 재동기, 이탈 상태.
//  A5-1: 결정 경계 = sim 시각 격자 (RTF ≠ 1 에서 틱 간격이 0.05 가 아니어도 결정 주기 평균 0.1 s),
//  램프 = 실제 sim 경과, 시계 멈춤, DWA 주기의 실행 가속도·이탈 타이머 = 실제 경과, 근접 인계.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "amr_navigation/core/velocity_profiler.hpp"
#include "amr_rl_controller/core/decision_engine.hpp"
#include "golden_io.hpp"

namespace C = amr_rl_controller::contract;
using amr_rl_controller::Arm;
using amr_rl_controller::DecisionEngine;
using amr_rl_controller::DwaReason;
using amr_rl_controller::EngineConfig;
using amr_rl_controller::PolicyInterface;
using amr_rl_controller::PolicyOutput;
using amr_rl_controller::Source;
using amr_rl_controller::TickInput;

namespace
{
class FakePolicy : public PolicyInterface
{
public:
  explicit FakePolicy(Arm a)
  : arm_(a) {}
  Arm arm() const override {return arm_;}
  PolicyOutput run(const float * obs, const float * mask) override
  {
    ++calls;
    last_obs.assign(obs, obs + C::kObsDim);
    if (mask) {
      last_mask.assign(mask, mask + C::kNActionsC);
    }
    PolicyOutput o;
    o.ok = ok;
    o.error = ok ? "" : "가짜 실패";
    o.infer_ms = infer_ms;
    o.b_action = {u0, u1};
    o.logits.assign(C::kNActionsC, -1e9f);
    if (mask) {
      for (int i = 0; i < C::kNActionsC; ++i) {
        if (mask[i] > 0.5f) {
          o.logits[static_cast<std::size_t>(i)] = logit_of(i);
        }
      }
    }
    o.c_action = forced_action >= 0 ? forced_action : argmax(o.logits);
    return o;
  }
  void resetHidden() override {++resets;}
  static int64_t argmax(const std::vector<float> & l)
  {
    int64_t b = 0;
    for (std::size_t i = 1; i < l.size(); ++i) {
      if (l[i] > l[static_cast<std::size_t>(b)]) {
        b = static_cast<int64_t>(i);
      }
    }
    return b;
  }
  float logit_of(int i) const {return prefer >= 0 ? (i == prefer ? 1.0f : 0.0f) : 0.01f * i;}

  Arm arm_;
  int calls{0};
  int resets{0};
  bool ok{true};
  double infer_ms{0.1};
  float u0{0.5f};
  float u1{0.0f};
  int64_t forced_action{-1};
  int prefer{-1};
  std::vector<float> last_obs;
  std::vector<float> last_mask;
};

TickInput openInput(double t, double v_meas = 0.0)
{
  TickInput in;
  in.now = t;
  in.state.pose = {0.0, 0.0, 0.0};
  for (int k = 0; k <= 200; ++k) {
    in.state.path.push_back({0.05 * k, 0.0});
  }
  in.state.scan_ranges.assign(C::kLidarSamples, std::numeric_limits<double>::infinity());
  in.state.v_meas = v_meas;
  in.dist_to_goal = 10.0;
  return in;
}
}  // namespace

TEST(DecisionEngine, HoldsAccelerationBetweenDecisions)
{
  FakePolicy p(Arm::kB);
  DecisionEngine eng(EngineConfig{}, &p);
  ASSERT_EQ(eng.ticksPerDecision(), 2);
  std::vector<double> v;
  for (int k = 0; k < 6; ++k) {
    auto in = openInput(0.05 * k);
    const auto r = eng.tick(in);
    EXPECT_FALSE(r.need_dwa);
    EXPECT_EQ(r.new_decision, k % 2 == 0) << k;
    v.push_back(r.v);
  }
  EXPECT_EQ(p.calls, 3);   // 결정은 2 틱마다
  // a_v = 0.5 m/s² ZOH: 0.025, 0.05, 0.075, 0.10, …
  for (int k = 0; k < 6; ++k) {
    EXPECT_NEAR(v[static_cast<std::size_t>(k)], 0.025 * (k + 1), 1e-12) << k;
  }
  EXPECT_NEAR(eng.prevActions()[0].first, 0.5, 1e-12);
  // 셋째 결정의 관측: v_cmd = 0.1 (/2.0), 최근 행동 a_v 0.5 (/1.0) 두 개
  EXPECT_NEAR(p.last_obs[C::kObsEgo + 4], 0.1 / 2.0, 1e-7);
  EXPECT_NEAR(p.last_obs[C::kObsEgo + 6], 0.5, 1e-7);
  EXPECT_NEAR(p.last_obs[C::kObsEgo + 8], 0.5, 1e-7);
  EXPECT_NEAR(p.last_obs[C::kObsEgo + 10], 0.0, 1e-7);
}

TEST(DecisionEngine, DwaFallbackReasons)
{
  struct Case
  {
    std::string name;
    DwaReason why;
  };
  for (const auto & c : std::vector<Case>{{"timeout", DwaReason::kTimeout},
      {"error", DwaReason::kPolicyError}, {"invalid", DwaReason::kStateInvalid},
      {"nan", DwaReason::kNonFinite}})
  {
    FakePolicy p(Arm::kB);
    EngineConfig cfg;
    cfg.max_inference_ms = 5.0;
    DecisionEngine eng(cfg, &p);
    auto in = openInput(0.0);
    if (c.name == "timeout") {
      p.infer_ms = 6.0;
    } else if (c.name == "error") {
      p.ok = false;
    } else if (c.name == "invalid") {
      in.state_ok = false;
      in.state_error = "스캔 없음";
    } else {
      p.u0 = std::numeric_limits<float>::quiet_NaN();
    }
    auto r = eng.tick(in);
    EXPECT_TRUE(r.need_dwa) << c.name;
    EXPECT_EQ(r.dwa_reason, c.why) << c.name;
    EXPECT_EQ(r.source, Source::kDwa) << c.name;
    EXPECT_FALSE(eng.commitDwa(0.05, 0.05).limited) << c.name;
    auto in2 = openInput(0.05);
    r = eng.tick(in2);
    EXPECT_TRUE(r.need_dwa) << c.name;   // 결정 주기 전체가 DWA
    EXPECT_FALSE(r.new_decision);
    EXPECT_FALSE(eng.commitDwa(0.1, 0.15).limited) << c.name;
    // 다음 결정: v_cmd = 마지막 DWA 명령, 실행 가속도 = (0.1 − 0)/0.1, (0.15 − 0)/0.1
    p.ok = true;
    p.infer_ms = 0.1;
    p.u0 = 0.0f;
    auto in3 = openInput(0.1);
    r = eng.tick(in3);
    EXPECT_FALSE(r.need_dwa) << c.name;
    EXPECT_NEAR(eng.vCmd(), 0.1, 1e-12) << c.name;
    EXPECT_NEAR(eng.wCmd(), 0.15, 1e-12) << c.name;
    EXPECT_NEAR(eng.prevActions()[0].first, 1.0, 1e-9) << c.name;
    EXPECT_NEAR(eng.prevActions()[0].second, 1.5, 1e-9) << c.name;
  }
}

// 리뷰 반영: DWA 틱의 명령도 실제 직전 명령 기준 계약 동적 창 안으로 (내부 DWA 의 창 중심이 낡음)
TEST(DecisionEngine, DwaCommandIsLimitedToContractWindowAroundLastCommand)
{
  const double dv = C::kAMax * 0.05;
  const double dw = C::kAlphaMax * 0.05;
  FakePolicy p(Arm::kB);   // u0 0.5 → a_v 0.5 m/s²
  DecisionEngine eng(EngineConfig{}, &p);
  for (int k = 0; k < 4; ++k) {
    auto in = openInput(0.05 * k);
    ASSERT_FALSE(eng.tick(in).need_dwa);
  }
  ASSERT_NEAR(eng.lastV(), 0.10, 1e-12);
  ASSERT_NEAR(eng.lastW(), 0.0, 1e-12);

  // 정책 실패 → DWA 주기. 낡은 창 중심에서 고른 (0.30, 0.6): v ≤ min(v_cap 0.12, 0.10 + dv)
  p.ok = false;
  auto in = openInput(0.20, 0.10);
  in.state.v_cap = 0.12;
  ASSERT_TRUE(eng.tick(in).need_dwa);
  auto c = eng.commitDwa(0.30, 0.6);
  EXPECT_TRUE(c.limited);
  EXPECT_NEAR(c.v, 0.12, 1e-12);
  EXPECT_NEAR(c.w, dw, 1e-12);
  EXPECT_NEAR(eng.lastV(), 0.12, 1e-12);
  EXPECT_NEAR(eng.lastW(), dw, 1e-12);

  // 같은 주기 둘째 틱: 감속 쪽 큰 점프 (−0.5, −1.0) → 한 틱 최대 감속
  in = openInput(0.25, 0.12);
  in.state.v_cap = 0.12;
  auto r = eng.tick(in);
  ASSERT_TRUE(r.need_dwa);
  ASSERT_FALSE(r.new_decision);
  c = eng.commitDwa(-0.5, -1.0);
  EXPECT_TRUE(c.limited);
  EXPECT_NEAR(c.v, 0.12 - dv, 1e-12);
  EXPECT_NEAR(c.w, 0.0, 1e-12);

  // 창 안 요청은 그대로
  in = openInput(0.30, 0.07);
  ASSERT_TRUE(eng.tick(in).need_dwa);
  c = eng.commitDwa(0.09, -0.08);
  EXPECT_FALSE(c.limited);
  EXPECT_NEAR(c.v, 0.09, 1e-12);
  EXPECT_NEAR(c.w, -0.08, 1e-12);

  // 속도 제한이 직전 명령 아래로 떨어지면 최대 감속만 (요청이 더 빨라도)
  in = openInput(0.35, 0.09);
  in.state.v_cap = 0.0;
  ASSERT_TRUE(eng.tick(in).need_dwa);
  c = eng.commitDwa(0.5, -0.08);
  EXPECT_TRUE(c.limited);
  EXPECT_NEAR(c.v, 0.09 - dv, 1e-12);

  // 비유한 요청 → 계약 제동 규칙 (v·ω 를 0 쪽으로)
  in = openInput(0.40, 0.04);
  ASSERT_TRUE(eng.tick(in).need_dwa);
  c = eng.commitDwa(std::numeric_limits<double>::quiet_NaN(), 0.0);
  EXPECT_TRUE(c.limited);
  EXPECT_NEAR(c.v, 0.0, 1e-12);
  EXPECT_NEAR(c.w, 0.0, 1e-12);

  // ω 는 ±ω_max 안: 직전 1.45 에서 2.0 요청 → 1.5
  EngineConfig cfg;
  DecisionEngine e2(cfg, &p);
  auto in2 = openInput(0.0);
  in2.state.w_meas = 1.45;
  ASSERT_TRUE(e2.tick(in2).need_dwa);   // p.ok = false → DWA, 재동기 ω = 1.45
  c = e2.commitDwa(0.0, 2.0);
  EXPECT_TRUE(c.limited);
  EXPECT_NEAR(c.w, C::kWMax, 1e-12);
}

TEST(DecisionEngine, ModeCChoosesMaskedCellAndRechecksMask)
{
  FakePolicy p(Arm::kC);
  DecisionEngine eng(EngineConfig{}, &p);
  auto in = openInput(0.0);
  auto r = eng.tick(in);
  ASSERT_FALSE(r.need_dwa);
  ASSERT_EQ(p.last_mask.size(), static_cast<std::size_t>(C::kNActionsC));
  // 정지·직전 행동 0: 저크 조건으로 a_v ∈ {−0.2, 0, 0.2} 중 v' ≥ 0 인 0·0.2, a_ω ∈ {−0.5, 0, 0.5}
  EXPECT_EQ(r.n_valid, 6);
  EXPECT_FLOAT_EQ(p.last_mask[C::kFallbackIndex], 1.0f);
  // 가짜 정책은 0.01·i (큰 인덱스 선호) → 유효 셀 중 가장 큰 59 = (0.2, 0.5)? 폴백 99 가 더 크다
  EXPECT_EQ(r.action, C::kFallbackIndex);
  EXPECT_EQ(r.source, Source::kFallbackCell);

  // 마스크 밖 행동을 내는 모델 → 99 로 바꾼다 (onnx_io.md 플러그인 의무)
  FakePolicy q(Arm::kC);
  q.forced_action = 0;   // (−1.0, −2.0) — 저크 마스크 밖
  DecisionEngine e2(EngineConfig{}, &q);
  auto in2 = openInput(0.0);
  r = e2.tick(in2);
  EXPECT_TRUE(r.invalid_action);
  EXPECT_EQ(r.model_action, 0);
  EXPECT_EQ(r.action, C::kFallbackIndex);

  // 유효 셀 선호: 58 = (0.2, 0) → 첫 틱 v = 0.2·0.05
  FakePolicy s(Arm::kC);
  s.prefer = 58;
  DecisionEngine e3(EngineConfig{}, &s);
  auto in3 = openInput(0.0);
  r = e3.tick(in3);
  EXPECT_EQ(r.action, 58);
  EXPECT_EQ(r.source, Source::kPolicy);
  EXPECT_NEAR(r.v, 0.01, 1e-12);
  EXPECT_NEAR(r.w, 0.0, 1e-12);
}

TEST(DecisionEngine, HysteresisKeepsPreviousCell)
{
  FakePolicy p(Arm::kC);
  EngineConfig cfg;
  cfg.hysteresis_beta = 10.0;
  DecisionEngine eng(cfg, &p);
  p.prefer = 49;   // (0, 0)
  auto in = openInput(0.0);
  auto r = eng.tick(in);
  EXPECT_EQ(r.action, 49);
  auto in1 = openInput(0.05);
  eng.tick(in1);
  p.prefer = 58;   // 정책이 바꾸려 해도 β 가 더 크면 직전 셀 유지
  auto in2 = openInput(0.1);
  r = eng.tick(in2);
  EXPECT_EQ(r.action, 49);
}

TEST(DecisionEngine, GoalAlignRotatesInPlaceWithAlphaLimit)
{
  FakePolicy p(Arm::kB);
  DecisionEngine eng(EngineConfig{}, &p);
  auto in = openInput(0.0);
  in.dist_to_goal = 0.05;
  in.yaw_error_to_goal = 1.0;
  auto r = eng.tick(in);
  EXPECT_EQ(r.source, Source::kAlign);
  EXPECT_EQ(p.calls, 0);
  EXPECT_NEAR(r.v, 0.0, 1e-12);
  EXPECT_NEAR(r.w, 0.1, 1e-12);   // α 2.0 × 0.05 s
  auto in1 = openInput(0.05);
  in1.dist_to_goal = 0.05;
  in1.yaw_error_to_goal = 1.0;
  r = eng.tick(in1);
  EXPECT_NEAR(r.w, 0.2, 1e-12);
  // 아직 움직이는 중(|v| > 0.05)이면 정책이 맡는다
  auto in2 = openInput(0.1, 0.3);
  in2.dist_to_goal = 0.05;
  r = eng.tick(in2);
  EXPECT_EQ(r.source, Source::kPolicy);
}

namespace
{
// 폐루프 정렬 (A2-5): 엔진 20 Hz → amr_navigation VelocityProfiler 코어 50 Hz (PI 끔, 배포 한계
// — 각저크 6.0, α 2.0) → 순수 지연 0.04 s + 1 차 지연 0.08 s 플랜트 (V:16-17) → yaw 적분
// (기준 단계 0.01 s).
// goal checker 흉내: 엔진 틱마다 |오차| < tol 이면 성공 → 그 뒤 명령 0 (controller_server 가 0 을
// 낸다). 3 s 더 굴려 멈춘 뒤의 오차를 돌려준다.
double alignFinalError(double e0, const EngineConfig & cfg, double tol = 0.05)
{
  FakePolicy p(Arm::kB);
  DecisionEngine eng(cfg, &p);
  amr_navigation::core::VelocityProfilerConfig pc;
  pc.linear = {-0.5, C::kVMaxHw, C::kAMax, C::kJerkMax};
  pc.angular = {-C::kWMax, C::kWMax, C::kAlphaMax, C::kAngJerkMax};
  pc.use_pid = false;
  amr_navigation::core::VelocityProfiler prof(pc);
  const double dt = 0.01;
  const double tm = 0.08;
  std::vector<double> delay(4, 0.0);   // 0.04 s
  double yaw = 0.0;
  double w = 0.0;
  double w_cmd = 0.0;
  double w_ref = 0.0;
  double a_ref = 0.0;   // 플러그인처럼 ω_ref 를 50 Hz 차분 (velocity_profiler/state)
  bool reached = false;
  for (int k = 0; k < 2000; ++k) {
    const double t = k * dt;
    if (k % 5 == 0 && !reached) {
      const double err = std::remainder(e0 - yaw, 2.0 * M_PI);
      if (std::abs(err) < tol) {
        reached = true;
        w_cmd = 0.0;
      } else {
        auto in = openInput(t);
        in.dist_to_goal = 0.0;
        in.yaw_error_to_goal = err;
        in.state.w_meas = w;
        in.state.a_w_meas = a_ref;
        w_cmd = eng.tick(in).w;
      }
    }
    if (k % 2 == 0) {
      const double prev = w_ref;
      w_ref = prof.update(0.0, w_cmd, false, 0.0, 0.0, 0.02).w;
      a_ref = (w_ref - prev) / 0.02;
    }
    delay.insert(delay.begin(), w_ref);
    const double u = delay.back();
    delay.pop_back();
    w += (u - w) * (1.0 - std::exp(-dt / tm));
    yaw += w * dt;
    if (reached && k > 1500) {
      break;
    }
  }
  EXPECT_TRUE(reached) << e0;
  EXPECT_LT(std::abs(w), 1e-3) << e0;
  return std::remainder(e0 - yaw, 2.0 * M_PI);
}
}  // namespace

// A2-5: Gazebo 에서 정렬이 goal checker 성공 뒤 0.16–0.26 rad 더 돌았다 (사다리꼴 정지 속도).
// 저크·지연을 넣은 정지 속도로 바꾼 뒤에는 하류 사슬을 거쳐 멈춘 오차가 yaw 허용치 0.05 안이다.
TEST(DecisionEngine, GoalAlignStopsWithinYawToleranceThroughJerkLimitedChain)
{
  for (const double e0 : {0.3, -0.3, 1.0, -1.0, 2.06, -2.06, 3.0, -3.0}) {
    EXPECT_LE(std::abs(alignFinalError(e0, EngineConfig{})), 0.05) << e0;
  }
}

TEST(DecisionEngine, ResyncsAfterGapAndOnReset)
{
  FakePolicy p(Arm::kB);
  p.u0 = 0.0f;
  DecisionEngine eng(EngineConfig{}, &p);
  auto in = openInput(0.0, 0.4);
  auto r = eng.tick(in);
  EXPECT_NEAR(r.v, 0.4, 1e-12);   // 측정 속도에서 시작
  auto in1 = openInput(3.0, 0.7);   // 0.5 s 넘는 공백
  r = eng.tick(in1);
  EXPECT_NEAR(r.v, 0.7, 1e-12);
  EXPECT_TRUE(r.new_decision);
  eng.reset();
  EXPECT_EQ(p.resets, 1);
  auto in2 = openInput(3.05, 0.2);
  r = eng.tick(in2);
  EXPECT_NEAR(r.v, 0.2, 1e-12);
}

TEST(DecisionEngine, DevStateUpdatedBeforeObservation)
{
  FakePolicy p(Arm::kB);
  DecisionEngine eng(EngineConfig{}, &p);
  auto in = openInput(0.0);
  in.state.pose = {1.0, 0.4, 0.0};   // 경로에서 0.4 m
  eng.tick(in);
  EXPECT_TRUE(eng.dev().in_dev);
  EXPECT_NEAR(eng.dev().peak, 0.4, 1e-12);
  EXPECT_FLOAT_EQ(p.last_obs[C::kObsDev], 1.0f);
  EXPECT_NEAR(p.last_obs[C::kObsDev + 1], 0.4, 1e-6);
  auto in1 = openInput(0.05);
  in1.state.pose = {1.0, 0.4, 0.0};
  eng.tick(in1);
  auto in2 = openInput(0.1);
  in2.state.pose = {1.05, 0.35, 0.0};
  eng.tick(in2);
  EXPECT_NEAR(eng.dev().t_since_peak, 0.1, 1e-12);
  eng.resetEpisode();
  EXPECT_FALSE(eng.dev().in_dev);
}

TEST(DecisionEngine, GoldenStateDecisionUsesContractMask)
{
  // d03 (전방 장애물) 상태로 C 결정: 엔진이 만든 관측·마스크 = golden (prev/v_cmd 를 맞춘 뒤)
  const auto j = golden::load(std::string(RL_CONTRACT_DIR) + "/golden/d03_obstacle_front.json");
  const auto s = golden::toState(j["state"]);
  FakePolicy p(Arm::kC);
  DecisionEngine eng(EngineConfig{}, &p);
  TickInput in;
  in.now = 0.0;
  in.state = s;
  in.state.v_meas = s.v_cmd;     // 재동기 v_cmd = 측정 속도
  in.state.w_meas = s.w_cmd;
  in.dist_to_goal = 5.0;
  const bool prev_zero = s.prev_actions[0].first == 0.0 && s.prev_actions[0].second == 0.0 &&
    s.prev_actions[1].first == 0.0 && s.prev_actions[2].first == 0.0;
  if (!prev_zero || s.v_meas != s.v_cmd || s.w_meas != s.w_cmd) {
    GTEST_SKIP() << "d03 은 재동기 상태와 다르다";
  }
  eng.tick(in);
  const auto & em = j["expected"]["mask"];
  for (int i = 0; i < C::kNActionsC; ++i) {
    EXPECT_EQ(p.last_mask[static_cast<std::size_t>(i)] > 0.5f, em[i].get<bool>()) << i;
  }
}

// ---------------------------------------------------------------- A5-1 sim 시각 결정
namespace
{
struct DecisionLog
{
  std::vector<double> t_dec;      ///< 결정 틱 시각
  std::vector<double> t_tick;
  std::vector<double> v;
  std::vector<double> last_period;
};

/// 틱 시각 목록으로 엔진을 돌린다 (정책 B, a_v = u0·1.0 고정, 개활지).
DecisionLog runTicks(const std::vector<double> & times, float u0 = 0.5f)
{
  FakePolicy p(Arm::kB);
  p.u0 = u0;
  DecisionEngine eng(EngineConfig{}, &p);
  DecisionLog log;
  for (const double t : times) {
    auto in = openInput(t);
    const auto r = eng.tick(in);
    EXPECT_FALSE(r.need_dwa);
    if (r.new_decision) {
      log.t_dec.push_back(t);
      log.last_period.push_back(r.last_period);
    }
    log.t_tick.push_back(t);
    log.v.push_back(r.v);
  }
  return log;
}

std::vector<double> uniformTicks(double dt, double t_end)
{
  std::vector<double> t;
  for (int k = 0; k * dt <= t_end + 1e-12; ++k) {
    t.push_back(k * dt);
  }
  return t;
}

std::vector<double> periodsOf(const DecisionLog & log)
{
  std::vector<double> p;
  for (std::size_t i = 1; i < log.t_dec.size(); ++i) {
    p.push_back(log.t_dec[i] - log.t_dec[i - 1]);
  }
  return p;
}

double mean(const std::vector<double> & x)
{
  double s = 0.0;
  for (const double v : x) {
    s += v;
  }
  return x.empty() ? std::numeric_limits<double>::quiet_NaN() : s / static_cast<double>(x.size());
}
}  // namespace

// RTF 0.5·0.7·1.0·1.4·1.6 의 sim 틱 간격 (controller_server 는 벽시계 20 Hz). 결정 주기 평균이 0.1
// ± 0.002,
// 각 주기가 [0.1 − dt, 0.1 + dt] 안이고, 엔진이 보고한 last_period 가 실제 간격과 같다.
// 틱 수 결정(1차 물결·A2)이면 dt 0.035 에서 평균 0.07, 0.07 에서 0.14 라 걸린다.
TEST(DecisionEngineSimTime, DecisionPeriodFollowsSimTimeGrid)
{
  for (const double dt : {0.025, 0.035, 0.04, 0.05, 0.07, 0.08}) {
    const auto log = runTicks(uniformTicks(dt, 30.0));
    const auto per = periodsOf(log);
    ASSERT_GT(per.size(), 250U) << dt;
    EXPECT_NEAR(mean(per), 0.1, 0.002) << dt;
    for (std::size_t i = 0; i < per.size(); ++i) {
      EXPECT_GE(per[i], 0.1 - dt - 1e-9) << dt << " " << i;
      EXPECT_LE(per[i], 0.1 + dt + 1e-9) << dt << " " << i;
      EXPECT_NEAR(log.last_period[i + 1], per[i], 1e-12) << dt << " " << i;
    }
  }
}

// 벽시계 지터 + 부하로 sim 틱 간격이 매 틱 U[0.02, 0.07] 로 바뀌어도 평균 주기 0.1 ± 0.002 (격자
// 고정 — 결정이
// 늦거나 일러도 오차가 쌓이지 않는다). 격자 대신 "결정 틱 + 0.1" 로 다시 잡으면 평균이 0.1 + 평균
// 넘침이 된다.
TEST(DecisionEngineSimTime, RandomTickJitterKeepsMeanPeriod)
{
  std::mt19937 rng(20261006);
  std::uniform_real_distribution<double> U(0.02, 0.07);
  for (int rep = 0; rep < 5; ++rep) {
    std::vector<double> t{0.0};
    while (t.back() < 60.0) {
      t.push_back(t.back() + U(rng));
    }
    const auto per = periodsOf(runTicks(t));
    EXPECT_NEAR(mean(per), 0.1, 0.002) << rep;
    // 다음 틱 간격을 모르고 직전 간격으로 반 틱을 재므로 한 주기는 0.1 ± 한 틱보다 조금 더 벗어날
    // 수 있다
    EXPECT_LE(*std::max_element(per.begin(), per.end()), 0.1 + 2 * 0.07) << rep;
    EXPECT_GE(*std::min_element(per.begin(), per.end()), 0.1 - 2 * 0.07) << rep;
  }
}

// 램프는 sim 초당 a: 일정 가속 a_v 0.25 를 RTF 0.7 (dt 0.035) 틱으로 돌리면 명령이 0.25·t 를 따른다
// (틱 수 램프면 0.25·(t/0.035)·0.05 = 1.43 배). 틱 간격 추정(지수 이동 평균)이 수렴한 뒤(30 틱) 한
// 틱의 명령 변화는
// a·dt 를 넘지 않는다 (결정 경계 포함 — 짧게 끝난 주기의 끝 명령으로 건너뛰지 않는다).
TEST(DecisionEngineSimTime, ZohRampSlopeIsPerSimSecond)
{
  const double a = 0.25;
  for (const double dt : {0.025, 0.035, 0.05, 0.08}) {
    const auto log = runTicks(uniformTicks(dt, 3.0), static_cast<float>(a));
    ASSERT_GT(log.v.size(), 36U);
    for (std::size_t i = 30; i < log.v.size(); ++i) {
      EXPECT_LE(log.v[i] - log.v[i - 1], a * dt + 1e-4) << dt << " " << i;   // 추정 수렴 잔차
      EXPECT_GE(log.v[i] - log.v[i - 1], -1e-9) << dt << " " << i;
      // 이 틱이 낸 값 = 다음 틱까지 붙잡을 값 ≈ a·(t + dt)
      EXPECT_NEAR(log.v[i], a * (log.t_tick[i] + dt), a * dt + 1e-4) << dt << " " << i;
    }
  }
}

// 벽시계 루프가 밀렸다 따라잡으면 sim 간격 1 ms 틱이 끼어든다 (Gazebo 실측, A5-1 1 회차). 그
// 틱에서 램프가 뒤로 물러나면
// 안 된다: 한 결정 주기 안에서는 명령이 단조, 결정 경계에서의 되돌림은 지수 평균의 한 걸음(a·α·dt)
// 이하.
TEST(DecisionEngineSimTime, OutlierShortTickDoesNotReverseRamp)
{
  const double a = 0.25;
  std::vector<double> t{0.0};
  for (int k = 1; t.back() < 3.0; ++k) {
    t.push_back(t.back() + (k % 13 == 0 ? 0.001 : (k % 13 == 1 ? 0.014 : 0.025)));
  }
  FakePolicy p(Arm::kB);
  p.u0 = static_cast<float>(a);
  DecisionEngine eng(EngineConfig{}, &p);
  double v_prev = 0.0;
  for (std::size_t i = 0; i < t.size(); ++i) {
    auto in = openInput(t[i]);
    const auto r = eng.tick(in);
    if (i > 0) {
      const double floor = r.new_decision ? v_prev - a * 0.2 * 0.025 - 1e-9 : v_prev - 1e-12;
      EXPECT_GE(r.v, floor) << i << " t " << t[i] << (r.new_decision ? " (결정)" : "");
    }
    v_prev = r.v;
  }
}

// 시계가 멈춘 틱 (sim 시각이 같다 — /clock 이 벽시계 틱 사이에 안 왔다): 새 결정 없이 같은 명령을
// 낸다.
TEST(DecisionEngineSimTime, ClockStallRepeatsCommandWithoutDecision)
{
  FakePolicy p(Arm::kB);
  DecisionEngine eng(EngineConfig{}, &p);
  auto in0 = openInput(0.0);
  const auto r0 = eng.tick(in0);
  auto in1 = openInput(0.0);
  const auto r1 = eng.tick(in1);
  EXPECT_TRUE(r0.new_decision);
  EXPECT_FALSE(r1.new_decision);
  EXPECT_NEAR(r1.v, r0.v, 1e-12);
  EXPECT_EQ(p.calls, 1);
  EXPECT_NEAR(r1.tick_dt, 0.0, 1e-12);
  auto in2 = openInput(0.05);
  const auto r2 = eng.tick(in2);
  EXPECT_FALSE(r2.new_decision);
  EXPECT_NEAR(r2.v, 0.05, 1e-12);   // a 0.5 × 0.1 (결정 뒤 0.05 + 다음 틱 추정 0.05)
}

// DWA 주기의 실행 가속도 = 실제로 바뀐 명령 / 실제 경과 (0.105 s 주기), 이탈 타이머도 실제 경과로
// 간다.
TEST(DecisionEngineSimTime, DwaAccelAndDevTimerUseElapsedSimTime)
{
  FakePolicy p(Arm::kB);
  p.ok = false;   // 정책 실패 → DWA
  DecisionEngine eng(EngineConfig{}, &p);
  const double dt = 0.035;
  auto in = openInput(0.0);
  in.state.pose = {1.0, 0.4, 0.0};   // 경로에서 0.4 m (이탈 구간)
  ASSERT_TRUE(eng.tick(in).need_dwa);
  eng.commitDwa(0.01, 0.0);
  for (int k = 1; k <= 2; ++k) {
    auto ik = openInput(k * dt);
    ik.state.pose = {1.0, 0.4, 0.0};
    ASSERT_TRUE(eng.tick(ik).need_dwa);
    eng.commitDwa(0.01 + 0.03 * k, 0.0);
  }
  p.ok = true;
  p.u0 = 0.0f;
  auto i3 = openInput(3 * dt);   // 0.105 ≥ 0.1 − 0.0175 → 결정
  i3.state.pose = {1.05, 0.35, 0.0};
  const auto r = eng.tick(i3);
  ASSERT_TRUE(r.new_decision);
  EXPECT_NEAR(r.last_period, 3 * dt, 1e-12);
  EXPECT_NEAR(eng.prevActions()[0].first, 0.07 / (3 * dt), 1e-9);
  EXPECT_NEAR(eng.vCmd(), 0.07, 1e-12);
  EXPECT_NEAR(eng.dev().t_since_peak, 3 * dt, 1e-12);
}

// 근접 인계: 반경 0.4 m 안이면 DWA (사유 5, 정렬보다 먼저), 히스테리시스 0.1 m 밖으로 나가야
// 정책으로 돌아간다.
// 새 목표(resetEpisode)는 인계를 푼다.
TEST(DecisionEngineSimTime, HandoverWithinRadiusUsesDwaWithHysteresis)
{
  FakePolicy p(Arm::kB);
  EngineConfig cfg;
  cfg.handover_radius = 0.4;
  cfg.handover_hysteresis = 0.1;
  DecisionEngine eng(cfg, &p);
  double t = 0.0;
  auto step = [&](double dist, double v_meas = 0.3) {
      auto in = openInput(t, v_meas);
      in.dist_to_goal = dist;
      t += 0.1;
      return eng.tick(in);
    };
  auto r = step(0.5);
  EXPECT_EQ(r.source, Source::kPolicy);
  EXPECT_FALSE(r.handover);
  r = step(0.39);
  EXPECT_TRUE(r.need_dwa);
  EXPECT_EQ(r.dwa_reason, DwaReason::kHandover);
  EXPECT_TRUE(r.handover);
  r = step(0.45);   // 반경 밖이지만 0.5 안 → 인계 유지
  EXPECT_EQ(r.dwa_reason, DwaReason::kHandover);
  r = step(0.05, 0.0);   // 정렬 조건이어도 인계 중이면 DWA
  EXPECT_EQ(r.dwa_reason, DwaReason::kHandover);
  r = step(0.51);
  EXPECT_EQ(r.source, Source::kPolicy);
  EXPECT_FALSE(eng.inHandover());
  r = step(0.3);
  EXPECT_TRUE(eng.inHandover());
  eng.resetEpisode();
  EXPECT_FALSE(eng.inHandover());
  // 끔(0)이면 정렬이 그대로
  FakePolicy q(Arm::kB);
  DecisionEngine e2(EngineConfig{}, &q);
  auto in = openInput(0.0);
  in.dist_to_goal = 0.05;
  in.yaw_error_to_goal = 0.5;
  EXPECT_EQ(e2.tick(in).source, Source::kAlign);
}

// 틱 간격이 매 틱 바뀌면(지터) dt_est 가 틀린 몫을 다음 결정에서 바로잡는다 — 명령이 a·t 에서
// 멀어지지 않는다
// (마지막으로 낸 값을 v_cmd 로 이으면 추정 오차가 결정마다 쌓여 a·t 에서 걷는다).
TEST(DecisionEngineSimTime, JitterDoesNotAccumulateIntegrationError)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> U(0.02, 0.07);
  std::vector<double> t{0.0};
  while (t.back() < 1.6) {   // a 0.5 로 v ≤ 0.85 (상한 1.0 전)
    t.push_back(t.back() + U(rng));
  }
  const auto log = runTicks(t);
  for (std::size_t i = 0; i < log.v.size(); ++i) {
    // 낸 값 = 다음 틱까지 붙잡을 값: a·t 와 한 결정 주기 안 오차(≤ a·(dt_max − dt_min) + a·dt_max)
    // 이내
    EXPECT_NEAR(log.v[i], 0.5 * log.t_tick[i], 0.5 * (0.07 + 0.05) + 1e-9) << i;
  }
  for (std::size_t i = 2; i < log.v.size(); ++i) {
    EXPECT_LE(std::abs(log.v[i] - log.v[i - 1]), 0.5 * (0.07 + 0.05) + 1e-9) << i;
  }
  EXPECT_NEAR(log.v.back(), 0.5 * log.t_tick.back(), 0.5 * 0.07 + 0.5 * 0.05);
}

// 틱 간격 추정은 지수 이동 평균 (α 0.2): 따라잡는 1 ms 틱 하나는 추정을 한 걸음(0.2·차)만
// 움직인다. 결정 경계의
// 반 틱 판정과 램프의 앞내다봄이 그 틱 하나에 흔들리지 않게 (직전 간격을 그대로 쓰면 0.001 이
// 된다).
TEST(DecisionEngineSimTime, TickIntervalEstimateIgnoresSingleOutlier)
{
  FakePolicy p(Arm::kB);
  DecisionEngine eng(EngineConfig{}, &p);
  double t = 0.0;
  double est = 0.0;
  for (int k = 0; k < 60; ++k) {
    auto in = openInput(t);
    est = eng.tick(in).dt_est;
    t += 0.025;
  }
  EXPECT_NEAR(est, 0.025, 1e-4);
  auto in = openInput(t - 0.025 + 0.001);
  const auto r = eng.tick(in);
  EXPECT_NEAR(r.tick_dt, 0.001, 1e-9);
  EXPECT_NEAR(r.dt_est, 0.025 + 0.2 * (0.001 - 0.025), 1e-4);
}
