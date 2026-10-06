// 결정 엔진 (ROS 비의존) — 제어 틱을 sim 시각 0.1 s 정책 결정으로 묶는다
// (onnx_io.md 「플러그인 쪽 처리 순서」, contract.md §3).
//
//  - 결정 경계는 **sim 시각 격자** (A5-1): 다음 경계 b(k+1) = b(k) + 0.1. 틱 시각 t ≥ b − dt_est/2
//  이면
//    그 틱이 결정 틱이다 (경계에 가장 가까운 틱). controller_server 는 벽시계 20 Hz 로 부르므로
//    Gazebo RTF < 1 이면
//    sim 기준 틱 간격이 0.05·RTF 다 — 1차 물결·A2 의 틱 수 결정은 sim 결정 주기가 0.1·RTF 로
//    줄었다.
//    경계보다 한 주기 가까이 뒤처지면 (멈춤·공백) 지금 틱으로 격자를 다시 잡는다.
//  - 결정 주기(0.1 s)마다: 이탈 상태 갱신(실제 경과 sim 시각) → 관측 291 → (C) 마스크 100 → 정책 →
//  적분.
//    결정 사이 틱에서는 가속도를 붙잡고 v_cmd + a·min(0.1, e + dt_est) 를 낸다 (ZOH, e = 결정 뒤
//    sim 경과,
//    dt_est = 다음 틱까지의 추정 = 틱 간격의 지수 이동 평균 — 20 Hz·RTF 1 이면 계약 §3 의 0.05·0.1
//    과 같다.
//    한 주기 안에서 램프 시각은 되돌리지 않는다 — A5-1b).
//    다음 결정의 v_cmd = 실제 경과까지 적분한 명령: 주기가 짧게 끝나면 램프 중간값 (끝 명령으로
//    건너뛰지
//    않는다), 길면 가속도를 그대로 붙잡아 (속도 상자 안) — 격자 평균 주기 0.1 에서 명령이 a·t 를
//    따른다.
//    정렬 주기는 0.1 에서 끝 명령을 붙잡는다 (되먹임이라 목표 ω 를 넘지 않게).
//  - 계약 폴백(C 의 99, DWA 제동 규칙)과 별개로, 정책을 쓸 수 없는 결정 주기는 **DWA 폴백**이다:
//    입력 무효(스캔·경로), 추론 실패(예외·모양), 시간 초과(추론 > max_inference_ms), 비유한 출력,
//    그리고 **근접 인계**(handover_radius > 0 이고 목표 직선거리 ≤ 반경, 나갈 때는 반경 +
//    히스테리시스).
//    그 주기의 모든 틱은 호출자가 DWA 로 명령을 만들고 commitDwa() 로 알려 준다.
//  - 목표 성공 조건(xy ≤ 0.10 m, |v| ≤ 0.05 m/s)에서는 정책 대신 제자리 yaw 정렬 (contract.md §8).
//    정렬 속도 상한은 저크·지연을 넣은 정지 속도 (A2-5 — 사다리꼴은 지나쳤다). 근접 인계 중이면
//    정렬도 DWA 가 한다.
//  - prev_actions = 실제로 낸 명령의 결정 주기 가속도 (DWA·정렬 주기 포함), 최근 순. DWA 주기는
//  실제 경과로 나눈다.
//  - DWA 명령도 직전에 실제로 낸 명령 기준 계약 동적 창(a_max·Δt_c, α_max·Δt_c, v_cap) 안으로
//    자른다 (commitDwa). Δt_c 는 **공칭** control_period — 내부 DWA 의 창도 공칭 주기를 가정하므로
//    같은 단위로
//    자른다 (A5-1 노트 설계 4). 내부 DWA 의 창 중심은 실제 직전 명령과 다를 수 있다 (낡은 v_last,
//    또는 재설정 뒤 측정 속도) — 플러그인 쪽 처리는 rl_controller.hpp.
#ifndef AMR_RL_CONTROLLER__CORE__DECISION_ENGINE_HPP_
#define AMR_RL_CONTROLLER__CORE__DECISION_ENGINE_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "amr_rl_controller/core/kinematics.hpp"
#include "amr_rl_controller/core/mask.hpp"
#include "amr_rl_controller/core/observation.hpp"
#include "amr_rl_controller/core/policy.hpp"
#include "amr_rl_controller/core/state.hpp"

namespace amr_rl_controller
{

struct EngineConfig
{
  double control_period{0.05};      ///< [s] 1 / controller_frequency
  double decision_period{contract::kDt};
  /// 이보다 오래 걸린 추론 결과는 버리고 DWA (20 Hz 예산 50 ms 의 절반)
  double max_inference_ms{25.0};
  double hysteresis_beta{0.0};      ///< C: 직전 셀 logit 가산점 (0 = 끔, contract.md §5.2)
  double reset_gap{0.5};            ///< [s] 틱 간격이 이보다 크면 측정 속도로 재동기
  double goal_xy_tol{contract::kGoalXyTol};
  double goal_stop_speed{contract::kGoalStopSpeed};
  double yaw_tol{0.05};             ///< [rad] 정렬 목표 (N:99)
  /// [s] 정렬 정지각에 넣는 명령→실속도 지연: 각저크 램프 평균 α/2j 0.17 + 서보 0.04–0.12 +
  /// 결정 ZOH 반 주기 0.05 ≈ 0.3 (DWA approach_latency 와 같은 값, nav2_params.yaml:140)
  double align_latency{0.3};
  double align_gain{2.0};           ///< [1/s] 정렬 비례 이득 (ω ≤ k·|e|)
  /// [m] 근접 인계 반경 — 목표 직선거리가 이 안이면 DWA 가 끝까지 몬다 (0 = 끔, 「실측 기반 후속
  /// 기술 조사」
  /// 목표 직전 정지 절: r_h 0.3–0.5 m, 히스테리시스 0.1 m)
  double handover_radius{0.0};
  double handover_hysteresis{0.1};  ///< [m] 인계를 풀 때는 반경 + 이 값보다 멀어져야 한다
};

enum class Source : int {kPolicy = 0, kFallbackCell = 1, kDwa = 2, kAlign = 3};
enum class DwaReason : int
{
  kNone = 0, kStateInvalid = 1, kPolicyError = 2, kTimeout = 3, kNonFinite = 4, kHandover = 5
};
constexpr int kNumDwaReasons = 6;

struct TickInput
{
  double now{0.0};                ///< [s] 노드 시계 (Gazebo 에서는 /clock sim 시각)
  bool state_ok{true};            ///< 관측 입력(스캔 720 빔·경로)이 유효한가
  std::string state_error;
  ContractState state;            ///< v_cmd·w_cmd·prev_actions·dev 는 엔진이 채운다
  double dist_to_goal{kInf};      ///< [m] 목표 직선거리
  double yaw_error_to_goal{0.0};  ///< [rad] wrap(목표 yaw − yaw)
};

struct TickResult
{
  bool need_dwa{false};           ///< 참이면 호출자가 DWA 명령을 만들어 commitDwa() 를 부른다
  double v{0.0};
  double w{0.0};
  bool new_decision{false};
  Source source{Source::kPolicy};
  DwaReason dwa_reason{DwaReason::kNone};
  int action{-1};                 ///< C: 실행한 행동 (셀 또는 99)
  int model_action{-1};           ///< C: 모델 argmax
  bool invalid_action{false};     ///< C: 모델 행동이 마스크 밖이라 99 로 바꿨다
  int n_valid{-1};
  bool saturated{false};
  double obs_ms{0.0};
  double mask_ms{0.0};
  double infer_ms{0.0};
  bool dwa_limited{false};        ///< 이 틱의 DWA 명령을 계약 동적 창으로 잘랐다 (호출자가 채움)
  std::string error;
  // 시간 진단 (A5-1) — sim 시각 기준
  double sim_now{0.0};            ///< [s] 이 틱의 시각
  double tick_dt{0.0};            ///< [s] 직전 틱과의 간격 (재동기 틱은 0)
  double last_period{0.0};        ///< [s] 가장 최근 결정 틱에서 닫힌 결정 주기의 길이 (첫 결정은 0)
  double dt_est{0.0};             ///< [s] 다음 틱까지의 추정 (램프에 쓴 값)
  bool handover{false};           ///< 근접 인계 중
};

/// need_dwa 틱에 실제로 낼 명령 (commitDwa 의 결과).
struct FallbackCommand
{
  double v{0.0};
  double w{0.0};
  bool limited{false};            ///< 요청이 계약 동적 창 밖(또는 비유한)이라 바꿨다
};

class DecisionEngine
{
public:
  DecisionEngine(const EngineConfig & cfg, PolicyInterface * policy);

  /// activate·재시작: 다음 틱에 측정 속도로 재동기, 이전 행동 0, 은닉 상태 0, 이탈 상태 0.
  void reset();
  /// 새 목표 (에피소드 경계): 은닉 상태·이탈 상태만 0 (속도 명령은 연속).
  void resetEpisode();
  TickResult tick(TickInput & in);
  /// need_dwa 틱: 호출자가 만든 명령을 직전에 낸 명령 (lastV, lastW) 기준 계약 동적 창
  /// v ∈ [lastV − a_max·Δt_c, min(v_cap, lastV + a_max·Δt_c)] (상한 < 하한이면 하한: 최대 감속),
  /// ω ∈ [lastW − α_max·Δt_c, lastW + α_max·Δt_c] ∩ [−ω_max, ω_max] 로 잘라 기록하고 돌려준다.
  /// 비유한 요청은 brakeStep(). Δt_c = control_period, v_cap = 이번 틱 입력의 v_cap.
  FallbackCommand commitDwa(double v, double w);
  /// 계약 제동 규칙 한 틱: v 를 0 쪽으로 a_max·Δt_c, ω 를 0 쪽으로 α_max·Δt_c (DWA 폴백 끔·비유한).
  FallbackCommand brakeStep() const;

  double vCmd() const {return v_cmd_;}
  double wCmd() const {return w_cmd_;}
  const std::array<Accel, contract::kNPrevActions> & prevActions() const {return prev_;}
  const DevState & dev() const {return dev_;}
  const EngineConfig & config() const {return cfg_;}
  /// 공칭 틱 수 (decision_period / control_period — 로그용. 결정 자체는 sim 시각 격자로 한다)
  int ticksPerDecision() const {return ticks_per_decision_;}
  bool inHandover() const {return in_handover_;}
  double lastV() const {return v_out_;}
  double lastW() const {return w_out_;}

private:
  enum class Kind {kNone, kPolicy, kDwa, kAlign};
  /// 결정 주기를 닫는다. elapsed = 그 주기의 실제 sim 길이 (DWA 주기의 실행 가속도에 쓴다).
  void closePeriod(double elapsed);
  /// dev_dt = 직전 결정 뒤 실제 sim 경과 (이탈 상태 타이머).
  void startPeriod(TickInput & in, TickResult & res, double dev_dt);
  int selectC(const PolicyOutput & out, const MaskResult & m, TickResult & res);
  /// 결정 뒤 sim 경과 t 의 램프 명령 (정책: t > 0.1 이면 가속도를 붙잡고 속도 상자로 자른다, 정렬:
  /// 0.1 에서 멈춤).
  Command rampAt(double t) const;

  EngineConfig cfg_;
  PolicyInterface * policy_;
  MaskBuilder mask_builder_;
  int ticks_per_decision_{2};

  bool synced_{false};
  double last_tick_{-1e9};
  double dt_est_{0.05};           ///< [s] 다음 틱까지의 추정 (양수 틱 간격의 지수 이동 평균, α 0.2)
  double t_emit_{0.0};            ///< [s] 이번 결정 주기의 마지막 램프 시각 (되돌리지 않음)
  double period_start_{0.0};      ///< [s] 지금 결정 주기의 결정 틱 시각
  double next_boundary_{0.0};     ///< [s] 다음 결정 경계 (sim 시각 격자)
  double last_period_{0.0};       ///< [s] 직전에 닫힌 결정 주기의 길이
  bool in_handover_{false};
  Kind kind_{Kind::kNone};
  double v_cmd_{0.0};             ///< 이번 결정 주기 시작 명령 (= 직전 주기 끝 명령)
  double w_cmd_{0.0};
  Command target_;                ///< 정책·정렬 주기의 끝 명령과 실행 가속도
  double v_out_{0.0};             ///< 마지막으로 낸 명령
  double w_out_{0.0};
  double v_cap_{contract::kVMaxOp};  ///< 마지막 틱 입력의 v_cap (commitDwa 의 상한)
  double v_hi_{contract::kVMaxOp};   ///< 이번 결정 주기의 속도 상자 상한 min(v_cap, v_max_hw)
  std::array<Accel, contract::kNPrevActions> prev_{};
  DevState dev_;
  int prev_cell_{-1};
  TickResult period_info_;        ///< 결정 틱의 진단을 주기 안 다른 틱에도 싣는다
};

}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__CORE__DECISION_ENGINE_HPP_
