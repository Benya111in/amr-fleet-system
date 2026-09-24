// Dynamic Window Approach — ROS 비의존 코어 (명세 4.4 "DWA 핵심 로직 직접 구현").
//
// 한 제어 주기(Δt_c = 1/20 s)의 절차 (Fox, Burgard & Thrun 1997; docs/algorithms/dwa.md):
//   0) 좁은 곳 경로 재중심(옵션): 기준 경로 점의 비용이 recenter_min_cost 이상이고
//      법선 ±recenter_max_shift 안 비용이 양쪽 끝보다 낮은 골(valley)이면 그 골 바닥으로 옮긴다
//      (이동량 이동평균). 지역 코스트맵은 로봇과 같은 odom 프레임이라 위치추정 오차(전역 경로가
//      0.60 m 통로 중심에서 2–4 cm 비낌, dwa.md §1.7)와 무관하게 실제 통로 중심을 따른다.
//      한쪽만 막힌 곳(골 아님)과 목표 근처는 옮기지 않는다.
//   1) 동적 창  V_d = [v_c − a_dec·Δt_c, v_c + a_acc·Δt_c] × [ω_c − α·Δt_c, ω_c + α·Δt_c]  ∩  V_s
//      창 중심 (v_c, ω_c) = 직전 명령 (측정값과 reset 임계
//      이상 벌어지면 측정값). 측정값 중심이면 하류
//      저크 필터 지연 때문에 실효 가속이 한계의 ~1/3 로 떨어진다(연구 브리프 local-planning §3.2).
//   2) 속도 샘플링  N_v × N_ω 격자 + 제동 후보(최대 감속, ω 를 0 쪽으로).
//   3) 궤적 시뮬레이션  (v, ω) 일정 원호의 정확 적분, T_sim(v)
//   = clip(|v|/a + 0.5, T_min, T_max), 간격 sim_dt.
//   4) 충돌 검사  풋프린트(FootprintChecker) — 목표 너머(d_goal + margin)는 검사하지 않는다.
//      T_sim(v)·v ≥ s_stop(v) 이므로 "검사 구간 무충돌" 이면 Fox
//      의 허용 속도(V_a: 충돌 전 정지 가능)도 성립.
//   5) 동적 장애물: 예측 통로 밖 가상 정지선(core::evaluateYield) 으로 v_cap 을 낮추고,
//      VO 원뿔 안의 샘플 제외(옵션) + 예측 충돌 시각 TTC₀ 비용.
//      VO 가 창 전체를 덮으면(포화) VO 를 끄지 않고 VO 진입 시각이 가장 늦은 샘플을 고른다.
//   6) 비용(최소화, 항별 [0,1] 정규화) J = w_h J_head + w_c J_clear
//   + w_v J_vel + w_p J_path + w_o J_osc + w_d J_dyn + w_f J_off
//        J_head  = |wrap(atan2(target − p_E) − θ_E)| / π,  p_E = 롤아웃의 path_eval_time
//                  지점, target = 경로 위 p_E 투영점에서 ℓ(v) 앞
//        J_clear = max_k max(0, c(p_k) − c(g_k)) / 252,  g_k = 같은 호길이의 기준 경로 점 (경로
//        자체 비용은 벌점 아님)
//        J_vel   = |v_des − v| / (v_max − v_min),  v_des = min(v_cap, d_stop⁻¹(d_goal))
//                  (d_stop: 저크·지연 포함 정지거리, SpeedProfile::maxSpeedForStop)
//        J_path  = mean_{k ≤ k_E} min(|e⊥(p_k)| / d_band, 1)  (추종 항은 앞 path_eval_time 만,
//                  충돌·여유는 롤아웃 전체 — docs/algorithms/dwa.md §1.5)
//        J_osc   = 제자리 회전 방향 반전 / 전후진 반전 플래그
//        J_dyn   = min(1, max(0, |v| − v_safe(TTC₀)) / (v_max − v_min)
//                         + λ·max(0, 1 − TTC₀ / T_pred)),
//                  v_safe(TTC₀) = a·max(0, TTC₀ − T_c − a/(2j)) (예측 접촉 전에 설 수 있는 속도)
//        J_off   = min(1, max(0, e_max − max(d_off, |e⊥(로봇)|)) / d_off_band)  — 경로 이탈 한계를
//                  넘는 롤아웃의 추가 벌점. J_path 는 d_band 에서 포화해 멀리서는 되돌리는 힘이
//                  없다 (리뷰 결함 08-2: 시나리오 08 최대 이탈 6.45 m). 로봇이 이미 밖이면 그
//                  거리까지는 벌하지 않아(단조 감소 포락선) 복귀를 막지 않는다.
//   7) 선택: 충돌 없고 VO 밖인 샘플 중 비용 최소 → 없으면(VO 포화) 충돌 없는 샘플 중
//      VO 진입이 가장 늦은 것 → 그것도 없으면 제동 후보.
#ifndef AMR_NAVIGATION__CORE__DWA_HPP_
#define AMR_NAVIGATION__CORE__DWA_HPP_

#include <cstddef>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include "amr_navigation/core/crossing_yield.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/velocity_obstacle.hpp"

namespace amr_navigation
{
namespace core
{

struct DwaLimits
{
  double max_vel_x{1.0};        // [m/s] 운용 최고 속도 (≤ robot_params limits.max_linear_velocity)
  double min_vel_x{0.0};        // [m/s] 음수면 후진 허용
  double max_vel_theta{1.5};    // [rad/s]
  double acc_lim_x{1.0};        // [m/s²]
  double decel_lim_x{1.0};      // [m/s²] (양수)
  double acc_lim_theta{2.0};    // [rad/s²]
  double jerk_lim_x{2.0};       // [m/s³] 정지거리 계산용
};

struct DwaWeights
{
  double heading{0.6};
  double clearance{1.0};
  double velocity{0.4};
  double path{2.0};
  double oscillation{0.2};
  double dynamic{1.5};
  double off_path{3.0};     // w_f: 이탈 한계 초과 벌점 (다른 어느 항보다 커야 되돌린다)
  double escape{2.5};       // w_e: 이미 들어선 통로(kCommitted)에서 빠져나가는 후보에 주는 이득
};

struct DwaConfig
{
  DwaLimits limits;
  DwaWeights weights;
  double control_period{0.05};      // [s] Δt_c (controller_frequency 의 역수)
  int vx_samples{11};
  int vth_samples{21};
  double sim_dt{0.1};               // [s]
  double sim_time_min{1.5};         // [s]
  double sim_time_max{2.5};         // [s]
  double commit_time{0.2};          // [s] T_c: 명령 유지 후 제동 시작 (정지거리 계산)
  double approach_latency{0.3};     // [s] 목표 접근 속도 v_des = d_stop⁻¹(d_goal) 의 명령 지연
  double window_reset_v{0.3};       // [m/s] 직전 명령과 측정값 차가 이보다 크면 창 중심 = 측정값
  double window_reset_w{0.5};       // [rad/s]
  double heading_lookahead_gain{0.4};    // ℓ = clip(gain·v + offset, min, max) [m]
  double heading_lookahead_offset{0.3};
  double heading_lookahead_min{0.4};
  double heading_lookahead_max{1.2};
  double path_band{0.8};            // [m] J_path 정규화 폭
  double max_path_offset{0.9};      // [m] J_off 가 걸리기 시작하는 이탈 (≤ 0 이면 끔).
                                    //     명세 4.7 이탈 한계 1.0 m 안쪽
  double off_path_band{1.0};        // [m] J_off 정규화 폭
  double path_eval_time{0.8};       // [s] 헤딩·경로 항을 볼 롤아웃 앞부분 (≤ 0 이면 전체)
  double goal_align_distance{0.15};   // [m] 이 안에서는 목표 방향 정렬(제자리 회전)
  double goal_overshoot_margin{0.1};  // [m] 목표 너머 충돌 검사 여유
  double oscillation_w_eps{0.05};   // [rad/s]
  // 동적 장애물
  bool use_dynamic_obstacles{true};
  bool use_velocity_obstacles{true};
  double prediction_time{5.0};      // [s] TTC₀ 지평 (평균 예측)
  double robot_radius{0.361};       // [m] 로봇 덮개 원 (외접 반경)
  double dynamic_margin{0.5};       // [m] TTC₀ 계산 여유 (critical zone 거리)
  double vo_time_horizon{2.0};      // [s] τ
  double vo_margin{0.3};            // [m] VO 합성 반경 여유 (emergency_stop_distance)
  double vo_max_range{6.0};         // [m] 이 거리 밖 장애물은 VO 에서 제외
  double dynamic_speed_threshold{0.2};  // [m/s] 이보다 느린 트랙은 정적(코스트맵이 처리)
  double dynamic_steer_gain{0.3};   // λ: J_dyn 의 TTC 보조항 (같은 속도에서 TTC 가 긴 방향 선호)
  // [s] VO 포화 시 진입 시각이 이만큼 안이면 같다고 보고 비용으로 고른다
  double vo_time_tie{0.02};
  // 횡단 양보 (가상 정지선, core::evaluateYield — 명세 4.7 "접촉 0")
  bool yield_crossing{true};
  double yield_corridor_margin{0.50};   // [m] 통로 반폭 여유 (= dynamic_margin, critical zone)
  double yield_stop_margin{0.25};       // [m] 통로 입구 앞 정지선 여유
  double yield_clear_margin{1.0};       // [s] 먼저 빠져나간다고 볼 시간 여유
  double yield_horizon{8.0};            // [s] 장애물 예측 지평 (통로 길이 = |u|·horizon)
  double yield_lookahead{4.0};          // [m] 경로에서 교차 구간을 찾는 최대 거리
  double yield_max_zone{6.0};           // [m] 교차 구간 길이 상한 (넘으면 나란한 주행)
  // 통로 안(kCommitted)에서 정지해 버리면 장애물이 그대로 걸어 들어온다 (시나리오 08 실측:
  // 접촉 4건 모두 로봇 속도 0 · yield_state=committed · 차선 가로 거리 |β| < 0.65 m).
  // 앞은 VO 가 막으므로 나가는 길은 뒤뿐이다 — 그 상태에서만 후진 샘플을 열고,
  // 통로 축에서 멀어지는 후보에 이득을 준다.
  double yield_escape_speed{0.25};      // [m/s] kCommitted 에서 허용하는 후진 속도 (0 = 끔)
  double yield_escape_drift{0.15};      // [m] 이탈이 지금보다 이만큼 넘게 늘면 이득 없음
  double yield_escape_v_meas{0.05};     // [m/s] 측정 속도가 이보다 낮을 때만 (정지·후진 중)
  // 좁은 곳 경로 재중심 (0 단계)
  bool recenter_narrow{true};
  double recenter_min_cost{100.0};  // 이 비용 이상인 경로 점만 (지역 s = 3: 장애물 ≈ 0.5 m 이내)
  double recenter_max_shift{0.10};  // [m] 법선 방향 탐색 폭 = 최대 이동 (0.60 m 통로 측면 여유)
  double recenter_step{0.025};      // [m] 탐색 간격 (지역 코스트맵 해상도)
  int recenter_smooth{4};           // 이동량 이동평균 반폭 [경로 점]
  double recenter_goal_keep{0.5};   // [m] 목표에서 이 거리 안은 옮기지 않는다
};

struct DwaCostTerms
{
  double heading{0.0};
  double clearance{0.0};
  double velocity{0.0};
  double path{0.0};
  double oscillation{0.0};
  double dynamic{0.0};
  double off_path{0.0};
  double escape{0.0};       // 통로 축까지의 거리로 정규화 (1 = 축 위, 0 = 통로 밖)
};

struct DwaCandidate
{
  double v{0.0};
  double w{0.0};
  std::vector<Pose2D> poses;    // 시뮬레이션 자세 (poses[0] = 현재)
  DwaCostTerms terms;
  double cost{std::numeric_limits<double>::infinity()};
  bool collision{false};
  bool vo_rejected{false};
  bool is_brake{false};
  double ttc{std::numeric_limits<double>::infinity()};       // 원호 예측 TTC₀ (R_d)
  double vo_time{std::numeric_limits<double>::infinity()};   // VO 진입 시각 (R, τ 안), 없으면 +inf
  double max_cte{0.0};          // 평가 구간(k ≤ k_E) 롤아웃의 최대 |경로 수직 거리| [m]
};

struct DwaInput
{
  Pose2D pose;                          // 코스트맵 프레임 로봇 자세
  double v_meas{0.0};
  double w_meas{0.0};
  bool has_last{false};
  double v_last{0.0};
  double w_last{0.0};
  const std::vector<Pose2D> * path{nullptr};   // 코스트맵 프레임 기준 경로 (로봇 근처부터)
  bool path_end_is_goal{true};
  double speed_limit{std::numeric_limits<double>::infinity()};   // [m/s] 외부 속도 제한
  std::vector<DynamicObstacle> obstacles;       // 코스트맵 프레임, 현재 시각 기준
  // 직전 주기에 적용된 횡단 양보 속도 상한. 상한이 **오르는** 속도를 로봇 가속 한계로 묶는 데
  // 쓴다 — 추적기 진행각 잡음(실측 ±28°/0.5 s)으로 통로 예측이 흔들리면 양보 판단이 몇 주기씩
  // clear 로 뒤집히고, 그때마다 상한이 ∞ 로 풀려 로봇이 다시 가속한다 (통합 08 실측). 내리는
  // 쪽은 묶지 않는다. 로봇이 낼 수 없는 가속을 막는 것이라 이 제한으로 더 느려지지 않는다.
  double yield_limit_last{std::numeric_limits<double>::infinity()};
};

struct DwaWindow
{
  double v_lo{0.0};
  double v_hi{0.0};
  double w_lo{0.0};
  double w_hi{0.0};
};

struct DwaResult
{
  bool found{false};
  double v{0.0};
  double w{0.0};
  DwaCandidate best;
  DwaWindow window;
  std::size_t n_samples{0};
  std::size_t n_valid{0};
  std::size_t n_collision{0};
  std::size_t n_vo_rejected{0};
  bool vo_saturated{false};      // VO 가 충돌 없는 샘플을 모두 덮어 VO 진입 최지연 샘플을 골랐음
  std::size_t n_recentered{0};   // 0 단계에서 골 바닥으로 옮긴 기준 경로 점 수
  double d_goal{0.0};
  YieldResult yield;             // 횡단 양보 판정 (가상 정지선)
  double v_cap{0.0};             // 이 주기에 실제로 쓴 속도 상한 [m/s]
};

/// 자세의 풋프린트 비용(0~253) 또는 충돌 시 음수.
using FootprintCostFn = std::function<double (const Pose2D &)>;
/// 점의 코스트맵 셀 비용 (J_clear 용).
using PointCostFn = std::function<double (double, double)>;

class DwaPlanner
{
public:
  explicit DwaPlanner(const DwaConfig & config = DwaConfig());

  void setConfig(const DwaConfig & config) {config_ = config;}
  const DwaConfig & config() const {return config_;}

  DwaResult compute(
    const DwaInput & in, const FootprintCostFn & footprint_cost,
    const PointCostFn & point_cost) const;

  // --- 단계별 공개 함수 (단위 테스트용) ---
  /// 창 중심 (v_c, ω_c) 에서의 동적 창. v_cap = min(v_max, 외부 제한).
  DwaWindow dynamicWindow(double v_c, double w_c, double v_cap) const;
  /// 창 격자 샘플 + 제동 후보(마지막 원소).
  std::vector<std::pair<double, double>> sampleVelocities(
    const DwaWindow & win, double v_c, double w_c) const;
  double simTime(double v) const;
  /// 원호 롤아웃: poses[0] = start, 간격 sim_dt, 총 n = ceil(T/dt) 스텝.
  std::vector<Pose2D> rollout(const Pose2D & start, double v, double w, double T) const;
  /// 저크 제한 정지거리: 명령 유지 T_c 후 가속 0 에서 저크 j 로 −a 까지 램프, 등감속 정지.
  static double stoppingDistance(double v, double a, double j, double t_c);
  /// 창 중심 선택 규칙.
  std::pair<double, double> windowCenter(const DwaInput & in) const;
  /// 설정에서 뽑은 횡단 양보 설정 (core::evaluateYield 용).
  YieldConfig yieldConfig() const;
  /// 0 단계: 좁은 곳(양쪽이 막힌 골) 기준 경로 점을 코스트맵 골 바닥으로 옮긴 복사본.
  /// shifted: 옮긴 점 수.
  std::vector<Pose2D> recenterPath(
    const std::vector<Pose2D> & path, const PointCostFn & point_cost, bool end_is_goal,
    std::size_t * shifted = nullptr) const;

private:
  DwaConfig config_;
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__DWA_HPP_
