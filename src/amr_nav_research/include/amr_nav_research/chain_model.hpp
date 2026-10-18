// F1 사슬 모형 (docs/research/paper-research/prereg_r2.md §0 F1).
//
// 직전 명령 이력을 배포 하류 사슬의 명목 모형에 통과시켜 "지금 측정돼야 할 속도" v̂, ŵ 를 만든다.
//   제어기 명령 (Δt_c 동안 유지)
//     → 프로파일러 기준 (50 Hz): 선속도 = core::JerkLimitedFilter (a_max, j_max)
//       각속도 = |v*| > curvature_min_speed 이면 곡률 보존 ω = κ·v_ref (α·dt 변화율 제한),
//                아니면 각저크 필터 — core::VelocityProfiler::update 의 기준 경로 그대로
//     → 서보 (100 Hz): 순수 지연 model_delay → 1차 model_tau → 가속 제한
//       (tools/chain_sim.cpp Servo, scripts/kinematic_sim.py ServoAxis 와 같은 식)
// 이산화: 한 제어 주기에 서보 round(Δt_c / 0.01) 스텝(20 Hz 면 5)을 돌리고, 프로파일러는 전역
// 서보 스텝 번호가 2 의 배수일 때 (chain_sim 의 k % 2 == 0 과 같은 시각표) 0.02 s 로 갱신한다.
// 모형 시작 시각을 chain_sim 의 k = 0 에 맞춘다 (has_last 가 없는 주기에 측정값으로 초기화).
// 넣지 않은 것 (문서화한 근사):
//   - 프로파일러의 PI 보정·측정 재동기화·붙잡힘 판정. 모형은 **명령만으로** 돈다 — 측정과의
//     괴리를 재는 검출기에 측정을 넣으면 괴리가 모형에 흡수되기 때문이다.
//   - ROS 시험대의 비동기 시각 (프로파일러 노드·kinematic_sim 의 위상).
//     20/50/100 Hz 고정 위상으로 근사한다.
#ifndef AMR_NAV_RESEARCH__CHAIN_MODEL_HPP_
#define AMR_NAV_RESEARCH__CHAIN_MODEL_HPP_

#include <cstdint>
#include <vector>

#include "amr_navigation/core/jerk_limiter.hpp"

namespace amr_nav_research
{

struct ChainModelConfig
{
  double delay{0.04};               ///< [s] 서보 순수 지연 (research.model_delay)
  double tau{0.08};                 ///< [s] 서보 1차 시간 상수 (research.model_tau)
  double servo_dt{0.01};            ///< [s] 서보 스텝 (100 Hz)
  int profiler_every{2};            ///< 프로파일러 갱신 간격 [서보 스텝] (50 Hz)
  amr_navigation::core::JerkLimits lin{-1e9, 1e9, 1.0, 2.0};   ///< 선속도 a_max·j_max
  amr_navigation::core::JerkLimits ang{-1.5, 1.5, 2.0, 6.0};   ///< ω 한계·α_max·각저크
  double curvature_min_speed{0.05};   ///< [m/s] 곡률 보존 모드 하한 (velocity_profiler.yaml)
  double servo_accel_v{1.0};        ///< [m/s²] 서보 가속 제한 (kinematic_sim max_linear_accel)
  double servo_accel_w{2.0};        ///< [rad/s²] (max_angular_accel)
};

struct ChainModelState
{
  bool init{false};
  // 명시적 초기화: ResearchState{} 같은 집합체 초기화에서 explicit 생성자를 거치지 않게
  amr_navigation::core::JerkLimitedFilter lin{amr_navigation::core::JerkLimits()};
  amr_navigation::core::JerkLimitedFilter ang{amr_navigation::core::JerkLimits()};
  double w_ref{0.0};
  bool curvature_mode{false};
  double out_v{0.0};                ///< 프로파일러 출력 (다음 갱신까지 유지)
  double out_w{0.0};
  std::vector<double> buf_v;        ///< 서보 지연 버퍼
  std::vector<double> buf_w;
  double sv{0.0};                   ///< 서보 출력 = 예상 측정
  double sw{0.0};
  std::int64_t step{0};             ///< 전역 서보 스텝 번호
};

/// 측정값 (v, w) 로 정상 상태 초기화 (지연 버퍼도 그 값으로 채운다).
void chainModelInit(ChainModelState * s, const ChainModelConfig & c, double v, double w);

/// 명령 (v_cmd, w_cmd) 를 duration 동안 유지했을 때의 사슬 전진. 서보 스텝 수 = round(duration/dt).
void chainModelAdvance(
  ChainModelState * s, const ChainModelConfig & c, double v_cmd, double w_cmd, double duration);

}  // namespace amr_nav_research

#endif  // AMR_NAV_RESEARCH__CHAIN_MODEL_HPP_
