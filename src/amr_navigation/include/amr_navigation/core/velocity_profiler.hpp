// 속도 프로파일러 + PID 속도 제어 — ROS 비의존 코어 (velocity_profiler_node 의 알고리즘 부분).
//
//   cmd_vel_nav (v*, ω*) ──► 저크 제한 필터(선속도) ──► v_ref, a_ref
//                          └► 곡률 보존 ω_ref = κ·v_ref, κ = ω*/v* (|v*|
//                          > v_eps), α_max 로 변화율 제한
//                             (|v*| ≤ v_eps: 제자리 회전 → 각속도 저크 제한 필터)
//   PI 보정 c = PI(y_m − y) (|c| ≤ c_max),
//     y_m = 기준 모델 M(s) = e^{−T_d s}/(T_m s + 1) 에 v_ref 를 넣은 예상 응답,
//     y = odometry/filtered 측정 (둘 다 같은 1차 저역 T_f 를 거친다 — 잡음이 kp 로 새지 않게).
//     플랜트가 모델과 같으면 y_m − y ≈ 0 이라 PI 가 개입하지 않고(스텝 오버슈트 0),
//     적재·슬립·서보 포화로 생긴 차이만 보정한다 (docs/algorithms/path_tracking.md 게인 튜닝).
//   출력 성형 (선속도): 최종 명령 u = v_ref + c 를 상태 (u, a_u) 의 저크 제한 추종으로 만든다 —
//     목표 u* = v_ref + c 의 가속도 a_ref 를 피드포워드로 두고 상대 오차 (u* − u, a_u − a_ref)
//     를 JerkLimitedFilter 와 같은 착지식으로 없애되 |a_u| ≤ a_max, |Δa_u| ≤ j·dt 로 자른다.
//     → 기준만 움직이면 u = v_ref 그대로(지연 0), 보정은 남은 가속·저크 여유 안에서만 더해진다.
//     그래서 **보내는 명령** 의 저크가 j_max 이하다 (리뷰: 기준 + PI 합의 저크 p99 29.6 m/s³).
//     PI 적분기는 실제로 더해진 보정 u − v_ref 로 역계산(back-calculation)해
//     성형이 붙잡은 만큼 와인드업하지 않는다.
//   측정 신선도: 새 측정이 없는 주기(odometry 반복·지연)에는 PI 를 갱신하지 않고 직전 보정 유지.
// 적재 질량 훅: setPayloadScale(s) 로 선가속·저크·각가속 한계에
//   s = m_empty / (m_empty + m_payload) 를 곱한다.
// 정지: 목표·기준이 0 이 되면 PI 를 끄고 보정 목표 0 → 성형이 남은 보정을 저크 한계로 0 에
//   착지시킨 뒤 정확히 0.
// 재동기화 (명령이 실행되지 않음): 필터 상태를 측정값으로 맞추고 적분기·보정을 리셋한다 —
//   (1) |v_ref − y| > resync_threshold (즉시, E-stop·급정지), 또는
//   (2) 측정 기반 붙잡힘 판정이 hold_time 동안 이어짐: 모델 응답 y_m 과 측정 차 > hold_error
//       이면서 y < 0.6·y_m (느린·슬립 플랜트는 제외), 또는 y ≈ 0 인데 y_m > 0.04 (저속 대기)
//       — safety_node 가 명령을 붙잡거나 깎음.
//       리뷰: 0.4 m/s 임계만으로는 저속 대기 중 PI 가 +0.2 까지 와인드업.
//   주기가 들쭉날쭉하면 |a_k − a_{k−2}| ≤ 2 j·dt_k 도 지켜 실제 간격 기준 수치 저크도 j 이하.
//   재동기화 순간의 명령 계단은 실행되지 않는 명령이므로 저크 보장에서 뺀다
//   (ProfilerOutput::resynced 로 표시).
#ifndef AMR_NAVIGATION__CORE__VELOCITY_PROFILER_HPP_
#define AMR_NAVIGATION__CORE__VELOCITY_PROFILER_HPP_

#include "amr_navigation/core/jerk_limiter.hpp"
#include "amr_navigation/core/pid.hpp"

namespace amr_navigation
{
namespace core
{

struct VelocityProfilerConfig
{
  JerkLimits linear{-0.5, 2.0, 1.0, 2.0};    // v_min, v_max, a_max, j_max
  JerkLimits angular{-1.5, 1.5, 2.0, 6.0};   // ω_min, ω_max, α_max, 각저크
  bool preserve_curvature{true};
  double curvature_min_speed{0.05};          // [m/s] v_eps
  bool use_pid{true};
  PidConfig pid_linear;
  PidConfig pid_angular;
  bool use_reference_model{true};            // false 면 PI 오차 = r − y (고전 2-자유도)
  double model_time_constant{0.08};          // [s] T_m (DiffDrive 속도 서보 + EKF 근사)
  double model_delay{0.04};                  // [s] T_d
  double nominal_period{0.02};               // [s] 50 Hz (모델 지연 스텝 수 계산)
  double measurement_filter_time{0.04};      // [s] T_f: 측정·모델 응답 공통 1차 저역 (0 = 끔)
  double max_linear_correction{0.2};         // [m/s] PID 보정 폭 (피드포워드 기준 ±)
  double max_angular_correction{0.3};        // [rad/s]
  double resync_threshold_v{0.4};            // [m/s] 즉시 재동기화
  double resync_threshold_w{0.6};            // [rad/s]
  double hold_error_v{0.15};                 // [m/s] 모델 응답 − 측정이 이보다 크게
  double hold_time{0.3};                     // [s] 이만큼 이어지면 붙잡힌 것으로 보고 재동기화
  // [m/s], [rad/s] 목표·기준이 모두 이 안이면 정확히 0 출력
  double stop_deadband{1e-3};
};

struct ProfilerOutput
{
  double v{0.0};        // 최종 명령
  double w{0.0};
  double v_ref{0.0};    // 프로파일 기준 (PID 전)
  double w_ref{0.0};
  double a_ref{0.0};    // 선가속도 기준
  double a_out{0.0};    // 최종 선속도 명령의 가속도 상태
  double correction{0.0};   // 최종 − 기준 (선속도)
  bool resynced{false};
};

class VelocityProfiler
{
public:
  explicit VelocityProfiler(const VelocityProfilerConfig & config = VelocityProfilerConfig());

  void setConfig(const VelocityProfilerConfig & config);
  const VelocityProfilerConfig & config() const {return config_;}

  /// 적재 질량 스케일 s ∈ (0, 1].
  void setPayloadScale(double s);
  double payloadScale() const {return payload_scale_;}

  void reset(double v = 0.0, double w = 0.0);

  /// 한 주기. has_meas 가 false 면 PID 없이 피드포워드만 (보정은 저크 한계로 0 에 착지).
  /// fresh = false 면 측정이 직전 주기와 같은 표본 → PI 를 갱신하지 않고 직전 보정 유지.
  ProfilerOutput update(
    double v_target, double w_target, bool has_meas, double v_meas, double w_meas, double dt,
    bool fresh = true);

  const JerkLimitedFilter & linearFilter() const {return lin_;}
  const JerkLimitedFilter & angularFilter() const {return ang_;}
  int resyncCount() const {return resync_count_;}

private:
  void applyLimits();
  void resetModels(double v, double w);
  void resync(double v, double w);
  /// 선속도 출력 성형 한 스텝: 목표 target (가속도 a_target) 을 (out_v_, out_a_) 가
  /// 저크 제한으로 추종.
  void shapeOutput(double target, double a_target, double dt);

  VelocityProfilerConfig config_;
  double payload_scale_{1.0};
  JerkLimitedFilter lin_;
  JerkLimitedFilter ang_;
  Pid pid_v_;
  Pid pid_w_;
  FirstOrderPlant model_v_;
  FirstOrderPlant model_w_;
  double w_ref_{0.0};
  bool curvature_mode_{false};
  double out_v_{0.0};           // 최종 선속도 명령 상태 (u, a_u)
  double out_a_{0.0};
  double out_a_prev_{0.0};      // 한 스텝 전 가속도 상태 (주기 들쭉날쭉 시 수치 저크 가드)
  bool has_prev_a_{false};
  double corr_v_{0.0};          // 보정 목표 (PI 출력, 신선하지 않은 주기에는 유지)
  double corr_w_{0.0};
  double filt_y_{0.0};          // 측정 저역 (선·각)
  double filt_ym_{0.0};
  double filt_yw_{0.0};
  double filt_ymw_{0.0};
  bool filt_init_{false};
  double hold_elapsed_{0.0};
  int resync_count_{0};
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__VELOCITY_PROFILER_HPP_
