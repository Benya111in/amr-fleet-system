// 속도 프로파일러 + PID 속도 제어 — ROS 비의존 코어 (velocity_profiler_node 의 알고리즘 부분).
//
//   cmd_vel_nav (v*, ω*) ──► 저크 제한 필터(선속도) ──► v_ref
//                          └► 곡률 보존 ω_ref = κ·v_ref, κ = ω*/v* (|v*|
//                          > v_eps), α_max 로 변화율 제한
//                             (|v*| ≤ v_eps: 제자리 회전 → 각속도 저크 제한 필터)
//   (v_ref, ω_ref) ──► 피드포워드 + PI(모델 추종) ──► cmd_vel_smoothed
//     u = r + PI(y_m − y),  y_m = 기준 모델 M(s) = e^{−T_d s}/(T_m s + 1) 에 r 을 넣은 예상 응답,
//     y = odometry/filtered 측정. 플랜트가 모델과 같으면 y_m −
//     y ≈ 0 이라 PI 가 개입하지 않고(스텝 오버슈트 0),
//     적재·슬립·서보 포화로 생긴 차이만 보정한다 (docs/algorithms/path_tracking.md 게인 튜닝).
// 적재 질량 훅: setPayloadScale(s) 로 선가속·저크·각가속 한계에
//   s = m_empty / (m_empty + m_payload) 를 곱한다.
// 정지 착지: 목표·기준이 0 이 되는 순간 남은 PI 보정(적재·슬립 플랜트에서 수 cm/s)을
//   바로 0 으로 끊으면 명령에 계단이 생긴다 → 직전 출력 상태(v, a)에서 같은 저크 한계
//   필터로 0 에 착지시킨 뒤 정확히 0.
// 재동기화: 필터 기준과 측정이 resync 임계보다 벌어지면(안전 노드 감속·E-stop 개입 등) 필터 상태를
//   측정값으로 맞추고 적분기를 리셋한다 — 오래된 기준으로 급가속하는 것을 막는다.
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
  double max_linear_correction{0.2};         // [m/s] PID 보정 폭 (피드포워드 기준 ±)
  double max_angular_correction{0.3};        // [rad/s]
  double resync_threshold_v{0.4};            // [m/s]
  double resync_threshold_w{0.6};            // [rad/s]
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

  /// 한 주기. has_meas 가 false 면 PID 없이 피드포워드만.
  ProfilerOutput update(
    double v_target, double w_target, bool has_meas, double v_meas, double w_meas, double dt);

  const JerkLimitedFilter & linearFilter() const {return lin_;}
  const JerkLimitedFilter & angularFilter() const {return ang_;}

private:
  void applyLimits();
  void resetModels(double v, double w);

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
  JerkLimitedFilter stop_;      // 정지 착지 필터 (출력 상태에서 0 으로)
  bool stopping_{false};
  double out_v1_{0.0};          // 직전 두 출력 (출력 가속도 추정)
  double out_v0_{0.0};
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__VELOCITY_PROFILER_HPP_
