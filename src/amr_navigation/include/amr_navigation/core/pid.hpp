// PID 속도 제어기 — ROS 비의존 (명세 4.5 "PID 기반 속도 제어기, 게인 튜닝 문서화").
//
// 2-자유도 PID + 피드포워드 + 역계산(back-calculation) 안티와인드업:
//   u_raw = ff + K_p·(b·r − y) + I + K_d·ẏ_f(측정 미분, 1차 저역)
//   u     = clamp(u_raw, u_min, u_max)
//   İ     = K_i·(r − y) + (u − u_raw)/T_t      (T_t ≤ 0
//   이면 조건부 적분: 포화 방향으로는 적분 정지)
// b(setpoint_weight) < 1 이면 기준 스텝에 의한 비례항 킥과 오버슈트가
//   줄어든다(피드포워드가 명목 추종 담당).
// 미분은 측정값에만 걸어 기준 스텝의 미분 킥을 없앤다.
// 튜닝 절차와 1차 지연 플랜트 모델(FirstOrderPlant) 스텝 응답 결과는
//   docs/algorithms/path_tracking.md.
#ifndef AMR_NAVIGATION__CORE__PID_HPP_
#define AMR_NAVIGATION__CORE__PID_HPP_

#include <cstddef>
#include <deque>
#include <limits>

namespace amr_navigation
{
namespace core
{

struct PidConfig
{
  double kp{0.4};
  double ki{2.0};
  double kd{0.0};
  double setpoint_weight{1.0};      // b
  double output_min{-std::numeric_limits<double>::infinity()};
  double output_max{std::numeric_limits<double>::infinity()};
  double tracking_time{0.1};        // T_t [s] 역계산 시정수 (≤ 0 → 조건부 적분)
  double derivative_tau{0.02};      // [s] 측정 미분 저역 필터 시정수
  double integral_limit{std::numeric_limits<double>::infinity()};   // |I| 상한 (보조 가드)
};

class Pid
{
public:
  explicit Pid(const PidConfig & config = PidConfig());

  void setConfig(const PidConfig & config) {config_ = config;}
  const PidConfig & config() const {return config_;}

  void reset(double integral = 0.0);

  /// r: 기준, y: 측정, dt: 주기, ff: 피드포워드. 반환: 포화된 출력.
  double update(double r, double y, double dt, double ff = 0.0);
  /// 출력 한계를 이번 호출에만 따로 준다 (피드포워드 주변의 보정 폭 제한 등).
  double update(double r, double y, double dt, double ff, double out_min, double out_max);

  /// 하류(출력 성형·포화)가 PI 출력과 다른 값을 실제로 냈을 때 그 차 du = u_applied − u 만큼 역계산
  /// (T_t ≤ 0 이면 적분기를 du 방향으로 더 밀지 않도록 아무것도 하지 않는다).
  void backCalculate(double du, double dt);
  /// 적분기에 du 를 그대로 더한다 (추종 시정수 0 의 역계산: 출력이 곧바로 u_applied 가 된다).
  void addToIntegral(double du);

  double integral() const {return integral_;}
  bool saturated() const {return saturated_;}
  double lastUnsaturated() const {return u_raw_;}

private:
  PidConfig config_;
  double integral_{0.0};
  double prev_y_{0.0};
  double d_filt_{0.0};
  bool has_prev_{false};
  bool saturated_{false};
  double u_raw_{0.0};
};

/// 1차 지연 + 순수 지연 플랜트  y' = (K·u(t − T_d) − y)/T_p
///    (DiffDrive 속도 서보 근사, 튜닝·시험용).
class FirstOrderPlant
{
public:
  FirstOrderPlant(double gain, double time_constant, double delay, double dt, double y0 = 0.0);
  /// 입력 u 를 넣고 dt 만큼 전진, 출력 반환.
  double step(double u);
  /// 주기가 흔들리는 경우: 지연은 생성 시의 스텝 수, 1차 지연은 이번 dt 로 적분.
  double step(double u, double dt);
  double output() const {return y_;}
  /// 정상상태 y0 로 초기화 (지연 버퍼 = y0/K 입력 이력).
  void reset(double y0 = 0.0);

private:
  double gain_;
  double tau_;
  double dt_;
  std::size_t delay_steps_;
  double y_;
  std::deque<double> buffer_;
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__PID_HPP_
