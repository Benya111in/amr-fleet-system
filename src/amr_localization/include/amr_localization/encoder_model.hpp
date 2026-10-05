// 바퀴 인코더 에뮬레이터 (명세 4.1 "Wheel Encoder: 틱 분해능 4096 이상, 슬립 노이즈 적용").
//
// Gazebo DiffDrive/JointStatePublisher 는 잡음 없는 조인트 각을 준다. 이 모델이 실제
// 인코더처럼 다음 두 단계를 적용한다 (config/sensors.yaml wheel_encoder):
//   1) 양자화: 다회전 누적 조인트 각 φ → n = round(φ N / 2π) 틱 (int64, 4 h 연속에도 오버플로 없음)
//   2) 거리당 슬립 잡음: Δφ_meas = Δn (2π/N) + η,  η ~ N(0, σ_s² φ_ref |Δn (2π/N)|)
//      분산이 회전량에 비례하므로 누적 분산은 굴러간 거리에만 의존하고 샘플 주기·속도와 무관하다
//      (φ_ref = ℓ_ref / r: σ_s 를 정의한 기준 굴림 거리. 기본 ℓ_ref 0.01 m
//       = 0.5 m/s × 50 Hz 한 주기 — 이 기준에서 이전의 주기당 곱셈 모델 (1 + N(0, σ_s)) 과 같다).
// 1 틱 = 2π/4096 rad = r·1.534e-3 m (r = 0.0825 → 0.127 mm).
//
// 다회전 카운터: Gazebo 조인트 각은 감기지 않는 누적각이라 주기 차를 그대로 더한다 (접지 않는다 —
// 이전 구현은 차를 (−π, π] 로 접어 샘플 간 공백이 π/ω 를 넘으면 한 바퀴(2πr = 0.518 m)를 잃었다).
// (−π, π] 로 감겨 오는 입력(wrapped_input)은 조인트 속도 힌트 ω Δt 에 가장 가까운 2π 분기를 고르고,
// 힌트가 없는데 공백이 π/ω_max 를 넘으면 분기를 정할 수 없으므로 ambiguous 로 표시한다
// (WheelOdometry 가 그 구간 분산을 한 바퀴 크기로 키운다). 연속 입력에서도 |Δφ| > ω_max Δt 인
// 불가능한 점프(조인트 리셋 등)는 ambiguous 로 표시한다.

#ifndef AMR_LOCALIZATION__ENCODER_MODEL_HPP_
#define AMR_LOCALIZATION__ENCODER_MODEL_HPP_

#include <cstdint>
#include <limits>
#include <random>

namespace amr_localization
{

/// 인코더 모델 파라미터.
struct EncoderParams
{
  int ticks_per_revolution{4096};  ///< N [tick/rev]
  double slip_noise_stddev{0.01};  ///< σ_s [무차원] — 기준 회전각 φ_ref 굴림에서의 상대 σ
  /// φ_ref [rad]: 슬립 분산 σ_s² φ_ref |Δφ| 의 기준 회전각 (= ℓ_ref / r, 기본 0.01 m / 0.0825 m)
  double slip_reference_angle{0.01 / 0.0825};
  bool quantize{true};             ///< false 면 양자화 생략 (분석용)
  bool slip_noise{true};           ///< false 면 슬립 잡음 생략 (분석용)
  bool wrapped_input{false};       ///< 조인트 각이 (−π, π] 로 감겨 오는 입력 (Gazebo 는 false)
  double max_wheel_speed{30.0};    ///< ω_max [rad/s] 타당성·분기 모호 판정 한계
};

/// 한 번의 갱신 결과.
struct EncoderReading
{
  std::int64_t ticks{0};         ///< 누적 틱 카운트
  std::int64_t delta_ticks{0};   ///< 직전 갱신 대비 틱 변화
  double true_delta_angle{0.0};  ///< 잡음 없는 조인트 각 변화 [rad] (평가/테스트용)
  double delta_angle{0.0};       ///< 측정 회전각 증분 = 양자화 + 슬립 잡음 [rad]
  double slip_factor{1.0};       ///< 측정 / 양자화 증분 (= 1 + ε, 증분 0 이면 1)
  bool valid{false};             ///< 첫 샘플(기준점 설정)이면 false
  bool ambiguous{false};         ///< 2π 분기 모호 또는 불가능한 점프 (이 증분을 믿지 말 것)
};

/// 바퀴 1개 인코더 에뮬레이터. 난수 seed 를 고정하면 재현 가능하다.
class WheelEncoderModel
{
public:
  WheelEncoderModel(const EncoderParams & params, std::uint64_t seed);

  /// 누적 조인트 각 [rad] 을 넣어 측정값을 얻는다. 첫 호출은 기준점만 잡는다 (valid=false).
  /// velocity [rad/s] 는 조인트 속도 힌트 (없으면 NaN),
  /// dt [s] 는 직전 샘플과의 시간 간격 (모르면 0).
  EncoderReading update(
    double joint_angle, double velocity = std::numeric_limits<double>::quiet_NaN(),
    double dt = 0.0);

  /// 기준점을 지운다 (다음 update 가 다시 첫 샘플).
  void reset();

  /// 1 틱 각 2π/N [rad].
  double tickAngle() const {return tick_angle_;}

  /// 파라미터.
  const EncoderParams & params() const {return params_;}

  /// 현재 누적 틱.
  std::int64_t ticks() const {return ticks_;}

  /// 누적(다회전) 조인트 각 [rad].
  double unwrappedAngle() const {return unwrapped_angle_;}

private:
  std::int64_t quantizeAngle(double angle) const;
  double unwrapDelta(double raw_delta, double velocity, double dt, bool & ambiguous) const;

  EncoderParams params_;
  double tick_angle_;
  bool initialized_{false};
  double last_raw_angle_{0.0};
  double last_velocity_{std::numeric_limits<double>::quiet_NaN()};
  double unwrapped_angle_{0.0};
  std::int64_t ticks_{0};
  std::mt19937_64 rng_;
  std::normal_distribution<double> noise_{0.0, 1.0};
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__ENCODER_MODEL_HPP_
