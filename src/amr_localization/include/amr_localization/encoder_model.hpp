// 바퀴 인코더 에뮬레이터 (명세 4.1 "Wheel Encoder: 틱 분해능 4096 이상, 슬립 노이즈 적용").
//
// Gazebo DiffDrive/JointStatePublisher 는 잡음 없는 조인트 각을 준다. 이 모델이 실제
// 인코더처럼 다음 두 단계를 적용한다 (config/sensors.yaml wheel_encoder):
//   1) 양자화: 누적 조인트 각 φ → n = round(φ N / 2π) 틱 (int64, 4 h 연속에도 오버플로 없음)
//   2) 곱셈 슬립 잡음: Δφ_meas = Δn (2π/N) (1 + ε),  ε ~ N(0, σ_s²)
// 1 틱 = 2π/4096 rad = r·1.534e-3 m (r = 0.0825 → 0.127 mm).
// 조인트 각이 ±π 로 감겨 오는 경우를 대비해 주기 차가 π 를 넘으면 2π 감김으로 보고 풀어 쓴다.

#ifndef AMR_LOCALIZATION__ENCODER_MODEL_HPP_
#define AMR_LOCALIZATION__ENCODER_MODEL_HPP_

#include <cstdint>
#include <random>

namespace amr_localization
{

/// 인코더 모델 파라미터.
struct EncoderParams
{
  int ticks_per_revolution{4096};  ///< N [tick/rev]
  double slip_noise_stddev{0.01};  ///< σ_s [무차원]
  bool quantize{true};             ///< false 면 양자화 생략 (분석용)
  bool slip_noise{true};           ///< false 면 슬립 잡음 생략 (분석용)
};

/// 한 번의 갱신 결과.
struct EncoderReading
{
  std::int64_t ticks{0};         ///< 누적 틱 카운트
  std::int64_t delta_ticks{0};   ///< 직전 갱신 대비 틱 변화
  double true_delta_angle{0.0};  ///< 잡음 없는 조인트 각 변화 [rad] (평가/테스트용)
  double delta_angle{0.0};       ///< 측정 회전각 증분 = 양자화 + 슬립 잡음 [rad]
  double slip_factor{1.0};       ///< 이번 주기 1 + ε
  bool valid{false};             ///< 첫 샘플(기준점 설정)이면 false
};

/// 바퀴 1개 인코더 에뮬레이터. 난수 seed 를 고정하면 재현 가능하다.
class WheelEncoderModel
{
public:
  WheelEncoderModel(const EncoderParams & params, std::uint64_t seed);

  /// 누적 조인트 각 [rad] 을 넣어 측정값을 얻는다. 첫 호출은 기준점만 잡는다 (valid=false).
  EncoderReading update(double joint_angle);

  /// 기준점을 지운다 (다음 update 가 다시 첫 샘플).
  void reset();

  /// 1 틱 각 2π/N [rad].
  double tickAngle() const {return tick_angle_;}

  /// 파라미터.
  const EncoderParams & params() const {return params_;}

  /// 현재 누적 틱.
  std::int64_t ticks() const {return ticks_;}

private:
  std::int64_t quantizeAngle(double angle) const;

  EncoderParams params_;
  double tick_angle_;
  bool initialized_{false};
  double last_raw_angle_{0.0};
  double unwrapped_angle_{0.0};
  std::int64_t ticks_{0};
  std::mt19937_64 rng_;
  std::normal_distribution<double> noise_;
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__ENCODER_MODEL_HPP_
