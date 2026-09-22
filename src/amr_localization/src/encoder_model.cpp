// 바퀴 인코더 에뮬레이터 구현.

#include "amr_localization/encoder_model.hpp"

#include <cmath>
#include <stdexcept>

namespace amr_localization
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
}  // namespace

WheelEncoderModel::WheelEncoderModel(const EncoderParams & params, std::uint64_t seed)
: params_(params),
  tick_angle_(0.0),
  rng_(seed)
{
  if (params_.ticks_per_revolution <= 0) {
    throw std::invalid_argument("ticks_per_revolution must be positive");
  }
  if (params_.slip_noise_stddev < 0.0) {
    throw std::invalid_argument("slip_noise_stddev must be non-negative");
  }
  if (!(params_.slip_reference_angle > 0.0)) {
    throw std::invalid_argument("slip_reference_angle must be positive");
  }
  if (!(params_.max_wheel_speed > 0.0)) {
    throw std::invalid_argument("max_wheel_speed must be positive");
  }
  tick_angle_ = kTwoPi / static_cast<double>(params_.ticks_per_revolution);
}

std::int64_t WheelEncoderModel::quantizeAngle(double angle) const
{
  return static_cast<std::int64_t>(std::llround(angle / tick_angle_));
}

void WheelEncoderModel::reset()
{
  initialized_ = false;
  last_velocity_ = std::numeric_limits<double>::quiet_NaN();
}

double WheelEncoderModel::unwrapDelta(
  double raw_delta, double velocity, double dt, bool & ambiguous) const
{
  ambiguous = false;
  const double reach = params_.max_wheel_speed * dt;   // 공백 동안 가능한 최대 회전
  if (!params_.wrapped_input) {
    // 연속 누적각: 차가 곧 회전량. 불가능한 점프(조인트 리셋 등)만 표시한다.
    ambiguous = dt > 0.0 && std::abs(raw_delta) > reach + M_PI;
    return raw_delta;
  }
  if (dt > 0.0 && std::isfinite(velocity)) {
    // 감긴 입력: 속도 힌트(사다리꼴 적분)에 가장 가까운 2π 분기
    const double predicted = std::isfinite(last_velocity_) ?
      0.5 * (last_velocity_ + velocity) * dt : velocity * dt;
    const double k = std::round((predicted - raw_delta) / kTwoPi);
    return raw_delta + kTwoPi * k;
  }
  // 힌트 없음: (−π, π] 로 접는다. 공백이 π/ω_max 를 넘으면 분기가 모호하다.
  ambiguous = dt > 0.0 && reach > M_PI;
  return std::remainder(raw_delta, kTwoPi);
}

EncoderReading WheelEncoderModel::update(double joint_angle, double velocity, double dt)
{
  EncoderReading out;
  if (!initialized_) {
    initialized_ = true;
    last_raw_angle_ = joint_angle;
    last_velocity_ = velocity;
    unwrapped_angle_ = joint_angle;
    ticks_ = quantizeAngle(unwrapped_angle_);
    out.ticks = ticks_;
    return out;
  }

  bool ambiguous = false;
  const double delta = unwrapDelta(joint_angle - last_raw_angle_, velocity, dt, ambiguous);
  last_raw_angle_ = joint_angle;
  last_velocity_ = velocity;
  unwrapped_angle_ += delta;

  const std::int64_t new_ticks = quantizeAngle(unwrapped_angle_);
  out.delta_ticks = new_ticks - ticks_;
  ticks_ = new_ticks;
  out.ticks = ticks_;
  out.true_delta_angle = delta;
  out.ambiguous = ambiguous;

  const double measured = params_.quantize ?
    static_cast<double>(out.delta_ticks) * tick_angle_ : delta;
  double noise = 0.0;
  if (params_.slip_noise && params_.slip_noise_stddev > 0.0) {
    // 거리당 슬립: Var η = σ_s² φ_ref |Δφ| (난수는 매 갱신 하나 — seed 재현성)
    noise = params_.slip_noise_stddev *
      std::sqrt(params_.slip_reference_angle * std::abs(measured)) * noise_(rng_);
  }
  out.delta_angle = measured + noise;
  out.slip_factor = measured != 0.0 ? out.delta_angle / measured : 1.0;
  out.valid = true;
  return out;
}

}  // namespace amr_localization
