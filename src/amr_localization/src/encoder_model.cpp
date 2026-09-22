// 바퀴 인코더 에뮬레이터 구현.

#include "amr_localization/encoder_model.hpp"

#include <cmath>
#include <stdexcept>

namespace amr_localization
{

WheelEncoderModel::WheelEncoderModel(const EncoderParams & params, std::uint64_t seed)
: params_(params),
  tick_angle_(0.0),
  rng_(seed),
  noise_(0.0, params.slip_noise_stddev > 0.0 ? params.slip_noise_stddev : 1.0)
{
  if (params_.ticks_per_revolution <= 0) {
    throw std::invalid_argument("ticks_per_revolution must be positive");
  }
  if (params_.slip_noise_stddev < 0.0) {
    throw std::invalid_argument("slip_noise_stddev must be non-negative");
  }
  tick_angle_ = 2.0 * M_PI / static_cast<double>(params_.ticks_per_revolution);
}

std::int64_t WheelEncoderModel::quantizeAngle(double angle) const
{
  return static_cast<std::int64_t>(std::llround(angle / tick_angle_));
}

void WheelEncoderModel::reset()
{
  initialized_ = false;
}

EncoderReading WheelEncoderModel::update(double joint_angle)
{
  EncoderReading out;
  if (!initialized_) {
    initialized_ = true;
    last_raw_angle_ = joint_angle;
    unwrapped_angle_ = joint_angle;
    ticks_ = quantizeAngle(unwrapped_angle_);
    out.ticks = ticks_;
    return out;
  }

  // 감긴 각도 대비: 주기 차를 (−π, π] 로 접는다
  // (50 Hz 에서 최대 바퀴 속도 24 rad/s 라도 주기당 0.48 rad 라 모호하지 않다).
  double raw_delta = joint_angle - last_raw_angle_;
  raw_delta = std::remainder(raw_delta, 2.0 * M_PI);
  last_raw_angle_ = joint_angle;
  unwrapped_angle_ += raw_delta;

  const std::int64_t new_ticks = quantizeAngle(unwrapped_angle_);
  out.delta_ticks = new_ticks - ticks_;
  ticks_ = new_ticks;
  out.ticks = ticks_;
  out.true_delta_angle = raw_delta;

  const double measured = params_.quantize ?
    static_cast<double>(out.delta_ticks) * tick_angle_ : raw_delta;
  double eps = 0.0;
  if (params_.slip_noise && params_.slip_noise_stddev > 0.0) {
    eps = noise_(rng_);
  }
  out.slip_factor = 1.0 + eps;
  out.delta_angle = measured * out.slip_factor;
  out.valid = true;
  return out;
}

}  // namespace amr_localization
