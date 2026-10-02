// 1차 저역 통과 필터 구현.

#include "amr_localization/low_pass_filter.hpp"

#include <cmath>

namespace amr_localization
{

FirstOrderLowPass::FirstOrderLowPass(double cutoff_hz)
: cutoff_hz_(cutoff_hz)
{
}

double FirstOrderLowPass::alphaFor(double cutoff_hz, double dt)
{
  if (!(cutoff_hz > 0.0) || !(dt > 0.0) || !std::isfinite(dt)) {
    return 1.0;
  }
  const double omega_c = 2.0 * M_PI * cutoff_hz * dt;
  if (omega_c >= M_PI) {
    return 1.0;  // 차단 주파수가 Nyquist 이상 → 필터 의미 없음
  }
  const double c = std::cos(omega_c);
  const double a = 2.0 - c;
  const double beta = a - std::sqrt(a * a - 1.0);
  return 1.0 - beta;
}

double FirstOrderLowPass::magnitudeResponse(double alpha, double omega)
{
  const double beta = 1.0 - alpha;
  return alpha / std::sqrt(1.0 - 2.0 * beta * std::cos(omega) + beta * beta);
}

double FirstOrderLowPass::filter(double x, double dt)
{
  if (!initialized_) {
    initialized_ = true;
    y_ = x;
    return y_;
  }
  const double alpha = alphaFor(cutoff_hz_, dt);
  y_ = alpha * x + (1.0 - alpha) * y_;
  return y_;
}

}  // namespace amr_localization
