// TTC 재계획 트리거 구현 (ttc_trigger.hpp 설명 참고).
#include "amr_behavior/ttc/ttc_trigger.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace amr_behavior
{

double minTimeToCollision(const std::vector<TtcSample> & samples, const TtcFilter & filter)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & s : samples) {
    if (filter.only_dynamic && !s.is_dynamic) {
      continue;
    }
    if (!(s.confidence >= filter.min_confidence)) {
      continue;
    }
    if (std::isnan(s.ttc) || s.ttc < 0.0) {
      continue;
    }
    if (s.ttc < best) {
      best = s.ttc;
    }
  }
  return best;
}

void TtcTrigger::update(const std::vector<TtcSample> & samples, double stamp)
{
  samples_ = samples;
  stamp_ = stamp;
}

bool TtcTrigger::evaluate(double now, const Config & config, const TtcFilter & filter)
{
  if (now - stamp_ > config.max_age) {
    last_min_ttc_ = std::numeric_limits<double>::infinity();
    return false;
  }
  last_min_ttc_ = minTimeToCollision(samples_, filter);
  if (!(last_min_ttc_ < config.threshold)) {
    return false;
  }
  if (config.cooldown > 0.0 && now - last_trigger_ < config.cooldown) {
    return false;
  }
  last_trigger_ = now;
  return true;
}

}  // namespace amr_behavior
