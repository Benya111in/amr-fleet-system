// 이탈 상태기계 (reward.dev_update, metrics.py:250-263 과 같은 순서).
#ifndef AMR_RL_CONTROLLER__CORE__DEV_STATE_HPP_
#define AMR_RL_CONTROLLER__CORE__DEV_STATE_HPP_

#include "amr_rl_controller/core/contract.hpp"
#include "amr_rl_controller/core/state.hpp"

namespace amr_rl_controller
{

/// 밖이면 e > 0.30 에서 시작. 안이면 e > peak 면 peak 갱신·타이머 0, 아니면 타이머 += dt;
/// 그 뒤 e ≤ 0.15 면 끝 (모두 0).
inline DevState devUpdate(const DevState & dev, double e, double dt = contract::kDt)
{
  if (!dev.in_dev) {
    if (e > contract::kDevOut) {
      return DevState{true, e, 0.0};
    }
    return DevState{};
  }
  DevState nd = e > dev.peak ? DevState{true, e, 0.0} :
  DevState{true, dev.peak, dev.t_since_peak + dt};
  if (e <= contract::kDevBack) {
    return DevState{};
  }
  return nd;
}

}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__CORE__DEV_STATE_HPP_
