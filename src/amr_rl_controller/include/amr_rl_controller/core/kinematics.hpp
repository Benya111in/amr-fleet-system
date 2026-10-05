// 행동 → 속도 명령 적분 (contract.md §5, kinematics.py 의 C++ 사본).
#ifndef AMR_RL_CONTROLLER__CORE__KINEMATICS_HPP_
#define AMR_RL_CONTROLLER__CORE__KINEMATICS_HPP_

#include <array>

#include "amr_rl_controller/core/contract.hpp"
#include "amr_rl_controller/core/state.hpp"

namespace amr_rl_controller
{

/// 적분 결과: 새 명령 (v, ω) 와 실행 명령 가속도 (a_v, a_ω).
struct Command
{
  double v{0.0};
  double w{0.0};
  double a_v{0.0};
  double a_w{0.0};
};

/// 비교군 B: u ∈ [−1, 1]² (밖이면 자른다) → ray 클램프.
Command integrateB(double u0, double u1, double v_cmd, double w_cmd, double v_cap);
/// 격자 셀 인덱스 (0..98) → (a_v, a_ω). 범위 밖이면 std::out_of_range.
Accel cellAccel(int index);
/// 폴백 = DWA 제동 후보와 같은 규칙 (dwa.cpp:163-170 을 0.1 s 로).
Command fallbackCommand(double v_cmd, double w_cmd);
/// 비교군 C: 셀 또는 폴백(99). 클램프하지 않는다 (마스크가 한계를 보장).
Command integrateC(int index, double v_cmd, double w_cmd);
/// 정규화 가속도 거리 (Δa_v/1.0)² + (Δa_ω/2.0)².
double accelDistance(const Accel & a, const Accel & b);
/// 격자 최근접 셀 (마스크 없음, 폴백 제외). 동률이면 작은 인덱스.
int nearestCell(double a_v, double a_w);
/// 마스크 안 최근접 행동 (유효 셀 + mask[99] 이면 가속도 fallback_accel 의 폴백).
int nearestCellMasked(
  double a_v, double a_w, const std::array<bool, contract::kNActionsC> & mask,
  const Accel & fallback_accel);
/// BC 레이블 (v1.1.0): DWA 명령을 마스크 안 최근접 행동으로.
int bcLabel(
  double v_dwa, double w_dwa, double v_cmd, double w_cmd,
  const std::array<bool, contract::kNActionsC> & mask);

}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__CORE__KINEMATICS_HPP_
