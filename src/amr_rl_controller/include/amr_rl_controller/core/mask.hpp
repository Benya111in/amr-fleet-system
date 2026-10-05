// 비교군 C 후보 마스크 (contract.md §5.2, mask.compute_mask 의 C++ 사본).
//
// 셀 (a_v, a_ω) 는 직전 명령 기준 상대 가속도 격자다. 판정 규칙은 amr_navigation_core 의
// DWA 원본 함수를 그대로 쓴다: T_sim = DwaPlanner::simTime, 원호 롤아웃 = DwaPlanner::rollout
// (n = ⌈T/0.1⌉ 자세 전부), VO = chordVelocity + velocityObstacleTime. 장애물 입력은 코스트맵이
// 아니라 이번 스캔의 유효 끝점이다 (계약 §2 마지막 단락).
#ifndef AMR_RL_CONTROLLER__CORE__MASK_HPP_
#define AMR_RL_CONTROLLER__CORE__MASK_HPP_

#include <array>
#include <vector>

#include "amr_navigation/core/dwa.hpp"
#include "amr_rl_controller/core/contract.hpp"
#include "amr_rl_controller/core/kinematics.hpp"
#include "amr_rl_controller/core/state.hpp"

namespace amr_rl_controller
{

struct MaskResult
{
  std::array<bool, contract::kNActionsC> mask{};   ///< 99 셀 + 폴백(항상 참)
  std::array<bool, contract::kNCells> speed_ok{};
  std::array<bool, contract::kNCells> jerk_ok{};
  std::array<bool, contract::kNCells> fp_ok{};
  std::array<double, contract::kNCells> vo_time{};   ///< 계산 안 함·VO 없음 = +inf
  std::array<double, contract::kNCells> v_next{};
  std::array<double, contract::kNCells> w_next{};
  bool saturated{false};
  Command fallback;
  int n_valid{0};                                   ///< 유효 셀 수 (폴백 제외)
};

class MaskBuilder
{
public:
  MaskBuilder();
  /// d_goal = max(남은 호길이, 목표 직선거리) (dwa.cpp:449-459).
  static double dGoalForRollout(const ContractState & s);
  /// (v, ω) 원호가 스캔 점과 겹치지 않는가 (목표 너머 d_goal + 0.1 m 는 검사 안 함).
  bool footprintFree(
    const std::vector<Point2D> & pts, double v, double w, double d_goal) const;
  MaskResult compute(const ContractState & s) const;
  const amr_navigation::core::DwaPlanner & dwa() const {return dwa_;}

private:
  amr_navigation::core::DwaPlanner dwa_;
};

}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__CORE__MASK_HPP_
