// 관측 벡터 (contract.md §4, observation.build_observation 의 C++ 사본). 길이 291, float32.
//
// 정규화(상수로 나누기·자르기)는 이 함수가 하고, 학습 running mean/std 는 ONNX 모델 안에 있다.
#ifndef AMR_RL_CONTROLLER__CORE__OBSERVATION_HPP_
#define AMR_RL_CONTROLLER__CORE__OBSERVATION_HPP_

#include <array>
#include <vector>

#include "amr_rl_controller/core/contract.hpp"
#include "amr_rl_controller/core/state.hpp"

namespace amr_rl_controller
{

using Observation = std::array<float, contract::kObsDim>;

/// 720 빔 → 180 섹터 풋프린트 여유거리 최솟값 / 10 (상한 1).
std::array<double, contract::kNSectors> lidarSectors(const std::vector<double> & ranges);

/// ContractState → 관측 291. 빔 수가 720 이 아니거나 경로가 비면 std::invalid_argument.
Observation buildObservation(const ContractState & s);

}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__CORE__OBSERVATION_HPP_
