// 정책 추론 인터페이스 (ROS·ORT 비의존). OnnxPolicy 가 구현하고, 시험은 가짜 정책을 쓴다.
#ifndef AMR_RL_CONTROLLER__CORE__POLICY_HPP_
#define AMR_RL_CONTROLLER__CORE__POLICY_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "amr_rl_controller/core/contract.hpp"

namespace amr_rl_controller
{

enum class Arm : int {kB = 0, kC = 1};

struct PolicyOutput
{
  bool ok{false};
  std::string error;                 ///< ok 가 거짓일 때 이유
  std::array<float, 2> b_action{};   ///< B: tanh(μ) ∈ [−1, 1]²
  std::vector<float> logits;         ///< C: 마스크된 logits (100)
  int64_t c_action{-1};              ///< C: 모델의 argmax
  double infer_ms{0.0};              ///< Run() 벽시계 시간
};

class PolicyInterface
{
public:
  virtual ~PolicyInterface() = default;
  virtual Arm arm() const = 0;
  /// obs: 291, mask: 100 (B 는 무시). 예외를 던지지 않고 ok=false 로 실패를 알린다.
  virtual PolicyOutput run(const float * obs, const float * mask) = 0;
  /// 순환 정책 은닉 상태를 0 으로 (없으면 아무것도 안 함).
  virtual void resetHidden() {}
};

}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__CORE__POLICY_HPP_
