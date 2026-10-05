// ONNX Runtime C++ 세션 래퍼 — rl/contract/onnx_io.md 의 입출력 계약을 configure 시점에 검증한다.
//
// 검증 (하나라도 어긋나면 생성자가 std::runtime_error):
//   메타데이터 contract_version 주 버전 = 1, obs_dim = 291, arm = 기대값(B·C),
//   normalization = inside,
//   dt = 0.1; 입력 obs [batch, 291] float (C 는 mask [batch, 100] float 추가);
//   출력 B: action [batch, 2] float, C: logits [batch, 100] float + action [batch] int64.
//   선택: h_in·h_out (순환 정책, [layers, batch, hidden],
//   메타데이터 rnn_layers·rnn_hidden 또는 고정 모양).
// run() 은 예외를 던지지 않는다 — 실패는 PolicyOutput.ok = false 와 이유로 알린다.
#ifndef AMR_RL_CONTROLLER__POLICY__ONNX_POLICY_HPP_
#define AMR_RL_CONTROLLER__POLICY__ONNX_POLICY_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "amr_rl_controller/core/policy.hpp"

namespace amr_rl_controller
{

struct OnnxPolicyConfig
{
  std::string model_path;
  Arm expected_arm{Arm::kC};
  int intra_op_threads{1};
  bool require_metadata{true};    ///< 필수 메타데이터가 없으면 실패 (onnx_io.md 「공통」)
};

class OnnxPolicy : public PolicyInterface
{
public:
  explicit OnnxPolicy(const OnnxPolicyConfig & cfg);
  ~OnnxPolicy() override;

  Arm arm() const override {return arm_;}
  PolicyOutput run(const float * obs, const float * mask) override;
  void resetHidden() override;

  const std::map<std::string, std::string> & metadata() const {return meta_;}
  bool recurrent() const {return !h_.empty();}
  /// 한 줄 요약 (로드 확인 로그용).
  std::string describe() const;
  /// ORT 버전 문자열.
  static std::string ortVersion();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Arm arm_{Arm::kC};
  std::map<std::string, std::string> meta_;
  std::vector<float> h_;
  std::vector<int64_t> h_shape_;
  std::string path_;
};

}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__POLICY__ONNX_POLICY_HPP_
