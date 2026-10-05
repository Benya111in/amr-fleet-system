#include "amr_rl_controller/policy/onnx_policy.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace amr_rl_controller
{

namespace C = contract;

namespace
{
Ort::Env & ortEnv()
{
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "amr_rl_controller");
  return env;
}

std::string shapeStr(const std::vector<int64_t> & s)
{
  std::ostringstream os;
  os << '[';
  for (std::size_t i = 0; i < s.size(); ++i) {
    os << (i ? ", " : "") << s[i];
  }
  os << ']';
  return os.str();
}

struct IoInfo
{
  std::string name;
  ONNXTensorElementDataType type{ONNX_TENSOR_ELEMENT_DATA_TYPE_UNDEFINED};
  std::vector<int64_t> shape;   // 동적 축 = −1
};

/// 모양 확인: 마지막 축들이 want 와 같고 (want 의 −1 은 아무 값), 축 수가 같다.
bool shapeMatches(const std::vector<int64_t> & got, const std::vector<int64_t> & want)
{
  if (got.size() != want.size()) {
    return false;
  }
  for (std::size_t i = 0; i < got.size(); ++i) {
    if (want[i] >= 0 && got[i] >= 0 && got[i] != want[i]) {
      return false;
    }
    if (want[i] >= 0 && got[i] < 0) {
      return false;   // 고정이어야 하는 축이 동적
    }
  }
  return true;
}
}  // namespace

struct OnnxPolicy::Impl
{
  Ort::SessionOptions options;
  std::unique_ptr<Ort::Session> session;
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  std::vector<IoInfo> inputs;
  std::vector<IoInfo> outputs;
  std::vector<const char *> in_names;
  std::vector<const char *> out_names;
  int idx_obs{-1};
  int idx_mask{-1};
  int idx_h_in{-1};
  int out_action{-1};
  int out_logits{-1};
  int out_h{-1};
};

OnnxPolicy::OnnxPolicy(const OnnxPolicyConfig & cfg)
: impl_(std::make_unique<Impl>()), arm_(cfg.expected_arm), path_(cfg.model_path)
{
  auto fail = [this](const std::string & why) {
      throw std::runtime_error("ONNX 정책 '" + path_ + "': " + why);
    };
  Impl & I = *impl_;
  I.options.SetIntraOpNumThreads(std::max(1, cfg.intra_op_threads));
  I.options.SetInterOpNumThreads(1);
  I.options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
  I.options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  // 공유 서버: 유휴 스레드가 바쁜 대기로 코어를 태우지 않게
  I.options.AddConfigEntry("session.intra_op.allow_spinning", "0");
  try {
    I.session = std::make_unique<Ort::Session>(ortEnv(), cfg.model_path.c_str(), I.options);
  } catch (const Ort::Exception & e) {
    fail(std::string("로드 실패: ") + e.what());
  }

  Ort::AllocatorWithDefaultOptions alloc;
  // 메타데이터
  {
    Ort::ModelMetadata md = I.session->GetModelMetadata();
    auto keys = md.GetCustomMetadataMapKeysAllocated(alloc);
    for (auto & k : keys) {
      const std::string key(k.get());
      auto v = md.LookupCustomMetadataMapAllocated(key.c_str(), alloc);
      meta_[key] = v ? std::string(v.get()) : std::string();
    }
  }
  auto need = [&](const std::string & key) -> std::string {
      const auto it = meta_.find(key);
      if (it == meta_.end()) {
        if (cfg.require_metadata) {
          fail("메타데이터 '" + key + "' 없음 (onnx_io.md 필수)");
        }
        return std::string();
      }
      return it->second;
    };
  const std::string ver = need("contract_version");
  if (!ver.empty()) {
    const int major = std::atoi(ver.substr(0, ver.find('.')).c_str());
    if (major != C::kContractMajor) {
      fail(
        "contract_version " + ver + " 의 주 버전이 " + std::to_string(C::kContractMajor) +
        " 이 아니다");
    }
  }
  const std::string obs_dim = need("obs_dim");
  if (!obs_dim.empty() && obs_dim != std::to_string(C::kObsDim)) {
    fail("obs_dim " + obs_dim + " != " + std::to_string(C::kObsDim));
  }
  const std::string arm = need("arm");
  const std::string want_arm = cfg.expected_arm == Arm::kB ? "B" : "C";
  if (!arm.empty() && arm != want_arm) {
    fail("arm " + arm + " != 파라미터 mode " + want_arm);
  }
  const std::string norm = need("normalization");
  if (!norm.empty() && norm != "inside") {
    fail("normalization '" + norm + "' != inside");
  }
  const std::string dt = need("dt");
  if (!dt.empty() && std::abs(std::atof(dt.c_str()) - C::kDt) > 1e-9) {
    fail("dt " + dt + " != 0.1");
  }

  // 입출력
  for (std::size_t i = 0; i < I.session->GetInputCount(); ++i) {
    IoInfo io;
    io.name = I.session->GetInputNameAllocated(i, alloc).get();
    // TypeInfo 를 붙잡아 둔다: GetTensorTypeAndShapeInfo() 는 소유하지 않는 뷰라 임시 객체에서
    // 꺼내면 해제된 메모리를 읽는다 (모양 차원 수가 쓰레기 → vector 길이 초과로 실측)
    const Ort::TypeInfo type_info = I.session->GetInputTypeInfo(i);
    auto ti = type_info.GetTensorTypeAndShapeInfo();
    io.type = ti.GetElementType();
    io.shape = ti.GetShape();
    I.inputs.push_back(io);
  }
  for (std::size_t i = 0; i < I.session->GetOutputCount(); ++i) {
    IoInfo io;
    io.name = I.session->GetOutputNameAllocated(i, alloc).get();
    const Ort::TypeInfo type_info = I.session->GetOutputTypeInfo(i);
    auto ti = type_info.GetTensorTypeAndShapeInfo();
    io.type = ti.GetElementType();
    io.shape = ti.GetShape();
    I.outputs.push_back(io);
  }
  auto find = [](const std::vector<IoInfo> & v, const std::string & n) {
      for (std::size_t i = 0; i < v.size(); ++i) {
        if (v[i].name == n) {
          return static_cast<int>(i);
        }
      }
      return -1;
    };
  auto check = [&](const std::vector<IoInfo> & v, int idx, const std::string & n,
      ONNXTensorElementDataType t, const std::vector<int64_t> & shape) {
      if (idx < 0) {
        fail("'" + n + "' 가 없다");
      }
      const IoInfo & io = v[static_cast<std::size_t>(idx)];
      if (io.type != t) {
        fail(
          "'" + n + "' dtype " + std::to_string(static_cast<int>(io.type)) + " != " +
          std::to_string(static_cast<int>(t)));
      }
      if (!shapeMatches(io.shape, shape)) {
        fail("'" + n + "' 모양 " + shapeStr(io.shape) + " != " + shapeStr(shape));
      }
    };
  const auto F = ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
  I.idx_obs = find(I.inputs, "obs");
  check(I.inputs, I.idx_obs, "obs", F, {-1, C::kObsDim});
  if (arm_ == Arm::kC) {
    I.idx_mask = find(I.inputs, "mask");
    check(I.inputs, I.idx_mask, "mask", F, {-1, C::kNActionsC});
    I.out_logits = find(I.outputs, "logits");
    check(I.outputs, I.out_logits, "logits", F, {-1, C::kNActionsC});
    I.out_action = find(I.outputs, "action");
    check(I.outputs, I.out_action, "action", ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64, {-1});
  } else {
    if (find(I.inputs, "mask") >= 0) {
      fail("B 모델에 'mask' 입력이 있다 (C 모델인가?)");
    }
    I.out_action = find(I.outputs, "action");
    check(I.outputs, I.out_action, "action", F, {-1, 2});
  }
  // 순환 (선택)
  I.idx_h_in = find(I.inputs, "h_in");
  I.out_h = find(I.outputs, "h_out");
  if ((I.idx_h_in >= 0) != (I.out_h >= 0)) {
    fail("h_in 과 h_out 은 함께 있어야 한다");
  }
  if (I.idx_h_in >= 0) {
    const auto & hs = I.inputs[static_cast<std::size_t>(I.idx_h_in)].shape;
    if (hs.size() != 3) {
      fail("h_in 모양 " + shapeStr(hs) + " 은 [layers, batch, hidden] 이어야 한다");
    }
    int64_t layers = hs[0];
    int64_t hidden = hs[2];
    if (meta_.count("rnn_layers")) {
      layers = std::atoll(meta_["rnn_layers"].c_str());
    }
    if (meta_.count("rnn_hidden")) {
      hidden = std::atoll(meta_["rnn_hidden"].c_str());
    }
    if (layers <= 0 || hidden <= 0) {
      fail("h_in 의 layers·hidden 을 정할 수 없다 (메타데이터 rnn_layers·rnn_hidden 필요)");
    }
    h_shape_ = {layers, 1, hidden};
    h_.assign(static_cast<std::size_t>(layers * hidden), 0.0f);
  }
  // 이름 배열 (입력은 모델 순서 그대로, 출력은 전부)
  for (const auto & io : I.inputs) {
    if (io.name != "obs" && io.name != "mask" && io.name != "h_in") {
      fail("모르는 입력 '" + io.name + "'");
    }
    I.in_names.push_back(io.name.c_str());
  }
  for (const auto & io : I.outputs) {
    I.out_names.push_back(io.name.c_str());
  }
}

OnnxPolicy::~OnnxPolicy() = default;

void OnnxPolicy::resetHidden()
{
  std::fill(h_.begin(), h_.end(), 0.0f);
}

std::string OnnxPolicy::ortVersion()
{
  return Ort::GetVersionString();
}

std::string OnnxPolicy::describe() const
{
  std::ostringstream os;
  os << "arm " << (arm_ == Arm::kB ? 'B' : 'C');
  for (const char * k : {"contract_version", "obs_dim", "algo", "simulator", "seed", "train_steps",
      "git_commit", "fixture"})
  {
    const auto it = meta_.find(k);
    if (it != meta_.end()) {
      os << ", " << k << " " << it->second;
    }
  }
  os << (recurrent() ? ", 순환(h " + shapeStr(h_shape_) + ")" : std::string(", 비순환"));
  os << ", ORT " << ortVersion();
  return os.str();
}

PolicyOutput OnnxPolicy::run(const float * obs, const float * mask)
{
  PolicyOutput out;
  Impl & I = *impl_;
  try {
    std::array<int64_t, 2> obs_shape{1, C::kObsDim};
    std::array<int64_t, 2> mask_shape{1, C::kNActionsC};
    std::vector<Ort::Value> inputs;
    inputs.reserve(I.inputs.size());
    for (const auto & io : I.inputs) {
      if (io.name == "obs") {
        inputs.push_back(
          Ort::Value::CreateTensor<float>(
            I.mem, const_cast<float *>(obs), C::kObsDim, obs_shape.data(), obs_shape.size()));
      } else if (io.name == "mask") {
        inputs.push_back(
          Ort::Value::CreateTensor<float>(
            I.mem, const_cast<float *>(mask), C::kNActionsC, mask_shape.data(),
            mask_shape.size()));
      } else {
        inputs.push_back(
          Ort::Value::CreateTensor<float>(
            I.mem, h_.data(), h_.size(), h_shape_.data(), h_shape_.size()));
      }
    }
    const auto t0 = std::chrono::steady_clock::now();
    auto outs = I.session->Run(
      Ort::RunOptions{nullptr}, I.in_names.data(), inputs.data(), inputs.size(),
      I.out_names.data(), I.out_names.size());
    out.infer_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    auto count = [](const Ort::Value & v) {
        return v.GetTensorTypeAndShapeInfo().GetElementCount();
      };
    if (arm_ == Arm::kB) {
      const Ort::Value & a = outs[static_cast<std::size_t>(I.out_action)];
      if (count(a) != 2) {
        out.error = "action 원소 수 " + std::to_string(count(a)) + " != 2";
        return out;
      }
      const float * p = a.GetTensorData<float>();
      out.b_action = {p[0], p[1]};
    } else {
      const Ort::Value & l = outs[static_cast<std::size_t>(I.out_logits)];
      const Ort::Value & a = outs[static_cast<std::size_t>(I.out_action)];
      if (count(l) != static_cast<std::size_t>(C::kNActionsC) || count(a) != 1) {
        out.error = "logits/action 원소 수 " + std::to_string(count(l)) + "/" +
          std::to_string(count(a));
        return out;
      }
      const float * p = l.GetTensorData<float>();
      out.logits.assign(p, p + C::kNActionsC);
      out.c_action = a.GetTensorData<int64_t>()[0];
    }
    if (I.out_h >= 0) {
      const Ort::Value & h = outs[static_cast<std::size_t>(I.out_h)];
      if (count(h) != h_.size()) {
        out.error = "h_out 원소 수 " + std::to_string(count(h)) + " != " +
          std::to_string(h_.size());
        return out;
      }
      const float * p = h.GetTensorData<float>();
      std::copy(p, p + h_.size(), h_.begin());
    }
    out.ok = true;
  } catch (const Ort::Exception & e) {
    out.ok = false;
    out.error = std::string("ORT Run 실패: ") + e.what();
  } catch (const std::exception & e) {
    out.ok = false;
    out.error = e.what();
  }
  return out;
}

}  // namespace amr_rl_controller
