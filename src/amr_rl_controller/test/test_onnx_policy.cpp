// OnnxPolicy (ORT C++ 세션 래퍼) 시험.
//  - 계약 고정물(fixture_b/c.onnx)을 C++ 관측·마스크로 끝-끝 실행해 r0 의 기대 출력
//    (rl/contract/onnx/fixtures/fixture_expected.json, 파이썬 onnxruntime) 과 1e-5 안에서 일치
//  - configure 시점 검증: 주 버전·obs_dim·arm·메타데이터 누락·없는 파일 → 예외
//  - 순환(h_in/h_out) 상태 전달과 resetHidden, NaN 출력 모델 → 엔진 DWA 폴백
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "amr_rl_controller/core/decision_engine.hpp"
#include "amr_rl_controller/core/mask.hpp"
#include "amr_rl_controller/core/observation.hpp"
#include "amr_rl_controller/policy/onnx_policy.hpp"
#include "golden_io.hpp"

namespace fs = std::filesystem;
namespace C = amr_rl_controller::contract;
using amr_rl_controller::Arm;
using amr_rl_controller::OnnxPolicy;
using amr_rl_controller::OnnxPolicyConfig;

namespace
{
bool g_models_ok = false;

class ModelEnv : public ::testing::Environment
{
public:
  void SetUp() override
  {
    const std::string cmd = std::string("RL_CONTRACT_DIR=") + RL_CONTRACT_DIR + " python3 " +
      MAKE_MODELS + " " + FIXTURE_DIR + " > " + FIXTURE_DIR + ".log 2>&1";
    fs::create_directories(fs::path(FIXTURE_DIR).parent_path());
    g_models_ok = std::system(cmd.c_str()) == 0 &&
      fs::exists(std::string(FIXTURE_DIR) + "/fixture_c.onnx");
  }
};
const auto * const g_env = ::testing::AddGlobalTestEnvironment(new ModelEnv);

std::string model(const std::string & n) {return std::string(FIXTURE_DIR) + "/" + n;}

#define REQUIRE_MODELS() \
  if (!g_models_ok) {GTEST_SKIP() << "torch 로 시험 모델을 만들지 못했다 (" FIXTURE_DIR ".log)";}

OnnxPolicyConfig cfg(const std::string & n, Arm arm)
{
  OnnxPolicyConfig c;
  c.model_path = model(n);
  c.expected_arm = arm;
  return c;
}

std::vector<std::string> decisionGoldens()
{
  std::vector<std::string> out;
  for (const auto & e : fs::directory_iterator(std::string(RL_CONTRACT_DIR) + "/golden")) {
    const std::string n = e.path().filename().string();
    if (n.rfind("d", 0) == 0 && e.path().extension() == ".json") {
      out.push_back(e.path().string());
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}
}  // namespace

TEST(OnnxPolicy, FixturesMatchPythonOrtEndToEnd)
{
  REQUIRE_MODELS();
  OnnxPolicy pb(cfg("fixture_b.onnx", Arm::kB));
  OnnxPolicy pc(cfg("fixture_c.onnx", Arm::kC));
  EXPECT_EQ(pb.metadata().at("contract_version"), C::kContractVersion);
  const auto exp = golden::load(
    std::string(
      RL_CONTRACT_DIR) + "/onnx/fixtures/fixture_expected.json");
  const double tol = exp["tolerance"].get<double>();
  const auto files = decisionGoldens();
  ASSERT_EQ(files.size(), exp["rows"].size());
  const amr_rl_controller::MaskBuilder mb;
  for (std::size_t r = 0; r < files.size(); ++r) {
    const auto j = golden::load(files[r]);
    const auto & row = exp["rows"][r];
    ASSERT_EQ(j["name"], row["name"]);
    const auto s = golden::toState(j["state"]);
    const auto obs = amr_rl_controller::buildObservation(s);     // C++ 관측 (golden obs 아님)
    const auto m = mb.compute(s);
    std::array<float, C::kNActionsC> mask{};
    for (std::size_t i = 0; i < mask.size(); ++i) {
      mask[i] = m.mask[i] ? 1.0f : 0.0f;
    }
    const auto ob = pb.run(obs.data(), nullptr);
    ASSERT_TRUE(ob.ok) << ob.error;
    EXPECT_NEAR(ob.b_action[0], row["b_action"][0].get<double>(), tol) << row["name"];
    EXPECT_NEAR(ob.b_action[1], row["b_action"][1].get<double>(), tol) << row["name"];
    const auto oc = pc.run(obs.data(), mask.data());
    ASSERT_TRUE(oc.ok) << oc.error;
    EXPECT_EQ(oc.c_action, row["c_action"].get<int64_t>()) << row["name"];
    for (int i = 0; i < C::kNActionsC; ++i) {
      const auto & e = row["c_logits_valid"][i];
      if (e.is_null()) {
        EXPECT_LT(oc.logits[static_cast<std::size_t>(i)], -1e8) << row["name"] << " " << i;
      } else {
        EXPECT_NEAR(oc.logits[static_cast<std::size_t>(i)], e.get<double>(), tol)
          << row["name"] << " logit " << i;
      }
    }
    EXPECT_GE(ob.infer_ms, 0.0);
  }
}

TEST(OnnxPolicy, RejectsContractViolationsAtLoad)
{
  REQUIRE_MODELS();
  EXPECT_THROW(OnnxPolicy(cfg("bad_major_c.onnx", Arm::kC)), std::runtime_error);
  EXPECT_THROW(OnnxPolicy(cfg("bad_obsdim_c.onnx", Arm::kC)), std::runtime_error);
  EXPECT_THROW(OnnxPolicy(cfg("no_meta_c.onnx", Arm::kC)), std::runtime_error);
  EXPECT_THROW(OnnxPolicy(cfg("fixture_b.onnx", Arm::kC)), std::runtime_error);   // arm 불일치
  EXPECT_THROW(OnnxPolicy(cfg("fixture_c.onnx", Arm::kB)), std::runtime_error);
  EXPECT_THROW(OnnxPolicy(cfg("does_not_exist.onnx", Arm::kC)), std::runtime_error);
  // 메타데이터 검사를 끄면 입출력 모양만 본다
  auto c = cfg("no_meta_c.onnx", Arm::kC);
  c.require_metadata = false;
  EXPECT_NO_THROW(OnnxPolicy p(c));
}

TEST(OnnxPolicy, FallbackOnlyMaskSelectsFallback)
{
  REQUIRE_MODELS();
  OnnxPolicy pc(cfg("fixture_c.onnx", Arm::kC));
  std::array<float, C::kObsDim> obs{};
  std::array<float, C::kNActionsC> mask{};
  mask[C::kFallbackIndex] = 1.0f;
  for (int k = 0; k < 5; ++k) {
    obs[static_cast<std::size_t>(k * 50)] = static_cast<float>(k) - 2.0f;
    const auto o = pc.run(obs.data(), mask.data());
    ASSERT_TRUE(o.ok);
    EXPECT_EQ(o.c_action, C::kFallbackIndex);
  }
}

TEST(OnnxPolicy, RecurrentStateCarriesAndResets)
{
  REQUIRE_MODELS();
  OnnxPolicy p(cfg("fixture_c_gru.onnx", Arm::kC));
  ASSERT_TRUE(p.recurrent());
  std::array<float, C::kObsDim> obs{};
  obs.fill(0.3f);
  std::array<float, C::kNActionsC> mask{};
  mask.fill(1.0f);
  const auto a = p.run(obs.data(), mask.data());
  const auto b = p.run(obs.data(), mask.data());   // 같은 입력, 다른 은닉 상태
  ASSERT_TRUE(a.ok && b.ok);
  double diff = 0.0;
  for (std::size_t i = 0; i < a.logits.size(); ++i) {
    diff = std::max(diff, static_cast<double>(std::abs(a.logits[i] - b.logits[i])));
  }
  EXPECT_GT(diff, 1e-6);
  p.resetHidden();
  const auto c = p.run(obs.data(), mask.data());
  for (std::size_t i = 0; i < a.logits.size(); ++i) {
    EXPECT_FLOAT_EQ(a.logits[i], c.logits[i]);
  }
}

TEST(OnnxPolicy, NanLogitsFallBackToDwaInEngine)
{
  REQUIRE_MODELS();
  OnnxPolicy p(cfg("nan_c.onnx", Arm::kC));
  amr_rl_controller::DecisionEngine eng({}, &p);
  amr_rl_controller::TickInput in;
  in.now = 1.0;
  in.state.path = {{0.0, 0.0}, {5.0, 0.0}};
  in.state.scan_ranges.assign(C::kLidarSamples, std::numeric_limits<double>::quiet_NaN());
  in.dist_to_goal = 5.0;
  const auto r = eng.tick(in);
  EXPECT_TRUE(r.need_dwa);
  EXPECT_EQ(r.dwa_reason, amr_rl_controller::DwaReason::kNonFinite);
}
