// 추론 지연 측정 (ROS 비의존).
// golden decision 상태 17 개를 돌려 가며 한 결정 주기의 단계별 시간을 잰다:
//   obs = buildObservation, mask = MaskBuilder::compute (C 만), infer = OnnxPolicy::run (ORT Run),
//   total = 세 단계 합 (플러그인의 결정 틱 비용에서 ROS 메시지 변환을 뺀 것).
// 사용: latency_bench <model.onnx> <B|C> [반복 수 5000] [intra_op 스레드 1] [워밍업 200] [worst]
//   worst: golden 대신 합성 최악 장면 하나 — 720 빔 전부 유효(반경 4 m 원,
//          롤아웃 안에서 충돌 없음 → 모든 자세에서 모든 점 검사), v_cmd 1.0 (T_sim 1.5 s),
//          직전 행동 0 (저크 통과 9 셀),
//          동적 트랙 8 개 (VO 계산), 경로 30 m (600 점 투영).
// 출력: 단계별 p50 / p90 / p99 / max / mean [ms] 와 20 Hz 예산 50 ms 대비 비율.
// 환경 변수 LATENCY_PERIOD_MS (A2): 반복 사이에 이만큼 쉰다 — 플러그인처럼 결정 주기(100 ms)마다
// 한 번 부를 때의 지연 (캐시·클럭이 식은 상태). 0 이면 쉬지 않는 연속 반복 (1차 물결 r3 측정과
// 같음).
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "amr_rl_controller/core/mask.hpp"
#include "amr_rl_controller/core/observation.hpp"
#include "amr_rl_controller/policy/onnx_policy.hpp"
#include "golden_io.hpp"

namespace fs = std::filesystem;
namespace C = amr_rl_controller::contract;

namespace
{
double pct(std::vector<double> v, double p)
{
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const double idx = p / 100.0 * static_cast<double>(v.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(idx);
  const std::size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (idx - static_cast<double>(lo)) * (v[hi] - v[lo]);
}

void row(const char * name, const std::vector<double> & v)
{
  double sum = 0.0;
  for (double x : v) {
    sum += x;
  }
  std::printf(
    "| %-6s | %8.4f | %8.4f | %8.4f | %8.4f | %8.4f | %6.2f %% |\n", name, pct(v, 50), pct(v, 90),
    pct(v, 99), pct(v, 100), v.empty() ? 0.0 : sum / static_cast<double>(v.size()),
    pct(v, 99) / 50.0 * 100.0);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "사용: %s <model.onnx> <B|C> [iters] [threads] [warmup]\n", argv[0]);
    return 2;
  }
  const std::string model = argv[1];
  const bool c_mode = std::string(argv[2]) == "C";
  const int iters = argc > 3 ? std::atoi(argv[3]) : 5000;
  const int threads = argc > 4 ? std::atoi(argv[4]) : 1;
  const int warmup = argc > 5 ? std::atoi(argv[5]) : 200;
  const bool worst = argc > 6 && std::string(argv[6]) == "worst";
  const char * period_env = std::getenv("LATENCY_PERIOD_MS");
  const double period_ms = period_env ? std::atof(period_env) : 0.0;

  std::vector<amr_rl_controller::ContractState> states;
  for (const auto & e : fs::directory_iterator(std::string(RL_CONTRACT_DIR) + "/golden")) {
    const std::string n = e.path().filename().string();
    if (n.rfind("d", 0) == 0 && e.path().extension() == ".json") {
      states.push_back(golden::toState(golden::load(e.path().string())["state"]));
    }
  }
  if (worst) {
    states.clear();
    amr_rl_controller::ContractState s;
    s.pose = {0.0, 0.0, 0.0};
    for (int k = 0; k <= 600; ++k) {
      s.path.push_back({0.05 * k, 0.0});
    }
    s.scan_ranges.assign(C::kLidarSamples, 4.0);
    s.v_meas = s.v_cmd = 1.0;
    s.v_cap = 2.0;   // v' 1.02 까지 속도 조건 통과
    for (int i = 0; i < 8; ++i) {
      const double a = 0.7 * i - 2.0;
      s.tracks.push_back(
        {i, 5.0 * std::cos(a), 5.0 * std::sin(a), -0.8 * std::cos(a),
          -0.8 * std::sin(a), true});
    }
    states.push_back(s);
  }
  if (states.empty()) {
    std::fprintf(stderr, "golden decision 상태가 없다\n");
    return 1;
  }
  amr_rl_controller::OnnxPolicyConfig pc;
  pc.model_path = model;
  pc.expected_arm = c_mode ? amr_rl_controller::Arm::kC : amr_rl_controller::Arm::kB;
  pc.intra_op_threads = threads;
  amr_rl_controller::OnnxPolicy policy(pc);
  const amr_rl_controller::MaskBuilder mb;

  std::vector<double> t_obs, t_mask, t_inf, t_tot;
  t_obs.reserve(static_cast<std::size_t>(iters));
  using clk = std::chrono::steady_clock;
  auto ms = [](clk::time_point a, clk::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
  int failures = 0;
  for (int k = 0; k < warmup + iters; ++k) {
    if (period_ms > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(period_ms));
    }
    const auto & s = states[static_cast<std::size_t>(k) % states.size()];
    const auto t0 = clk::now();
    const auto obs = amr_rl_controller::buildObservation(s);
    const auto t1 = clk::now();
    std::array<float, C::kNActionsC> mask{};
    if (c_mode) {
      const auto m = mb.compute(s);
      for (std::size_t i = 0; i < mask.size(); ++i) {
        mask[i] = m.mask[i] ? 1.0f : 0.0f;
      }
    }
    const auto t2 = clk::now();
    const auto out = policy.run(obs.data(), mask.data());
    const auto t3 = clk::now();
    if (!out.ok) {
      ++failures;
    }
    if (k >= warmup) {
      t_obs.push_back(ms(t0, t1));
      t_mask.push_back(ms(t1, t2));
      t_inf.push_back(ms(t2, t3));
      t_tot.push_back(ms(t0, t3));
    }
  }
  std::printf(
    "model %s | mode %s | iters %d (warmup %d) | intra_op %d | states %zu%s | failures %d | "
    "period %.1f ms\n",
    model.c_str(), c_mode ? "C" : "B", iters, warmup, threads, states.size(),
    worst ? " (합성 최악 장면)" : " (golden d01-d17)", failures, period_ms);
  std::printf("%s\n", policy.describe().c_str());
  std::printf("| 단계   |   p50 ms |   p90 ms |   p99 ms |   max ms |  mean ms | p99/50ms |\n");
  std::printf("|--------|----------|----------|----------|----------|----------|----------|\n");
  row("obs", t_obs);
  if (c_mode) {
    row("mask", t_mask);
  }
  row("infer", t_inf);
  row("total", t_tot);
  return failures == 0 ? 0 : 1;
}
