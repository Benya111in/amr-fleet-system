// 학습 정책 출력 일치 도구 (ROS 비의존, 보너스 1 A2) — 플러그인과 같은 C++ 경로로 관측·행동을 낸다.
//
//   parity_dump --model M.onnx [--mode B|C] [--threads 1] --out PREFIX
//               [--states S.jsonl]   계약 상태 (golden "state" 와 같은 스키마, 한 줄에 하나)
//                                    → buildObservation (+ C 는 MaskBuilder) → PREFIX_obs.f32
//               [--obs O.f32]        float32 [N, 291] 관측을 그대로 (B 만)
//               [--repeat 3]         같은 프로세스에서 정순·역순·섞음(시드 고정) 차례로 실행해
//                                    비트 비교
//               [--integrate I.f64]  float64 [N, 5] (u0, u1, v_cmd, w_cmd, v_cap) → integrateB
//                                    → PREFIX_integrate.f64 [N, 4] (v', w', a_v, a_w)
// 출력: PREFIX_act.f32 (B: [N, 2] tanh(μ), C: [N, 100] logits), C 면 PREFIX_cact.i64 [N],
//       PREFIX_report.json (모델 메타데이터·n·반복 비트 비교·OnnxPolicy::run 시간 p50/p99/max).
// 행동은 OnnxPolicy::run (플러그인과 같은 세션 설정: intra_op = --threads, inter_op 1, 순차,
// ORT_ENABLE_ALL, 스핀 끔, 배치 1) 이 낸 값이다.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "amr_rl_controller/core/kinematics.hpp"
#include "amr_rl_controller/core/mask.hpp"
#include "amr_rl_controller/core/observation.hpp"
#include "amr_rl_controller/policy/onnx_policy.hpp"
#include "golden_io.hpp"

namespace C = amr_rl_controller::contract;

namespace
{
template<typename T>
std::vector<T> readBin(const std::string & path)
{
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) {
    throw std::runtime_error("열 수 없다: " + path);
  }
  const auto bytes = static_cast<std::size_t>(f.tellg());
  if (bytes % sizeof(T) != 0) {
    throw std::runtime_error("크기가 원소의 배수가 아니다: " + path);
  }
  std::vector<T> v(bytes / sizeof(T));
  f.seekg(0);
  f.read(reinterpret_cast<char *>(v.data()), static_cast<std::streamsize>(bytes));
  return v;
}

template<typename T>
void writeBin(const std::string & path, const std::vector<T> & v)
{
  std::ofstream f(path, std::ios::binary);
  f.write(
    reinterpret_cast<const char *>(v.data()), static_cast<std::streamsize>(v.size() * sizeof(T)));
}

double pct(std::vector<double> v, double p)
{
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const double idx = p / 100.0 * static_cast<double>(v.size() - 1);
  const auto lo = static_cast<std::size_t>(idx);
  const std::size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (idx - static_cast<double>(lo)) * (v[hi] - v[lo]);
}

std::map<std::string, std::string> args(int argc, char ** argv)
{
  std::map<std::string, std::string> a;
  for (int i = 1; i + 1 < argc; i += 2) {
    a[argv[i]] = argv[i + 1];
  }
  return a;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    auto a = args(argc, argv);
    if (!a.count("--model") || !a.count("--out")) {
      std::cerr << "사용: parity_dump --model M.onnx --out PREFIX [--mode B|C] [--threads 1] "
        "[--states S.jsonl | --obs O.f32] [--repeat 3] [--integrate I.f64]\n";
      return 2;
    }
    const std::string out = a["--out"];
    const bool c_mode = a.count("--mode") && a["--mode"] == "C";
    const int threads = a.count("--threads") ? std::stoi(a["--threads"]) : 1;
    const int repeat = a.count("--repeat") ? std::stoi(a["--repeat"]) : 3;
    nlohmann::json rep;

    if (a.count("--integrate")) {
      const auto in = readBin<double>(a["--integrate"]);
      std::vector<double> res;
      res.reserve(in.size() / 5 * 4);
      for (std::size_t i = 0; i + 4 < in.size(); i += 5) {
        const auto c = amr_rl_controller::integrateB(
          in[i], in[i + 1], in[i + 2], in[i + 3],
          in[i + 4]);
        res.insert(res.end(), {c.v, c.w, c.a_v, c.a_w});
      }
      writeBin(out + "_integrate.f64", res);
      rep["integrate_n"] = in.size() / 5;
    }

    std::vector<float> obs;
    std::vector<float> masks;
    std::size_t n = 0;
    if (a.count("--states")) {
      std::ifstream f(a["--states"]);
      std::string line;
      const amr_rl_controller::MaskBuilder mb;
      while (std::getline(f, line)) {
        if (line.empty()) {
          continue;
        }
        const auto s = golden::toState(nlohmann::json::parse(line));
        const auto o = amr_rl_controller::buildObservation(s);
        obs.insert(obs.end(), o.begin(), o.end());
        if (c_mode) {
          const auto m = mb.compute(s);
          for (const bool b : m.mask) {
            masks.push_back(b ? 1.0f : 0.0f);
          }
        }
        ++n;
      }
      writeBin(out + "_obs.f32", obs);
    } else if (a.count("--obs")) {
      if (c_mode) {
        throw std::runtime_error("--obs 는 B 만 (C 는 마스크가 상태에서 나온다)");
      }
      obs = readBin<float>(a["--obs"]);
      n = obs.size() / C::kObsDim;
      if (n * C::kObsDim != obs.size()) {
        throw std::runtime_error("관측 파일이 291 의 배수가 아니다");
      }
    }
    rep["n"] = n;
    if (n == 0) {
      std::ofstream(out + "_report.json") << rep.dump(1) << "\n";
      return 0;
    }

    amr_rl_controller::OnnxPolicyConfig pc;
    pc.model_path = a["--model"];
    pc.expected_arm = c_mode ? amr_rl_controller::Arm::kC : amr_rl_controller::Arm::kB;
    pc.intra_op_threads = threads;
    amr_rl_controller::OnnxPolicy policy(pc);
    rep["policy"] = policy.describe();
    rep["threads"] = threads;
    const std::size_t width = c_mode ? C::kNActionsC : 2;

    // 반복 r: 0 정순, 1 역순, 2.. 섞음 (시드 = r). 모든 반복의 출력을 원래 순서 자리에 둔다.
    std::vector<std::vector<float>> acts(static_cast<std::size_t>(std::max(1, repeat)));
    std::vector<int64_t> cact(n, -1);
    std::vector<double> ms;
    int failures = 0;
    for (std::size_t r = 0; r < acts.size(); ++r) {
      std::vector<std::size_t> order(n);
      std::iota(order.begin(), order.end(), 0);
      if (r == 1) {
        std::reverse(order.begin(), order.end());
      } else if (r >= 2) {
        std::mt19937 g(static_cast<unsigned>(r));
        std::shuffle(order.begin(), order.end(), g);
      }
      acts[r].assign(n * width, 0.0f);
      for (const std::size_t i : order) {
        const float * mk = c_mode ? &masks[i * C::kNActionsC] : nullptr;
        const auto o = policy.run(&obs[i * C::kObsDim], mk);
        if (!o.ok) {
          ++failures;
          continue;
        }
        if (r == 0) {
          ms.push_back(o.infer_ms);
        }
        if (c_mode) {
          std::copy(o.logits.begin(), o.logits.end(), acts[r].begin() + i * width);
          cact[i] = o.c_action;
        } else {
          acts[r][i * 2] = o.b_action[0];
          acts[r][i * 2 + 1] = o.b_action[1];
        }
      }
    }
    writeBin(out + "_act.f32", acts[0]);
    if (c_mode) {
      writeBin(out + "_cact.i64", cact);
    }
    // 반복 사이 비트 비교
    nlohmann::json bits = nlohmann::json::array();
    for (std::size_t r = 1; r < acts.size(); ++r) {
      std::size_t diff = 0;
      double max_abs = 0.0;
      for (std::size_t k = 0; k < acts[0].size(); ++k) {
        uint32_t x = 0;
        uint32_t y = 0;
        std::memcpy(&x, &acts[0][k], 4);
        std::memcpy(&y, &acts[r][k], 4);
        if (x != y) {
          ++diff;
          max_abs = std::max(max_abs, static_cast<double>(std::abs(acts[0][k] - acts[r][k])));
        }
      }
      bits.push_back({{"pass", r}, {"differing_values", diff}, {"max_abs", max_abs}});
    }
    rep["repeat_bit_compare"] = bits;
    rep["failures"] = failures;
    rep["run_ms"] = {{"p50", pct(ms, 50)}, {"p99", pct(ms, 99)}, {"max", pct(ms, 100)}};
    rep["metadata"] = policy.metadata();
    std::ofstream(out + "_report.json") << rep.dump(1) << "\n";
    return failures == 0 ? 0 : 1;
  } catch (const std::exception & e) {
    std::cerr << "parity_dump: " << e.what() << "\n";
    return 1;
  }
}
