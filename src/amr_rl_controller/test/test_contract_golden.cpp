// C++ 계약 구현을 r0 의 golden 벡터(rl/contract/golden/*.json, 계약 v1.2.0, 32 사례)와 대조한다.
//   decision d01–d17: 관측 291 (1e-5), 마스크 100·셀별 speed/jerk/fp 판정 (정확 일치),
//                     VO 진입 시각 (1e-4, null = +inf), 포화 여부, 폴백 명령 (1e-9)
//   integrate i01:    B 적분·C 적분·격자 최근접·BC 레이블 (1e-9 / 정확 일치), 격자 값
//   step s01–s12:     이탈 상태기계 (보상 항은 학습 환경 몫 — 플러그인은 보상을 계산하지 않는다)
//   profiler p01:     Gazebo 하류 사슬의 velocity_profiler 코어(amr_navigation)가 학습 환경의
//                     기준 모델과 같은지 (1e-6) — 플러그인 코드는 아니지만 전이 점검 (§5.4)
//   safety f01:       safety_lite = 학습 환경의 근사 φ. 플러그인·Gazebo 는 정본 safety_node 라
//                     대상이 아니다 (contract.md §7.4)
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "amr_navigation/core/velocity_profiler.hpp"
#include "amr_rl_controller/core/contract.hpp"
#include "amr_rl_controller/core/dev_state.hpp"
#include "amr_rl_controller/core/geometry.hpp"
#include "amr_rl_controller/core/kinematics.hpp"
#include "amr_rl_controller/core/mask.hpp"
#include "amr_rl_controller/core/observation.hpp"
#include "golden_io.hpp"

namespace fs = std::filesystem;
namespace C = amr_rl_controller::contract;
using amr_rl_controller::buildObservation;
using amr_rl_controller::MaskBuilder;

namespace
{
std::vector<std::string> goldenFiles(const std::string & prefix)
{
  std::vector<std::string> out;
  for (const auto & e : fs::directory_iterator(std::string(RL_CONTRACT_DIR) + "/golden")) {
    const std::string n = e.path().filename().string();
    if (n.rfind(prefix, 0) == 0 && e.path().extension() == ".json") {
      out.push_back(e.path().string());
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}
}  // namespace

TEST(ContractGolden, DecisionCasesPresent)
{
  // v1.1.0 이후 decision 17 사례 (d01–d17) — v1.2.0 은 보상만 바꿔 값이 같다
  EXPECT_EQ(goldenFiles("d").size(), 17U);
}

TEST(ContractGolden, AllCasesAccountedForAtThisVersion)
{
  // v1.2.0: 32 사례 = decision 17 + step 12 + integrate 1 + profiler 1 + safety 1 (§12).
  // 모든 파일의 contract_version 이 이 플러그인의 판과 같아야 한다 — 계약이 바뀌면 먼저 깨진다.
  // 새 종류의 사례가 생기면 (아래 kinds 밖) 플러그인이 다뤄야 하는지 정하고 이 시험을 고친다.
  const auto all = goldenFiles("");
  EXPECT_EQ(all.size(), 32U);
  std::map<std::string, int> kinds;
  for (const auto & f : all) {
    const auto j = golden::load(f);
    EXPECT_EQ(j["contract_version"].get<std::string>(), std::string(C::kContractVersion)) << f;
    ++kinds[j["kind"].get<std::string>()];
  }
  const std::map<std::string, int> want{
    {"decision", 17}, {"step", 12}, {"integrate", 1}, {"profiler", 1}, {"safety", 1}};
  EXPECT_EQ(kinds, want);
}

TEST(ContractGolden, ObservationMatches)
{
  for (const auto & f : goldenFiles("d")) {
    const auto j = golden::load(f);
    ASSERT_EQ(j["kind"], "decision") << f;
    EXPECT_EQ(j["contract_version"].get<std::string>(), std::string(C::kContractVersion)) << f;
    const double tol = j["tolerance"]["obs"].get<double>();
    const auto s = golden::toState(j["state"]);
    const auto obs = buildObservation(s);
    const auto & exp = j["expected"]["obs"];
    ASSERT_EQ(exp.size(), static_cast<std::size_t>(C::kObsDim)) << f;
    int bad = 0;
    for (std::size_t i = 0; i < obs.size(); ++i) {
      const double e = exp[i].get<double>();
      if (std::abs(static_cast<double>(obs[i]) - e) > tol) {
        if (++bad <= 5) {
          ADD_FAILURE() << f << " obs[" << i << "] = " << obs[i] << " != " << e;
        }
      }
    }
    EXPECT_EQ(bad, 0) << f;
  }
}

TEST(ContractGolden, MaskMatches)
{
  const MaskBuilder mb;
  for (const auto & f : goldenFiles("d")) {
    const auto j = golden::load(f);
    const auto s = golden::toState(j["state"]);
    const auto m = mb.compute(s);
    const auto & e = j["expected"];
    const double vo_tol = j["tolerance"]["vo_time"].get<double>();
    const double fb_tol = j["tolerance"]["fallback"].get<double>();
    for (int i = 0; i < C::kNActionsC; ++i) {
      EXPECT_EQ(m.mask[static_cast<std::size_t>(i)], e["mask"][i].get<bool>())
        << f << " mask[" << i << "]";
    }
    for (int i = 0; i < C::kNCells; ++i) {
      const std::size_t k = static_cast<std::size_t>(i);
      EXPECT_EQ(m.speed_ok[k], e["speed_ok"][i].get<bool>()) << f << " speed_ok[" << i << "]";
      EXPECT_EQ(m.jerk_ok[k], e["jerk_ok"][i].get<bool>()) << f << " jerk_ok[" << i << "]";
      EXPECT_EQ(m.fp_ok[k], e["fp_ok"][i].get<bool>()) << f << " fp_ok[" << i << "]";
      const double ev = golden::num(e["vo_time"][i]);
      if (std::isinf(ev)) {
        EXPECT_TRUE(std::isinf(m.vo_time[k])) << f << " vo_time[" << i << "] = " << m.vo_time[k];
      } else {
        EXPECT_NEAR(m.vo_time[k], ev, vo_tol) << f << " vo_time[" << i << "]";
      }
    }
    EXPECT_EQ(m.saturated, e["saturated"].get<bool>()) << f;
    // golden n_valid = mask.sum() (폴백 포함), MaskResult::n_valid 는 폴백 제외
    EXPECT_EQ(m.n_valid + 1, e["n_valid"].get<int>()) << f;
    const auto & fb = e["fallback"];
    EXPECT_NEAR(m.fallback.v, fb[0].get<double>(), fb_tol) << f;
    EXPECT_NEAR(m.fallback.w, fb[1].get<double>(), fb_tol) << f;
    EXPECT_NEAR(m.fallback.a_v, fb[2].get<double>(), fb_tol) << f;
    EXPECT_NEAR(m.fallback.a_w, fb[3].get<double>(), fb_tol) << f;
  }
}

TEST(ContractGolden, IntegrateMatches)
{
  const auto files = goldenFiles("i01");
  ASSERT_EQ(files.size(), 1U);
  const auto j = golden::load(files[0]);
  const auto & e = j["expected"];
  const double tol = j["tolerance"]["integrate"].get<double>();
  for (int i = 0; i < C::kNAv; ++i) {
    EXPECT_NEAR(C::latticeAv(i), e["lattice_av"][i].get<double>(), 1e-12);
  }
  for (int i = 0; i < C::kNAw; ++i) {
    EXPECT_NEAR(C::latticeAw(i), e["lattice_aw"][i].get<double>(), 1e-12);
  }
  for (const auto & r : e["b"]) {
    const auto c = amr_rl_controller::integrateB(
      r["u"][0].get<double>(), r["u"][1].get<double>(), r["v_cmd"].get<double>(),
      r["w_cmd"].get<double>(), r["v_cap"].get<double>());
    EXPECT_NEAR(c.v, r["out"][0].get<double>(), tol) << r.dump();
    EXPECT_NEAR(c.w, r["out"][1].get<double>(), tol) << r.dump();
    EXPECT_NEAR(c.a_v, r["out"][2].get<double>(), tol) << r.dump();
    EXPECT_NEAR(c.a_w, r["out"][3].get<double>(), tol) << r.dump();
  }
  for (const auto & r : e["c"]) {
    const auto c = amr_rl_controller::integrateC(
      r["index"].get<int>(), r["v_cmd"].get<double>(), r["w_cmd"].get<double>());
    EXPECT_NEAR(c.v, r["out"][0].get<double>(), tol) << r.dump();
    EXPECT_NEAR(c.w, r["out"][1].get<double>(), tol) << r.dump();
    EXPECT_NEAR(c.a_v, r["out"][2].get<double>(), tol) << r.dump();
    EXPECT_NEAR(c.a_w, r["out"][3].get<double>(), tol) << r.dump();
  }
  for (const auto & r : e["nearest_cell"]) {
    EXPECT_EQ(
      amr_rl_controller::nearestCell(r["a"][0].get<double>(), r["a"][1].get<double>()),
      r["cell"].get<int>()) << r.dump();
  }
  ASSERT_GE(e["bc_label"].size(), 7U);
  for (const auto & r : e["bc_label"]) {
    std::array<bool, C::kNActionsC> mask{};
    for (int i = 0; i < C::kNActionsC; ++i) {
      mask[static_cast<std::size_t>(i)] = r["mask"][i].get<bool>();
    }
    const int lab = amr_rl_controller::bcLabel(
      r["dwa"][0].get<double>(), r["dwa"][1].get<double>(), r["v_cmd"].get<double>(),
      r["w_cmd"].get<double>(), mask);
    const auto & o = r["out"];
    EXPECT_EQ(lab, o["label"].get<int>()) << r["scene"].get<std::string>();
    EXPECT_TRUE(mask[static_cast<std::size_t>(lab)]);
    EXPECT_EQ(
      amr_rl_controller::nearestCell(o["a"][0].get<double>(), o["a"][1].get<double>()),
      o["nearest_unmasked"].get<int>()) << r["scene"].get<std::string>();
  }
}

TEST(ContractGolden, DevMachineMatchesStepCases)
{
  // step 사례 s01–s12: pre.dev 와 post 자세의 경로 거리 e → expected.dev (reward.dev_update).
  // v1.2.0 은 s01–s11 의 보상 항만 바꿨고 (dev·e 그대로), s12 (목표 지나침) 를 더했다.
  int n = 0;
  for (const auto & f : goldenFiles("s")) {
    const auto j = golden::load(f);
    ASSERT_EQ(j["kind"], "step") << f;
    const auto pre = golden::toState(j["pre"]);
    const auto post = golden::toState(j["post"]);
    const double e = amr_rl_controller::ContractPath(post.path).project(
      {post.pose.x, post.pose.y}).dist;
    EXPECT_NEAR(e, j["expected"]["e"].get<double>(), j["tolerance"]["e"].get<double>()) << f;
    const auto d = amr_rl_controller::devUpdate(pre.dev, e);
    const auto & ed = j["expected"]["dev"];
    EXPECT_EQ(d.in_dev, ed["in_dev"].get<bool>()) << f;
    EXPECT_NEAR(d.peak, ed["peak"].get<double>(), 1e-9) << f;
    EXPECT_NEAR(d.t_since_peak, ed["t_since_peak"].get<double>(), 1e-9) << f;
    ++n;
  }
  EXPECT_EQ(n, 12);
}

TEST(ContractGolden, ProfilerCaseMatchesNavigationVelocityProfilerCore)
{
  // p01: 정책 명령 램프 → 50 Hz 기준 (v_ref, ω_ref, a_ref). 학습 환경(2D·Isaac)은 기준 구현
  // profiler.ProfilerReference 를, Gazebo 는 amr_navigation 의 velocity_profiler_node 를 쓴다 —
  // 그 코어 (PI 끔 = 기준 경로만) 가 golden 과 같아야 학습과 배포의 하류 사슬이 같다.
  const auto files = goldenFiles("p01");
  ASSERT_EQ(files.size(), 1U);
  const auto j = golden::load(files[0]);
  ASSERT_EQ(j["kind"], "profiler");
  const double tol = j["tolerance"]["profiler"].get<double>();
  const auto & e = j["expected"];
  const double dt = e["dt"].get<double>();
  amr_navigation::core::VelocityProfilerConfig pc;
  pc.linear = {-0.5, C::kVMaxHw, C::kAMax, C::kJerkMax};
  pc.angular = {-C::kWMax, C::kWMax, C::kAlphaMax, C::kAngJerkMax};
  pc.use_pid = false;
  amr_navigation::core::VelocityProfiler prof(pc);
  int n = 0;
  for (const auto & st : e["steps"]) {
    const double v0 = st["cmd_from"][0].get<double>();
    const double w0 = st["cmd_from"][1].get<double>();
    const double v1 = st["cmd_to"][0].get<double>();
    const double w1 = st["cmd_to"][1].get<double>();
    const auto & ref = st["ref"];
    const int k_n = static_cast<int>(ref.size());
    amr_navigation::core::ProfilerOutput out;
    for (int k = 1; k <= k_n; ++k) {
      const double f = static_cast<double>(k) / k_n;
      out = prof.update(v0 + f * (v1 - v0), w0 + f * (w1 - w0), false, 0.0, 0.0, dt);
      const auto & r = ref[static_cast<std::size_t>(k - 1)];
      EXPECT_NEAR(out.v_ref, r[0].get<double>(), tol) << n << " " << k;
      EXPECT_NEAR(out.w_ref, r[1].get<double>(), tol) << n << " " << k;
    }
    EXPECT_NEAR(out.a_ref, st["a_ref"].get<double>(), tol) << n;
    ++n;
  }
  EXPECT_EQ(n, 28);
}

TEST(ContractGeometry, WrapAngleMatchesPythonModulo)
{
  // (a + π) % 2π − π: −π → −π, π → −π
  EXPECT_NEAR(amr_rl_controller::wrapAngle(-M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(amr_rl_controller::wrapAngle(M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(amr_rl_controller::wrapAngle(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(amr_rl_controller::wrapAngle(-7.0), -7.0 + 2.0 * M_PI, 1e-12);
}

TEST(ContractGeometry, ScanWithWrongBeamCountThrows)
{
  amr_rl_controller::ContractState s;
  s.path = {{0.0, 0.0}, {1.0, 0.0}};
  s.scan_ranges.assign(719, 5.0);
  EXPECT_THROW(buildObservation(s), std::invalid_argument);
  EXPECT_THROW(MaskBuilder().compute(s), std::invalid_argument);
}

// golden 이 가르지 못하는 규칙 두 개 (변이 검사에서 놓친 것, 기대값은 파이썬 기준 구현 결과).
TEST(ContractRules, GoalOvershootCutExcludesPosesBeyondGoal)
{
  // 전방 벽 x = 1.2 m (빔 360 = 1.05 m), v_cmd 0.8: 롤아웃 k ≥ 12 (호길이 0.96 m) 에서 벽에 닿는다.
  // 목표 0.4 m 면 d_goal + 0.1 = 0.5 m 너머는 검사하지 않아 9 셀 모두 가행, 목표 5 m 면 0 셀.
  // 파이썬 compute_mask: 0.4 → fp [39 40 41 48 49 50 57 58 59], 5.0 → fp [] (2026-10-06 [계산])
  for (const double goal : {0.4, 5.0}) {
    amr_rl_controller::ContractState s;
    for (int k = 0; k <= static_cast<int>(std::lround(goal / 0.05)); ++k) {
      s.path.push_back({0.05 * k, 0.0});
    }
    s.scan_ranges.assign(C::kLidarSamples, std::numeric_limits<double>::quiet_NaN());
    s.scan_ranges[360] = 1.05;
    s.v_meas = s.v_cmd = 0.8;
    const auto m = MaskBuilder().compute(s);
    const std::vector<int> cells{39, 40, 41, 48, 49, 50, 57, 58, 59};
    for (int i = 0; i < C::kNCells; ++i) {
      const bool sel = std::find(cells.begin(), cells.end(), i) != cells.end();
      const std::size_t k = static_cast<std::size_t>(i);
      EXPECT_EQ(m.speed_ok[k] && m.jerk_ok[k], sel) << goal << " " << i;
      EXPECT_EQ(m.fp_ok[k], sel && goal < 1.0) << goal << " " << i;
    }
    EXPECT_EQ(m.n_valid, goal < 1.0 ? 9 : 0);
  }
}

TEST(ContractRules, ProjectionTieTakesSmallerArcLength)
{
  // 갔다 돌아오는 경로: (1, ±0.1) 은 두 구간에서 같은 거리 → 작은 호길이 1.0
  // (파이썬 project_onto_path 결과 (1.0, 0.1, ±0.1, 0.0))
  const amr_rl_controller::ContractPath path({{0.0, 0.0}, {2.0, 0.0}, {0.0, 0.0}});
  for (const double y : {0.1, -0.1}) {
    const auto p = path.project({1.0, y});
    EXPECT_NEAR(p.s, 1.0, 1e-12) << y;
    EXPECT_NEAR(p.dist, 0.1, 1e-12) << y;
    EXPECT_NEAR(p.signed_e, y, 1e-12) << y;
    EXPECT_NEAR(p.seg_yaw, 0.0, 1e-12) << y;
  }
}
