// probe_chain_r3c — prereg_r3c.md §3 E5 탐침 P1·P3 (원고 식 (1)·(4) 확인).
//
//   probe_chain_r3c <out_dir>
//
// chain_sim_x 와 같은 하류 사슬과 같은 시각표를 쓴다: 제어 20 Hz (k % 5 == 0) 에 명령을 바꾸고,
// core::VelocityProfiler (기본 설정 = chain_sim_x 의 prof, 50 Hz, k % 2 == 0, 되먹임 = 서보 값)
// → Servo (순수 지연 0.04 s → 1차 0.08 s → 가속 제한 1.0 m/s², 100 Hz). 제어기는 없다 (개루프).
//   P1 사슬 괴리 g(a): 명령 램프 v_c^k = min(1.0, a·(k+1)·Δt_c), a = 1.0 m/s² (0 → 1.0 m/s).
//      제어 주기마다 제어기가 보는 괴리 v_c^{k-1} − v_m 와 그 최대를 낸다. 식 (1):
//      g(a) = a²/2j + a(d + τ) = 0.25 + 0.12 = 0.37 m/s.
//   P3 정지거리: 명령 v0 ∈ {0.4, 0.6, 0.8, 1.0} m/s 를 10 s 유지(정상 주행)한 뒤 제어 주기 경계에서
//      명령을 0 으로 계단 변경하고, 그 순간부터 정지(|v_m| < 1e-6)까지 이동 거리를 잰다.
//      식 (4) (t_c = 0, SpeedProfile::stoppingDistance) · 사다리꼴 v²/2a ·
//      식 (4) (t_c = d + τ) 와 비교.
// 출력: <out_dir>/probe_chain_r3c.json (요약), p1_ramp.csv, p3_stop_v<v0>.csv (100 Hz 궤적).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "amr_navigation/core/speed_profile.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"

namespace core = amr_navigation::core;

namespace
{
constexpr double kDt = 0.01;     // 플랜트 [s]
constexpr double kDtc = 0.05;    // 제어 주기 [s]
constexpr int kStepsPerCycle = 5;
constexpr double kA = 1.0;       // 가속 한계 [m/s²]
constexpr double kJ = 2.0;       // 저크 한계 [m/s³]
constexpr double kDelay = 0.04;  // 서보 순수 지연 [s]
constexpr double kTau = 0.08;    // 서보 1차 시간 상수 [s]

// chain_sim_x 의 Servo 와 같은 식 (tools/chain_sim_x.cpp)
struct Servo
{
  Servo(double tau, double delay, double a_max)
  : tau_(tau), a_max_(a_max), buf_(static_cast<std::size_t>(std::lround(delay / kDt)), 0.0) {}
  double step(double cmd)
  {
    if (!buf_.empty()) {
      buf_.push_back(cmd);
      cmd = buf_.front();
      buf_.erase(buf_.begin());
    }
    const double alpha = 1.0 - std::exp(-kDt / tau_);
    const double lim = a_max_ * kDt;
    value_ += std::clamp(alpha * (cmd - value_), -lim, lim);
    return value_;
  }
  double tau_;
  double a_max_;
  std::vector<double> buf_;
  double value_{0.0};
};

struct Chain
{
  core::VelocityProfiler prof;
  Servo sv{kTau, kDelay, kA};
  Servo sw{kTau, kDelay, 2.0};
  double v_out{0.0};
  double w_out{0.0};
  double x{0.0};
  /// 플랜트 한 스텝 (k: 전역 스텝 번호). 반환 = 측정 속도 (서보 출력).
  double step(int k, double v_cmd)
  {
    if (k % 2 == 0) {
      const core::ProfilerOutput po = prof.update(v_cmd, 0.0, true, sv.value_, sw.value_, 0.02);
      v_out = po.v;
      w_out = po.w;
    }
    const double v = sv.step(v_out);
    sw.step(w_out);
    x += v * kDt;
    return v;
  }
};

struct P1Result
{
  double gap_max{0.0};         // 제어 주기 표본 v_c^{k-1} − v_m 최대
  double gap_at_ramp_end{0.0};  // 램프 마지막 명령 주기의 다음 주기 괴리
  double t_gap_max{0.0};
  double gap_cont_max{0.0};    // 100 Hz 연속 표본 v_c(t) − v_m(t) 최대 (v_c = 현재 유지 명령)
};

P1Result probeRamp(const std::string & csv)
{
  Chain ch;
  P1Result r;
  FILE * f = std::fopen(csv.c_str(), "w");
  if (f != nullptr) {
    std::fprintf(f, "t,v_cmd,v_prof,v_meas,gap_ctrl\n");
  }
  double v_cmd = 0.0;
  int ramp_end_cycle = -1;
  const int n = static_cast<int>(std::lround(4.0 / kDt));
  for (int k = 0; k < n; ++k) {
    double gap_ctrl = std::nan("");
    if (k % kStepsPerCycle == 0) {
      const int cyc = k / kStepsPerCycle;
      const double v_meas = ch.sv.value_;
      if (cyc > 0) {
        gap_ctrl = v_cmd - v_meas;   // 제어기가 보는 v_c^{k-1} − v_m
        if (gap_ctrl > r.gap_max) {
          r.gap_max = gap_ctrl;
          r.t_gap_max = k * kDt;
        }
        if (cyc == ramp_end_cycle + 1 && ramp_end_cycle >= 0) {
          r.gap_at_ramp_end = gap_ctrl;
        }
      }
      v_cmd = std::min(1.0, kA * (cyc + 1) * kDtc);
      if (ramp_end_cycle < 0 && v_cmd >= 1.0 - 1e-12) {
        ramp_end_cycle = cyc;
      }
    }
    const double v = ch.step(k, v_cmd);
    r.gap_cont_max = std::max(r.gap_cont_max, v_cmd - v);
    if (f != nullptr) {
      std::fprintf(
        f, "%.2f,%.4f,%.5f,%.5f,%s\n", (k + 1) * kDt, v_cmd, ch.v_out, v,
        std::isnan(gap_ctrl) ? "" : std::to_string(gap_ctrl).c_str());
    }
  }
  if (f != nullptr) {
    std::fclose(f);
  }
  return r;
}

struct P3Result
{
  double v0{0.0};
  double v_steady{0.0};
  double d_meas{0.0};
  double t_stop{0.0};
};

P3Result probeStop(double v0, const std::string & csv)
{
  Chain ch;
  P3Result r;
  r.v0 = v0;
  FILE * f = std::fopen(csv.c_str(), "w");
  if (f != nullptr) {
    std::fprintf(f, "t,v_cmd,v_prof,v_meas,x\n");
  }
  // 10 s 정상 주행 뒤 (제어 주기 경계)
  const int k_step = static_cast<int>(std::lround(10.0 / kDt));
  const int n = k_step + static_cast<int>(std::lround(6.0 / kDt));
  double v_cmd = 0.0;
  double x0 = 0.0;
  bool stopped = false;
  for (int k = 0; k < n; ++k) {
    if (k % kStepsPerCycle == 0) {
      v_cmd = k < k_step ? v0 : 0.0;
      if (k == k_step) {
        r.v_steady = ch.sv.value_;
        x0 = ch.x;
      }
    }
    const double v = ch.step(k, v_cmd);
    if (k >= k_step && !stopped && std::abs(v) < 1e-6) {
      stopped = true;
      r.t_stop = (k + 1 - k_step) * kDt;
      r.d_meas = ch.x - x0;
    }
    if (f != nullptr && k >= k_step - 100) {
      std::fprintf(f, "%.2f,%.4f,%.5f,%.5f,%.5f\n", (k + 1) * kDt, v_cmd, ch.v_out, v, ch.x);
    }
  }
  if (!stopped) {
    r.d_meas = ch.x - x0;
    r.t_stop = -1.0;
  }
  if (f != nullptr) {
    std::fclose(f);
  }
  return r;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "사용: probe_chain_r3c <out_dir>\n");
    return 2;
  }
  const std::string out = argv[1];
  const P1Result p1 = probeRamp(out + "/p1_ramp.csv");
  const double g_formula = kA * kA / (2.0 * kJ) + kA * (kDelay + kTau);
  FILE * f = std::fopen((out + "/probe_chain_r3c.json").c_str(), "w");
  if (f == nullptr) {
    std::fprintf(stderr, "probe_chain_r3c: %s 에 쓸 수 없다\n", out.c_str());
    return 1;
  }
  std::fprintf(
    f, "{\"P1\": {\"a\": %.3f, \"j\": %.3f, \"d\": %.3f, \"tau\": %.3f, \"g_formula\": %.4f, "
    "\"gap_ctrl_max\": %.4f, \"t_gap_ctrl_max\": %.2f, \"gap_ctrl_at_ramp_end\": %.4f, "
    "\"gap_cont_max\": %.4f},\n \"P3\": [", kA, kJ, kDelay, kTau, g_formula, p1.gap_max,
    p1.t_gap_max, p1.gap_at_ramp_end, p1.gap_cont_max);
  const double v0s[] = {0.4, 0.6, 0.8, 1.0};
  for (int i = 0; i < 4; ++i) {
    char name[64];
    std::snprintf(name, sizeof(name), "/p3_stop_v%.1f.csv", v0s[i]);
    const P3Result r = probeStop(v0s[i], out + name);
    const double d4 = core::SpeedProfile::stoppingDistance(r.v_steady, kA, kJ, 0.0);
    const double d4s = core::SpeedProfile::stoppingDistance(r.v_steady, kA, kJ, kDelay + kTau);
    const double d4l = core::SpeedProfile::stoppingDistance(r.v_steady, kA, kJ, 0.3);
    const double dtr = r.v_steady * r.v_steady / (2.0 * kA);
    std::fprintf(
      f, "%s\n  {\"v0\": %.2f, \"v_steady\": %.5f, \"d_meas\": %.4f, \"t_stop\": %.2f, "
      "\"d_eq4_tc0\": %.4f, \"d_trapezoid\": %.4f, \"d_eq4_tc_servo\": %.4f, "
      "\"d_eq4_tc0.3\": %.4f, "
      "\"err_eq4_tc0\": %.4f, \"err_trapezoid\": %.4f, \"err_eq4_tc_servo\": %.4f}",
      i == 0 ? "" : ",", r.v0, r.v_steady, r.d_meas, r.t_stop, d4, dtr, d4s, d4l, r.d_meas - d4,
      r.d_meas - dtr,
      r.d_meas - d4s);
  }
  std::fprintf(f, "\n ]}\n");
  std::fclose(f);
  std::printf("probe_chain_r3c: %s/probe_chain_r3c.json\n", out.c_str());
  return 0;
}
