// PID 게인 튜닝 도구 — 1차 지연 + 순수 지연 플랜트 모델에서
//   모델 추종 2-자유도 PI 게인을 격자 탐색한다.
//
//   ros2 run amr_navigation pid_tuning [csv_path]
//
// 구조 (velocity_profiler_node 와 같음): u = r + PI(y_m
//   − y), y_m = M(r), M = e^{−T_d s}/(T_m s + 1).
// 평가 (50 Hz, 각 게인 쌍):
//   os_nom   명목 플랜트(K 1.0, T_p 0.08, T_d 0.04)에서 0.15 m/s 스텝 오버슈트 [%]
//   os_worst 불일치 플랜트 27 종(K 0.8/0.9/1.0 × T_p 0.05/0.08/0.12
//   × T_d 0.02/0.04/0.06) 최악 오버슈트 [%]
//            (속도 서보는 슬립·포화로 이득이 1 이하가 되므로 K > 1 은 두지 않는다)
//   t_dist   입력 외란 −0.1 m/s (경사·적재 저항) 인가 후 |e| < 0.01 m/s 회복 시간 [s] (명목 플랜트)
//   t_gain   이득 0.8 플랜트(슬립)에서 0.5 m/s 스텝 정상오차 < 0.01 까지 시간 [s]
//   noise    폐루프 정상상태에서 측정 잡음 σ 0.014 m/s(명세 4.1 슬립 잡음, v = 2 m/s 기준)가 만드는
//            명령 표준편차 [m/s]
// 선택 규칙: os_nom ≤ 1 %, os_worst ≤ 5 %, noise ≤ 0.0075 m/s(연구 브리프 path-tracking §2.6.1 의
//   지령 잡음 예산 0.007 m/s 수준)를 만족하는 쌍 중 t_dist + t_gain 최소.
// 정렬: 주기 k 에 읽는 측정은 k−1 까지의 명령이 만든 응답이므로,
//   모델 출력도 이번 기준을 넣기 전 값을 쓴다.
// 결과 표는 docs/algorithms/path_tracking.md "PID 게인 튜닝" 절에 옮긴다.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "amr_navigation/core/pid.hpp"

using amr_navigation::core::FirstOrderPlant;
using amr_navigation::core::Pid;
using amr_navigation::core::PidConfig;

namespace
{
constexpr double kDt = 0.02;
constexpr double kModelTp = 0.08;
constexpr double kModelTd = 0.04;

struct Plant
{
  double k;
  double tp;
  double td;
};

struct Metrics
{
  double kp{0.0};
  double ki{0.0};
  double os_nom{0.0};
  double os_worst{0.0};
  double t_dist{0.0};
  double t_gain{0.0};
  double noise{0.0};
  bool ok{false};
};

PidConfig gains(double kp, double ki)
{
  PidConfig c;
  c.kp = kp;
  c.ki = ki;
  c.setpoint_weight = 1.0;
  c.tracking_time = kp > 0.0 && ki > 0.0 ? std::sqrt(kp / ki) : 0.1;
  return c;
}

// 스텝 응답 오버슈트 [%]
double stepOvershoot(double kp, double ki, const Plant & p, double r)
{
  Pid pid(gains(kp, ki));
  FirstOrderPlant plant(p.k, p.tp, p.td, kDt);
  FirstOrderPlant model(1.0, kModelTp, kModelTd, kDt);
  double y = 0.0;
  double ymax = 0.0;
  for (int k = 0; k < 150; ++k) {
    const double ym = model.output();
    model.step(r);
    y = plant.step(pid.update(ym, y, kDt, r, -2.0, 3.0));
    ymax = std::max(ymax, y);
  }
  return std::max(0.0, (ymax - r) / r * 100.0);
}

// 외란/이득 오차 회복 시간: 마지막으로 |r − y| ≥ 0.01 이었던 시각 − 외란 시각
double recoveryTime(double kp, double ki, const Plant & p, double r, double disturbance)
{
  Pid pid(gains(kp, ki));
  FirstOrderPlant plant(p.k, p.tp, p.td, kDt, disturbance == 0.0 ? 0.0 : r);
  FirstOrderPlant model(1.0, kModelTp, kModelTd, kDt, disturbance == 0.0 ? 0.0 : r);
  double y = plant.output();
  double last_bad = 0.0;
  for (int k = 0; k < 500; ++k) {
    const double ym = model.output();
    model.step(r);
    y = plant.step(pid.update(ym, y, kDt, r, -2.0, 3.0) + disturbance);
    if (std::abs(r - y) >= 0.01) {
      last_bad = (k + 1) * kDt;
    }
  }
  return last_bad;
}

double noiseStd(double kp, double ki)
{
  Pid pid(gains(kp, ki));
  FirstOrderPlant plant(1.0, kModelTp, kModelTd, kDt, 1.0);
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 0.014);
  double sum = 0.0;
  double sum2 = 0.0;
  const int N = 5000;
  double y = 1.0;
  for (int k = 0; k < N + 500; ++k) {
    const double u = pid.update(1.0, y + n(rng), kDt, 1.0, -2.0, 3.0);
    y = plant.step(u);
    if (k >= 500) {
      sum += u - 1.0;
      sum2 += (u - 1.0) * (u - 1.0);
    }
  }
  const double mean = sum / N;
  return std::sqrt(std::max(0.0, sum2 / N - mean * mean));
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::vector<double> kps{0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1.0};
  const std::vector<double> kis{0.5, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0};
  std::vector<Plant> mismatch;
  for (double k : {0.8, 0.9, 1.0}) {
    for (double tp : {0.05, 0.08, 0.12}) {
      for (double td : {0.02, 0.04, 0.06}) {
        mismatch.push_back({k, tp, td});
      }
    }
  }
  const Plant nominal{1.0, kModelTp, kModelTd};
  std::vector<Metrics> all;
  for (double kp : kps) {
    for (double ki : kis) {
      Metrics m;
      m.kp = kp;
      m.ki = ki;
      m.os_nom = stepOvershoot(kp, ki, nominal, 0.15);
      for (const auto & p : mismatch) {
        m.os_worst = std::max(m.os_worst, stepOvershoot(kp, ki, p, 0.15));
      }
      m.t_dist = recoveryTime(kp, ki, nominal, 1.0, -0.1);
      m.t_gain = recoveryTime(kp, ki, {0.8, kModelTp, kModelTd}, 0.5, 0.0);
      m.noise = noiseStd(kp, ki);
      m.ok = m.os_nom <= 1.0 && m.os_worst <= 5.0 && m.noise <= 0.0075;
      all.push_back(m);
    }
  }
  FILE * csv = argc > 1 ? std::fopen(argv[1], "w") : nullptr;
  if (csv != nullptr) {
    std::fprintf(csv, "kp,ki,os_nom_pct,os_worst_pct,t_dist_s,t_gain_s,noise_std,ok\n");
  }
  std::printf(
    "| kp | ki | os_nom [%%] | os_worst [%%] | t_dist [s] | t_gain [s] | noise [m/s] | ok |\n");
  std::printf("| --- | --- | --- | --- | --- | --- | --- | --- |\n");
  const Metrics * best = nullptr;
  for (const auto & m : all) {
    std::printf(
      "| %.1f | %.1f | %.2f | %.2f | %.2f | %.2f | %.4f | %s |\n", m.kp, m.ki, m.os_nom, m.os_worst,
      m.t_dist, m.t_gain, m.noise, m.ok ? "yes" : "no");
    if (csv != nullptr) {
      std::fprintf(
        csv, "%.2f,%.2f,%.4f,%.4f,%.3f,%.3f,%.5f,%d\n", m.kp, m.ki, m.os_nom, m.os_worst, m.t_dist,
        m.t_gain, m.noise, m.ok ? 1 : 0);
    }
    if (m.ok && (best == nullptr || m.t_dist + m.t_gain < best->t_dist + best->t_gain)) {
      best = &m;
    }
  }
  if (csv != nullptr) {
    std::fclose(csv);
  }
  if (best != nullptr) {
    std::printf(
      "\nselected: kp %.2f ki %.2f (T_t = sqrt(kp/ki) = %.3f s) os_nom %.2f%% os_worst %.2f%% "
      "t_dist %.2f s t_gain %.2f s noise %.4f m/s\n", best->kp, best->ki,
      std::sqrt(best->kp / best->ki), best->os_nom, best->os_worst, best->t_dist, best->t_gain,
      best->noise);
    return 0;
  }
  std::printf("\nno gain pair satisfies the constraints\n");
  return 1;
}
