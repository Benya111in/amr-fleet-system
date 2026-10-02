// c07: cost of the collision-probability bounds (1 core). Compile: g++ -O2 -std=c++17
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
static inline double Phi(double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); }
int main() {
  const int N = 2000000; std::mt19937 g(1); std::uniform_real_distribution<double> ud(0.0, 3.0), us(0.05, 1.2);
  std::vector<double> d(N), s(N); for (int i = 0; i < N; ++i) { d[i] = ud(g); s[i] = us(g); }
  const double R = 0.80; volatile double sink = 0;
  auto t0 = std::chrono::steady_clock::now(); double acc = 0;
  for (int i = 0; i < N; ++i) acc += Phi((R - d[i]) / s[i]);                       // half-plane bound
  auto t1 = std::chrono::steady_clock::now();
  for (int i = 0; i < N; ++i) { double h = Phi((R - d[i]) / s[i]);                  // min(HP, box), isotropic
    acc += (h - Phi((-R - d[i]) / s[i])) * (2 * Phi(R / s[i]) - 1); }
  auto t2 = std::chrono::steady_clock::now(); int ev = 0;
  for (int i = 0; i < N; ++i) { if (d[i] - R > 6 * s[i]) continue; ++ev;             // with 6-sigma prescreen
    double h = Phi((R - d[i]) / s[i]); acc += (h - Phi((-R - d[i]) / s[i])) * (2 * Phi(R / s[i]) - 1); }
  auto t3 = std::chrono::steady_clock::now();
  for (int i = 0; i < N; ++i) acc += std::hypot(d[i], s[i]);                         // distance only (GVO)
  auto t4 = std::chrono::steady_clock::now(); sink = acc;
  auto ns = [&](auto a, auto b) { return std::chrono::duration<double, std::nano>(b - a).count() / N; };
  std::printf("HP bound      : %.1f ns/eval\nmin(HP,box)   : %.1f ns/eval\nprescreened   : %.1f ns/eval (%.0f%% evaluated)\nhypot (GVO)   : %.1f ns/eval\n",
              ns(t0, t1), ns(t1, t2), ns(t2, t3), 100.0 * ev / N, ns(t3, t4));
  // worst-case cycle: 342 samples x (15 risk + 25 tail) poses x 10 tracks bound + 342 x 50 x 2 x 10 x 3 distances
  double worst = 342.0 * 40 * 10 * ns(t1, t2) * 1e-6 + 342.0 * 50 * 2 * 10 * 3 * ns(t3, t4) * 1e-6;
  std::printf("worst-case dynamic terms (10 tracks, no prescreen): %.2f ms\n", worst);
  return (int)(sink * 0);
}
