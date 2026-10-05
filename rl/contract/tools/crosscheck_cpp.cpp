// 계약 기준 구현(파이썬)이 이식한 C++ 원본 함수와 같은 값을 내는지 대조하는 도구.
// 결정적 난수 입력에 대해 C++ 원본의 출력을 줄 단위로 찍는다 → crosscheck_cpp.py 가 파이썬과 비교한다.
// 빌드·실행: rl/contract/tools/crosscheck_cpp.sh (amr-ai/rl:0.3 안, ROS 불필요)
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <vector>

#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/velocity_obstacle.hpp"
#include "amr_navigation/core/velocity_profiler.hpp"
#include "amr_perception/safety_gate.hpp"

namespace nav = amr_navigation::core;
namespace per = amr_perception;

static uint64_t g_state = 88172645463325252ULL;
static double urand(double lo, double hi)
{
  g_state ^= g_state << 13;
  g_state ^= g_state >> 7;
  g_state ^= g_state << 17;
  return lo + (hi - lo) * static_cast<double>(g_state % 1000000007ULL) / 1000000007.0;
}

int main()
{
  // 1) chordVelocity, velocityObstacleTime
  for (int i = 0; i < 2000; ++i) {
    const double v = urand(0.0, 1.2), w = urand(-1.5, 1.5);
    const auto c = nav::chordVelocity(0.0, v, w, 2.0);
    const nav::Point2D p{urand(-6.0, 6.0), urand(-6.0, 6.0)};
    const nav::Point2D wr{urand(-3.0, 3.0), urand(-3.0, 3.0)};
    const double R = urand(0.3, 1.5), tau = urand(0.5, 5.0);
    const double t = nav::velocityObstacleTime(p, wr, R, tau);
    std::printf("vo %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",
      v, w, c.x, c.y, p.x, p.y, wr.x, wr.y, R, tau, std::isfinite(t) ? t : -1.0);
  }
  // 2) simTime, rollout, 제동 후보 (control_period 0.1 = 계약 결정 주기)
  nav::DwaConfig cfg;
  cfg.limits.max_vel_x = 1.0;
  cfg.limits.min_vel_x = 0.0;
  cfg.control_period = 0.1;
  cfg.vx_samples = 11;
  cfg.vth_samples = 31;
  nav::DwaPlanner dwa(cfg);
  // v 는 하드웨어 상한 2.0 까지: |v| > 1.0 이면 T_sim 이 sim_dt 의 배수가 아니다 (n = ceil)
  for (int i = 0; i < 300; ++i) {
    const double v = urand(0.0, 2.0), w = urand(-1.5, 1.5);
    const double T = dwa.simTime(v);
    const auto poses = dwa.rollout(nav::Pose2D{0.0, 0.0, 0.0}, v, w, T);
    const auto & last = poses.back();
    const double vc = urand(0.0, 1.0), wc = urand(-1.5, 1.5);
    const auto win = dwa.dynamicWindow(vc, wc, 1.0);
    const auto s = dwa.sampleVelocities(win, vc, wc);
    std::printf("dwa %.17g %.17g %.17g %zu %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",
      v, w, T, poses.size(), last.x, last.y, last.theta, vc, wc, s.back().first, s.back().second);
  }
  // 3) 프로파일러 기준 경로 (PID 끔, 측정 없음)
  nav::VelocityProfilerConfig pc;
  pc.use_pid = false;
  nav::VelocityProfiler prof(pc);
  double vt = 0.0, wt = 0.0;
  for (int k = 0; k < 600; ++k) {
    if (k % 5 == 0) {
      vt = std::min(1.0, std::max(0.0, vt + urand(-0.1, 0.1)));
      wt = std::min(1.5, std::max(-1.5, wt + urand(-0.2, 0.2)));
    }
    const auto o = prof.update(vt, wt, false, 0.0, 0.0, 0.02);
    std::printf("prof %.17g %.17g %.17g %.17g %.17g\n", vt, wt, o.v_ref, o.w_ref, o.a_ref);
  }
  // 4) safety: clearanceSpeedLimit, maxPointSpeed, classifyZone
  per::SafetyParams sp;
  for (int i = 0; i < 1000; ++i) {
    const double d = urand(0.0, 4.0), v = urand(-0.5, 2.0), w = urand(-1.5, 1.5);
    std::printf("saf %.17g %.17g %.17g %.17g %.17g %d\n", d, v, w,
      per::SafetyGate::clearanceSpeedLimit(d, sp),
      per::SweptFootprint::maxPointSpeed(0.3, 0.2, per::Motion{v, w}),
      static_cast<int>(per::SafetyGate::classifyZone(d, sp)));
  }
  return 0;
}
