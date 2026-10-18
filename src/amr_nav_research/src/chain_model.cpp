#include "amr_nav_research/chain_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace amr_nav_research
{

void chainModelInit(ChainModelState * s, const ChainModelConfig & c, double v, double w)
{
  s->init = true;
  s->lin.setLimits(c.lin);
  s->ang.setLimits(c.ang);
  s->lin.reset(v, 0.0);
  s->ang.reset(w, 0.0);
  s->w_ref = w;
  s->curvature_mode = false;
  s->out_v = v;
  s->out_w = w;
  const std::size_t n = static_cast<std::size_t>(
    std::max(0L, std::lround(c.delay / std::max(1e-6, c.servo_dt))));
  s->buf_v.assign(n, v);
  s->buf_w.assign(n, w);
  s->sv = v;
  s->sw = w;
  s->step = 0;
}

namespace
{
// 서보 한 스텝: 순수 지연 → 1차 지연 → 가속 제한 (chain_sim Servo 와 같은 식)
double servoStep(
  std::vector<double> * buf, double * value, double cmd, double tau, double a_max,
  double dt)
{
  if (!buf->empty()) {
    buf->push_back(cmd);
    cmd = buf->front();
    buf->erase(buf->begin());
  }
  const double alpha = tau > 0.0 ? 1.0 - std::exp(-dt / tau) : 1.0;
  const double lim = a_max > 0.0 ? a_max * dt : 1e18;
  *value += std::clamp(alpha * (cmd - *value), -lim, lim);
  return *value;
}
}  // namespace

void chainModelAdvance(
  ChainModelState * s, const ChainModelConfig & c, double v_cmd, double w_cmd, double duration)
{
  s->lin.setLimits(c.lin);
  s->ang.setLimits(c.ang);
  const double dt = std::max(1e-6, c.servo_dt);
  const int n = std::max(0, static_cast<int>(std::lround(duration / dt)));
  const int every = std::max(1, c.profiler_every);
  const double dt_p = dt * every;
  const double v_t = std::clamp(v_cmd, c.lin.v_min, c.lin.v_max);
  const double w_t = std::clamp(w_cmd, c.ang.v_min, c.ang.v_max);
  for (int k = 0; k < n; ++k) {
    if (s->step % every == 0) {
      // velocity_profiler 기준 경로 (core::VelocityProfiler::update 의 v_ref·w_ref 부분)
      const double v_ref = s->lin.step(v_t, dt_p);
      const bool curvature = std::abs(v_t) > c.curvature_min_speed;
      if (curvature) {
        const double kappa = w_t / v_t;
        const double w_des = std::clamp(kappa * v_ref, c.ang.v_min, c.ang.v_max);
        const double dw = c.ang.a_max * dt_p;
        s->w_ref = std::clamp(w_des, s->w_ref - dw, s->w_ref + dw);
        s->curvature_mode = true;
        s->ang.reset(s->w_ref, 0.0);
      } else {
        if (s->curvature_mode) {
          s->ang.reset(s->w_ref, 0.0);
          s->curvature_mode = false;
        }
        s->w_ref = s->ang.step(w_t, dt_p);
      }
      s->out_v = v_ref;
      s->out_w = s->w_ref;
    }
    servoStep(&s->buf_v, &s->sv, s->out_v, c.tau, c.servo_accel_v, dt);
    servoStep(&s->buf_w, &s->sw, s->out_w, c.tau, c.servo_accel_w, dt);
    ++s->step;
  }
}

}  // namespace amr_nav_research
