#include "amr_rl_controller/core/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace amr_rl_controller
{

namespace C = contract;

namespace
{
double clamp(double x, double lo, double hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}
}  // namespace

Command integrateB(double u0, double u1, double v_cmd, double w_cmd, double v_cap)
{
  const double a_v = clamp(u0, -1.0, 1.0) * C::kAMax;
  const double a_w = clamp(u1, -1.0, 1.0) * C::kAlphaMax;
  const double v_hi = std::min(v_cap, C::kVMaxHw);
  Command c;
  c.v = clamp(v_cmd + a_v * C::kDt, C::kVMin, std::max(C::kVMin, v_hi));
  c.w = clamp(w_cmd + a_w * C::kDt, -C::kWMax, C::kWMax);
  c.a_v = (c.v - v_cmd) / C::kDt;
  c.a_w = (c.w - w_cmd) / C::kDt;
  return c;
}

Accel cellAccel(int index)
{
  if (index < 0 || index >= C::kNCells) {
    throw std::out_of_range("셀 인덱스 범위 밖: " + std::to_string(index));
  }
  return {C::latticeAv(index / C::kNAw), C::latticeAw(index % C::kNAw)};
}

Command fallbackCommand(double v_cmd, double w_cmd)
{
  const double dv = C::kAMax * C::kDt;
  Command c;
  c.v = v_cmd > 0.0 ? std::max(0.0, v_cmd - dv) : std::min(0.0, v_cmd + dv);
  const double dw = std::min(std::abs(w_cmd), C::kAlphaMax * C::kDt);
  c.w = w_cmd != 0.0 ? w_cmd - std::copysign(dw, w_cmd) : 0.0;
  c.a_v = (c.v - v_cmd) / C::kDt;
  c.a_w = (c.w - w_cmd) / C::kDt;
  return c;
}

Command integrateC(int index, double v_cmd, double w_cmd)
{
  if (index == C::kFallbackIndex) {
    return fallbackCommand(v_cmd, w_cmd);
  }
  const Accel a = cellAccel(index);
  Command c;
  c.v = v_cmd + a.first * C::kDt;
  c.w = w_cmd + a.second * C::kDt;
  c.a_v = a.first;
  c.a_w = a.second;
  return c;
}

double accelDistance(const Accel & a, const Accel & b)
{
  const double dv = (a.first - b.first) / C::kAMax;
  const double dw = (a.second - b.second) / C::kAlphaMax;
  return dv * dv + dw * dw;
}

int nearestCell(double a_v, double a_w)
{
  int best = -1;
  double best_d = std::numeric_limits<double>::infinity();
  for (int i = 0; i < C::kNCells; ++i) {
    const double d = accelDistance(cellAccel(i), {a_v, a_w});
    if (d < best_d - 1e-12) {
      best = i;
      best_d = d;
    }
  }
  return best;
}

int nearestCellMasked(
  double a_v, double a_w, const std::array<bool, C::kNActionsC> & mask,
  const Accel & fallback_accel)
{
  int best = -1;
  double best_d = std::numeric_limits<double>::infinity();
  for (int i = 0; i < C::kNActionsC; ++i) {
    if (!mask[static_cast<std::size_t>(i)]) {
      continue;
    }
    const Accel acc = i == C::kFallbackIndex ? fallback_accel : cellAccel(i);
    const double d = accelDistance(acc, {a_v, a_w});
    if (d < best_d - 1e-12) {
      best = i;
      best_d = d;
    }
  }
  if (best < 0) {
    throw std::invalid_argument("후보가 없다 (마스크가 전부 거짓)");
  }
  return best;
}

int bcLabel(
  double v_dwa, double w_dwa, double v_cmd, double w_cmd,
  const std::array<bool, C::kNActionsC> & mask)
{
  const Command fb = fallbackCommand(v_cmd, w_cmd);
  return nearestCellMasked(
    (v_dwa - v_cmd) / C::kDt, (w_dwa - w_cmd) / C::kDt, mask, {fb.a_v, fb.a_w});
}

}  // namespace amr_rl_controller
