#include "amr_nav_research/research_io.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace amr_nav_research
{

const char * resetPhase(
  const WindowCenterX & c, double v_last, double w_last, double window_reset_v,
  double window_reset_w)
{
  bool linear = std::abs(c.gap_v) > window_reset_v;
  if (c.reason == ResetReason::kHard) {
    linear = true;   // hard 는 선속도 차로만 판정한다
  } else if (std::isfinite(c.v_hat) && std::isfinite(c.w_hat)) {
    // F1: 어느 축의 모형 괴리가 (원래 임계로 정규화해) 더 큰가
    const double v_meas = v_last - c.gap_v;
    const double w_meas = w_last - c.gap_w;
    linear = std::abs(v_meas - c.v_hat) / std::max(1e-9, window_reset_v) >=
      std::abs(w_meas - c.w_hat) / std::max(1e-9, window_reset_w);
  }
  if (linear) {
    return c.gap_v > 0.0 ? "accel" : "decel";
  }
  if (std::abs(v_last) < 0.05 || std::abs(w_last) >= 0.3) {
    return "turn";
  }
  return "cruise";
}

ResetLogger::~ResetLogger()
{
  close();
}

bool ResetLogger::open(const std::string & path, const std::string & extra_header)
{
  close();
  if (path.empty()) {
    return false;
  }
  f_ = std::fopen(path.c_str(), "a");
  if (f_ == nullptr) {
    return false;
  }
  std::fseek(f_, 0, SEEK_END);
  if (std::ftell(f_) == 0) {
    // 2 라운드: 맨 끝에 F1 예상 측정 v_hat,w_hat (모형이 꺼져 있으면 nan) — 앞 열은 그대로
    std::fprintf(
      f_,
      "t,v_last,v_meas,w_last,w_meas,gap_v,gap_w,persist_count,reason,d_goal,phase%s%s,"
      "v_hat,w_hat\n", extra_header.empty() ? "" : ",", extra_header.c_str());
    std::fflush(f_);
  }
  return true;
}

void ResetLogger::close()
{
  if (f_ != nullptr) {
    std::fclose(f_);
    f_ = nullptr;
  }
}

bool ResetLogger::write(
  double t, const DwaInput & in, const WindowCenterX & c, double d_goal, double window_reset_v,
  double window_reset_w, const std::string & extra)
{
  if (f_ == nullptr || c.reason == ResetReason::kNone) {
    return false;
  }
  std::fprintf(
    f_, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%s,%.4f,%s%s%s,%.4f,%.4f\n", t, in.v_last,
    in.v_meas, in.w_last, in.w_meas, c.gap_v, c.gap_w, c.persist_count, resetReasonName(c.reason),
    std::isfinite(d_goal) ? d_goal : -1.0,
    resetPhase(c, in.v_last, in.w_last, window_reset_v, window_reset_w),
    extra.empty() ? "" : ",", extra.c_str(), c.v_hat, c.w_hat);
  std::fflush(f_);
  return true;
}

bool GoalCheckEmu::reached(double dist, double dyaw)
{
  if (check_xy_) {
    if (!(dist <= xy_tol_)) {
      return false;
    }
    if (stateful_) {
      check_xy_ = false;   // Nav2: 허용치 안에 한 번 들면 xy 검사를 잠근다
    }
  }
  return std::abs(dyaw) <= yaw_tol_;
}

}  // namespace amr_nav_research
