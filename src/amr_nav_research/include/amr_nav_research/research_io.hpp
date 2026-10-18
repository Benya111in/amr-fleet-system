// 연구 계측·판정 공용 도구 (플러그인 DwaControllerX 와 chain_sim_x 가 같이 쓴다).
//
// 1) 재설정 기록 CSV (contract.md §1 log_resets)
//    창 중심이 직전 명령에서 측정값으로 돌아간 주기마다 한 줄:
//      t, v_last, v_meas, w_last, w_meas, gap_v, gap_w, persist_count, reason, d_goal, phase
//    뒤에 쓰는 쪽의 식별 열을 덧붙인다 (chain_sim_x: scene, arm, seed /
//    플러그인: plan_id, controller). 2 라운드부터 맨 끝에 v_hat, w_hat (F1 사슬 모형의 예상
//    측정, 모형이 꺼져 있으면 nan). gap 은 부호 있는 차 (v_last − v_meas).
//    d_goal 은 남은 경로 거리 (없으면 -1). 기록하지 않는 것: 직전 명령이 없는 주기,
//    M3 절제(window_center_measured), R1 이 미룬 주기 (DwaResultX::center.suppressed 로 센다).
//
// 2) 국면 phase (재설정 순간의 명령·측정 관계로 정한다)
//    - 선속도 재설정이란: |gap_v| > window_reset_v 이거나 hard. F1(사슬 모형)이면 원래 임계로
//      정규화한 모형 괴리 |v_meas − v̂|/window_reset_v ≥ |w_meas − ŵ|/window_reset_w 일 때.
//    - 선속도 재설정:
//        gap_v > 0 (명령이 측정보다 위, 로봇이 가속 명령을 못 따라옴) → accel
//        gap_v ≤ 0 (명령이 측정보다 아래, 감속 명령을 못 따라옴)      → decel
//    - 그 밖 (각속도 재설정):
//        제자리 회전 명령(|v_last| < 0.05 m/s) 또는 |w_last| ≥ 0.3 rad/s → turn
//        그 밖 (거의 직진인데 ω 측정이 튐)                            → cruise
//
// 3) 목표 판정 (Nav2 SimpleGoalChecker 흉내)
//    원본 chain_sim: 매 주기 dist ≤ xy_tol && |dyaw| ≤ yaw_tol (stateful false).
//    stateful 이면 Nav2 1.1.20 nav2_controller/plugins/simple_goal_checker.cpp 처럼
//    xy 허용치에 한 번 들면 xy 검사를 잠그고 그 뒤로는 yaw 만 본다
//    (reset() 은 controller_server::setPlannerPath 에서만 — 새 경로를 받을 때).
//    비교 연산은 원본 chain_sim 과 같게 둔다: dist ≤ tol (Nav2 는 dx²+dy² > tol² 이면 실패,
//    같은 뜻), |dyaw| ≤ tol (Nav2 1.1.20 은 < — 정확히 같은 값일 때만 다르다).
#ifndef AMR_NAV_RESEARCH__RESEARCH_IO_HPP_
#define AMR_NAV_RESEARCH__RESEARCH_IO_HPP_

#include <cstdio>
#include <string>

#include "amr_nav_research/dwa_x.hpp"

namespace amr_nav_research
{

/// 재설정 국면 (위 2) 의 규칙).
const char * resetPhase(
  const WindowCenterX & c, double v_last, double w_last, double window_reset_v,
  double window_reset_w);

/// 재설정 기록 CSV. 빈 경로면 아무것도 하지 않는다. 덧붙이기로 열고, 빈 파일이면 머리줄을 쓴다.
class ResetLogger
{
public:
  ResetLogger() = default;
  ~ResetLogger();
  ResetLogger(const ResetLogger &) = delete;
  ResetLogger & operator=(const ResetLogger &) = delete;

  /// extra_header: 덧붙일 식별 열 이름 (예: "scene,arm,seed"). 실패하면 false.
  bool open(const std::string & path, const std::string & extra_header);
  void close();
  bool isOpen() const {return f_ != nullptr;}

  /// 재설정 사건이면 한 줄 쓰고 true. extra: 식별 열 값 (쉼표로 이은 문자열).
  bool write(
    double t, const DwaInput & in, const WindowCenterX & c, double d_goal,
    double window_reset_v, double window_reset_w, const std::string & extra);

private:
  std::FILE * f_{nullptr};
};

/// Nav2 SimpleGoalChecker 흉내 (위 3)).
class GoalCheckEmu
{
public:
  GoalCheckEmu(double xy_tol, double yaw_tol, bool stateful)
  : xy_tol_(xy_tol), yaw_tol_(yaw_tol), stateful_(stateful) {}
  void reset() {check_xy_ = true;}
  /// dist: 목표까지 거리 [m], dyaw: wrap 된 yaw 오차 [rad].
  bool reached(double dist, double dyaw);
  /// stateful 에서 xy 가 잠겼는가.
  bool xyLatched() const {return !check_xy_;}

private:
  double xy_tol_;
  double yaw_tol_;
  bool stateful_;
  bool check_xy_{true};
};

}  // namespace amr_nav_research

#endif  // AMR_NAV_RESEARCH__RESEARCH_IO_HPP_
