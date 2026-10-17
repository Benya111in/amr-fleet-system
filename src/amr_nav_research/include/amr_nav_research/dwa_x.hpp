// amr_nav_research::DwaPlannerX — amr_navigation::core::DwaPlanner 의 연구 사본
// (docs/research/paper-research/prereg.md §2, contract.md §1).
//
// 원본 코어(src/amr_navigation/src/core/dwa.cpp)를 그대로 옮기고 연구 스위치만 더했다.
// 스위치가 모두 기본값이면 출력이 원본과 비트 단위로 같다 (test_dwa_x_equivalence).
// 자료형(DwaConfig·DwaInput·DwaResult·DwaCandidate·DwaWindow)은 원본 헤더의 것을 그대로 쓰고,
// 연구용 입력·출력은 그것을 상속한 DwaInputX·DwaResultX 에 더한다.
//
// 연구 스위치 (ResearchConfig):
//   R1 reset_persist_cycles K, reset_hard_v
//      원본 창 중심 규칙: |v_last − v_meas| ≤ window_reset_v 이고
//      |w_last − w_meas| ≤ window_reset_w 이면 직전 명령, 아니면 측정값.
//      K > 0 이면 차가 임계를 K 주기 연속 넘을 때만 측정값으로 간다.
//      |v_last − v_meas| > reset_hard_v 이면 K 와 무관하게 즉시 측정값.
//   R2 jerk_consistent, jerk_lim
//      격자 표본 창(= 방출 명령 창)을 [v_c + (a_last − jΔt)Δt, v_c + (a_last + jΔt)Δt]
//      ∩ 가속 창으로 좁힌다. a_last 는 직전 방출 가속 (v_emit − v_c)/Δt 이다.
//      제동 후보(M4-d)는 예외 — 좁히기 전 가속 창으로만 자른다.
//      각속도: 하류 velocity_profiler 가 각저크(max_angular_jerk)를 거는 것은
//      제자리 회전(|v*| ≤ curvature_min_speed)일 때뿐이다. 그래서 창 전체가 그
//      영역(|v| ≤ 0.05 m/s)일 때만 jerk_lim_theta 로 같은 띠를 건다.
//      주행 중(곡률 보존 모드)에는 ω 를 건드리지 않는다.
//   R3 pp_candidate
//      순수 추종 후보 (v*, v*·κ_pp) 를 격자와 제동 후보 사이에 하나 더한다.
//      v* = 표본 창으로 자른 목표 속도 v_des (목표 접근 상한·헤딩 감속·정렬 포함).
//      L = clip(gain·|v*| + offset, min, max) — 헤딩 항과 같은 lookahead.
//      목표점 = 로봇 투영 호길이 + L 의 경로 점.
//      κ_pp = core::PurePursuit::curvatureToPoint(x_L, y_L) = 2·y_L / (x_L² + y_L²).
//      ω 는 표본 창으로 자르고, 다른 후보와 같은 충돌 검사·비용으로 평가한다.
//
// 주의: 원본에서 옮긴 주석의 "R1/R2" 는 sustained_sampling (dynamic-avoidance 브리프)
// 이야기다. 이 사본이 더한 연구 스위치 R1–R3 (paper-research prereg §2) 와 다르다.
//
// 주기 사이 상태 (ResearchState): compute() 를 const 로 두려고 입력(DwaInputX::research)
// 으로 받아 결과(DwaResultX::next)로 돌려준다. 호출자(플러그인·chain_sim_x)가 다음
// 주기에 그대로 넘긴다. has_last 가 false 면 코어가 상태를 0 으로 본다
// (직전 명령이 없으면 연속 주기 수·직전 가속도 없다).
#ifndef AMR_NAV_RESEARCH__DWA_X_HPP_
#define AMR_NAV_RESEARCH__DWA_X_HPP_

#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "amr_navigation/core/dwa.hpp"

namespace amr_nav_research
{

using amr_navigation::core::DwaCandidate;
using amr_navigation::core::DwaConfig;
using amr_navigation::core::DwaInput;
using amr_navigation::core::DwaResult;
using amr_navigation::core::DwaWindow;
using amr_navigation::core::FootprintCostFn;
using amr_navigation::core::PointCostFn;
using amr_navigation::core::Pose2D;
using amr_navigation::core::YieldConfig;

/// 연구 스위치. 기본값 = 끔 = 원본과 같은 동작.
struct ResearchConfig
{
  int reset_persist_cycles{0};   ///< R1 K. 0 = 원본 규칙
  double reset_hard_v{0.6};      ///< R1 [m/s] 이보다 큰 선속도 차는 즉시 재설정 (K > 0 일 때만)
  bool jerk_consistent{false};   ///< R2
  double jerk_lim{2.0};          ///< R2 [m/s³] (하류 velocity_profiler 선저크와 같은 값)
  bool pp_candidate{false};      ///< R3
};

/// 주기 사이에 이어지는 상태 (호출자가 DwaResultX::next 를 다음 입력으로 넘긴다).
struct ResearchState
{
  /// R1: 직전 주기까지 명령–측정 차가 임계를 넘은 연속 주기 수.
  int persist_count{0};
  /// R2: 직전 방출 선가속 (v_emit − v_c)/Δt_c [m/s²].
  double a_last{0.0};
  /// R2: 직전 방출 각가속 (w_emit − w_c)/Δt_c [rad/s²].
  double alpha_last{0.0};
};

/// 창 중심이 측정값으로 돌아간 이유 (재설정 사건만. 직전 명령이 없거나 M3 절제는 사건 아님).
enum class ResetReason : int
{
  kNone = 0,
  kThreshold = 1,   ///< 원본 규칙 (K = 0): 차가 임계를 넘었다
  kHard = 2,        ///< R1: 선속도 차 > reset_hard_v
  kPersist = 3,     ///< R1: 차가 K 주기 연속 임계를 넘었다
};

const char * resetReasonName(ResetReason r);

/// 창 중심 판정 결과 (R1 과 재설정 기록이 쓴다).
struct WindowCenterX
{
  double v_c{0.0};
  double w_c{0.0};
  bool measured{false};     ///< 창 중심 = 측정값
  ResetReason reason{ResetReason::kNone};   ///< kNone 이 아니면 재설정 사건
  bool suppressed{false};   ///< R1 이 재설정을 미뤘다 (차는 넘었고 직전 명령 중심 유지)
  double gap_v{0.0};        ///< v_last − v_meas (부호 있음)
  double gap_w{0.0};        ///< w_last − w_meas
  int persist_count{0};     ///< 이번 주기까지 차가 임계를 넘은 연속 주기 수
};

struct DwaInputX : public DwaInput
{
  DwaInputX() = default;
  explicit DwaInputX(const DwaInput & base)
  : DwaInput(base) {}
  ResearchState research;
};

struct DwaResultX : public DwaResult
{
  WindowCenterX center;
  ResearchState next;           ///< 다음 주기에 넘길 상태
  DwaWindow sample_window;      ///< 실제 격자 표본(방출) 창. R2 가 꺼져 있으면 window 와 같다
  bool jerk_applied{false};     ///< R2 가 이 주기 창을 좁혔다
  bool jerk_angular{false};     ///< R2 각저크 띠도 걸었다 (제자리 회전 영역)
  bool pp_added{false};         ///< R3 후보를 더했다
  bool pp_selected{false};      ///< R3 후보가 선택됐다
  double pp_v{0.0};
  double pp_w{0.0};
  double pp_kappa{0.0};
};

class DwaPlannerX
{
public:
  explicit DwaPlannerX(
    const DwaConfig & config = DwaConfig(),
    const ResearchConfig & research = ResearchConfig());

  void setConfig(const DwaConfig & config) {config_ = config;}
  const DwaConfig & config() const {return config_;}
  void setResearch(const ResearchConfig & research) {research_ = research;}
  const ResearchConfig & research() const {return research_;}

  DwaResultX compute(
    const DwaInputX & in, const FootprintCostFn & footprint_cost,
    const PointCostFn & point_cost) const;

  // --- 단계별 공개 함수 (원본과 같은 이름·뜻) ---
  DwaWindow dynamicWindow(double v_c, double w_c, double v_cap) const;
  std::vector<std::pair<double, double>> sampleVelocities(
    const DwaWindow & win, double v_c, double w_c) const;
  double simTime(double v) const;
  std::vector<Pose2D> rolloutRamp(
    const Pose2D & start, double v0, double w0, double v_t, double w_t, double T,
    std::vector<double> * arc = nullptr, std::vector<double> * speeds = nullptr) const;
  static void rampStep(
    double * v, double * a, double target, double a_max, double j_max,
    double dt);
  std::vector<Pose2D> rollout(const Pose2D & start, double v, double w, double T) const;
  static double stoppingDistance(double v, double a, double j, double t_c);
  /// 원본 창 중심 규칙 (R1 없음).
  std::pair<double, double> windowCenter(const DwaInput & in) const;
  YieldConfig yieldConfig() const;
  std::vector<Pose2D> recenterPath(
    const std::vector<Pose2D> & path, const PointCostFn & point_cost, bool end_is_goal,
    std::size_t * shifted = nullptr) const;
  void textbookTerms(
    DwaCandidate * c, const DwaInput & in, const std::vector<Pose2D> & path,
    const std::vector<double> & cum_s, double s_robot, bool aligning, double goal_yaw,
    double v_des_goal, double clear_abs) const;

  // --- 연구 단계 ---
  /// R1 창 중심 규칙. st 는 직전 주기 상태 (in.has_last 가 false 면 무시).
  WindowCenterX windowCenterX(const DwaInput & in, const ResearchState & st) const;
  /// R2 방출 창: win(가속 창) ∩ 저크 띠. 교집합이 비면 win 안에서 띠에 가장 가까운 끝.
  /// angular: 각저크 띠를 걸었으면 true.
  DwaWindow jerkWindow(
    const DwaWindow & win, double v_c, double w_c, const ResearchState & st,
    bool * angular = nullptr) const;
  /// 격자는 grid 창, 제동 후보는 brake 창 (R2 예외), pp 가 있으면 제동 후보 앞에 넣는다.
  std::vector<std::pair<double, double>> sampleVelocitiesX(
    const DwaWindow & grid, const DwaWindow & brake, double v_c, double w_c,
    const std::pair<double, double> * pp = nullptr) const;

private:
  DwaConfig config_;
  ResearchConfig research_;
};

}  // namespace amr_nav_research

#endif  // AMR_NAV_RESEARCH__DWA_X_HPP_
