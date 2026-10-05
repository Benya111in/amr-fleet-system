// 횡단 게이트 — 노출 구간 **밖**에서 진입 전에 결정한다 (명세 4.7 동적 장애물 회피, T안). ROS
// 비의존.
//
// 왜 필요한가: 통합 08 실측(1,500+ 시행)에서 접촉은 전부 "보행자의 몸 원통 안에서 저속·정지" 였고,
// 원통 안에 들어간 뒤에는 어떤 지역 계획기 비용항도 로봇을 옆으로 못 옮긴다 (가로 권한 ∝ 속도).
// 그런데 첫 계획은 설계 문서가 가정한 26.6° 직선이 아니라 좁은 통로 랙의 팽창 경계를 따라 보행 차선
// y=−7 과 17° 로 나란히 붙어 달리다 x≈2.1 에서 45° 로 건넌다 (`astar_benchmark` 길이 9.185 m·랙
// 여유
// 1.202 m; 직선이면 8.94 m·0.35 m). 몸 원통 안 경로 2.85 m + 옆 차선 y=−6 띠 3.35 m = 연속 노출 ≈
// 5.5 m.
// 통로 정지선·go/hold/retreat(§2.3–2.4)는 그 기하 안에서 결정하므로 작동할 수 없었다.
//
// 정책 (출발 위상 610개 시뮬레이션에서 접촉 0/610, 최소 여유 +0.43/+0.29 m, 대기 평균 1.4~1.7 s):
//   1) 트랙마다 **도달가능 집합**: 방향을 아는 트랙(|v| ≥ vknown 인 1.5 s 변위)은 진행선 통로
//      (반폭 R + |α|·sin heading_unc) 안에서 앞으로 [0, v_max·τ], 뒤로는 **되돌아옴 모델**(정지 v/a
//      → 회전 turn_time → 재가속)이 허용하는 만큼. 방향을 모르는 트랙은 반경 v_max·τ 원판.
//      속도 **크기**는 믿지 않는다(하한 0, 상한 v_max). 관측 지연 delay 만큼 τ 를 늘린다.
//   2) 노출 구간 = 로봇이 경로를 전속으로 달릴 때 어떤 트랙이 그 시각에 닿을 수 있는 경로점의
// 연속 구간(간격 group_gap 안이면 병합). 정지점 = 첫 구간 시작 − hold_back, 단 **기하로** 배제한다:
//      방향을 아는 어떤 트랙의 진행선 통로 안(앞 전부 + 되돌아올 수 있는 뒤)이나 방향 모르는 트랙이
//      hold_exclusion_horizon 안에 닿는 자리면 그 밖까지 뒤로 물린다 (뒤따라오는 보행자도 같다).
//   3) 정지점 앞에서 매 주기 재판정: 닿을 수 있는 트랙은 접근·정지 중에도 VO·TTC 에서 뺀다(게이트가
//      맡는다). 구간이 사라지면(모든 트랙이 못 닿는다) **커밋** — 그때 보였던 트랙 전부를 면제하고
//      구간 출구를 지날 때까지 그 트랙들에는 어떤 양보·hold·retreat·escape 도 개입하지 않는다.
//      커밋 뒤에 **나타난** 트랙은 다시 판정한다 (면제는 유지). 안전한 정지점에 못 미치면 지금 최대
//      감속으로 선다; **강제 커밋**(전속 통과)은 서는 자리가 노출 구간 안이고 몸으로도 진행선 위,
//      출구까지 commit_max_run 안일 때만이다 — 나란한 구간에서는 달려도 못 벗어난다.
//   4) 정지 중에는 v = 0, w = 0. 정지 자리를 어떤 트랙이 hold_threat_horizon 안에 닿을 수 있고
//      경로를 따라 물러나면 그 트랙에서 멀어질 때는 경로를 따라 후진한다(이탈 0).
// 상수는 시뮬레이션에서 0/610 을 낸 값 그대로다 (hold_back 1.2, delay 0.3, pos_err 0.25, vknown
// 0.3,
// turn_time 1.57, walk_accel 0.6, v_max 1.0). 되돌아옴 모델이 없으면 39/610 접촉 — 반환점(x=±6)이
// 노출 구간 끝에서 2.6 m 밖에 안 떨어져 "멀어지는" 보행자가 5 s 안에 돌아온다.
#ifndef AMR_NAVIGATION__CORE__CROSSING_GATE_HPP_
#define AMR_NAVIGATION__CORE__CROSSING_GATE_HPP_

#include <cstddef>
#include <limits>
#include <vector>

#include "amr_navigation/core/geometry.hpp"

namespace amr_navigation
{
namespace core
{

/// 게이트가 보는 트랙 (코스트맵 프레임). 확정 트랙 전부 — is_dynamic 이나 속도 문턱으로 거르지
/// 않는다: 반환점에 선 보행자가 "정적 물체" 로 사라지는 것이 08 의 "경고 없이 통로 안에서 발견"
/// 이었다.
struct GateTrack
{
  int id{-1};
  double x{0.0};
  double y{0.0};
  double radius{0.30};
  double ux{1.0};               ///< 진행 방향 단위벡터 (heading_known 일 때만 뜻이 있다)
  double uy{0.0};
  bool heading_known{false};    ///< 최근 line_window 동안 변위가 vknown·창 이상이면 부호를 믿는다
  double speed{0.0};            ///< [m/s] 진행 속력 추정 (되돌아옴 모델의 정지 시간에만 쓴다)
  double stationary_s{0.0};     ///< [s] 방향을 못 정한 채 서 있은 시간 (static_timeout 판정)
};

struct GateConfig
{
  bool enable{false};
  double robot_radius{0.361};       ///< [m] 외접원
  double pos_err{0.25};             ///< [m] 추적 위치 오차 (반경 합에 더한다)
  double heading_unc{0.1};          ///< [rad] 진행선 방향 불확실도 — 통로 반폭이 |α|·sin 만큼 큼
  double v_max{1.0};                ///< [m/s] 보행자 가정 최고 속도 (명세 상한 1.5 면 대기가 는다)
  double delay{0.3};                ///< [s] 관측 지연 (도달 시간에 더한다)
  double hold_back{1.2};            ///< [m] 노출 구간 시작 전 정지점 거리
  double turn_time{1.57};           ///< [s] 되돌아옴: 제자리 회전 시간 (180°/2 rad/s)
  double walk_accel{0.6};           ///< [m/s²] 보행 가감속
  double vknown{0.3};               ///< [m/s] 방향 부호를 믿는 변위 속력 하한 (트랙 공급자가 적용)
  double static_timeout{5.0};       ///< [s] 방향 없이 이보다 오래 선 트랙은 정적 (코스트맵 몫)
  double lookahead{8.0};            ///< [m] 노출 구간을 찾는 경로 길이
  double group_gap{1.5};            ///< [m] 이 간격 안의 노출 구간은 하나로 (사이에 설 자리 없음)
  double hold_tol{0.3};             ///< [m] 정지점 도달 허용 오차
  double stop_v{0.05};              ///< [m/s] "서 있다" 로 보는 속도
  double hold_threat_horizon{3.0};  ///< [s] 정지 자리 위협 판정 (방향 모르는 트랙 원판, 후진)
  /// [s] 정지점 배제: 멀어지는 트랙이 이 안에 되돌아올 수 있는 뒤쪽까지 그 진행선 통로로 본다
  double hold_exclusion_horizon{10.0};
  double exit_tol{0.2};             ///< [m] 커밋 해제: 출구를 이만큼 지나면 조우 종료
  /// [m] 강제 커밋(전속 통과)을 허용하는 출구까지 남은 길이 — 횡단은 2 m 안, 나란한 구간은 아니다
  double commit_max_run{2.0};
  bool retreat_enable{true};
  double retreat_speed{0.5};        ///< [m/s] 경로 후진 속도 (= 안전 게이트 retreat_max_speed)
  /// [m] 한 조우에서 물러나는 거리 상한 (2 m 로는 늦게 발견한 뒤 차선 띠 안에 남았다)
  double retreat_max{4.0};
  double retreat_gain{0.05};        ///< [m] 남은 예산만큼 물러난 자리가 위협에서 이만큼은 멀어야
  // 로봇 한계 (도달 시각·정지거리)
  double accel{1.0};
  double decel{1.0};
  double jerk{2.0};
  double latency{0.3};
  double robot_v_max{1.0};
};

enum class GatePhase
{
  kOpen = 0,        // 노출 구간 없음 — 게이트 개입 없음
  kApproach = 1,    // 노출 구간 앞: 정지점까지 속도 상한
  kHold = 2,        // 정지점에 서서 재판정 (v = 0, w = 0)
  kCommitted = 3,   // 진입 결정(또는 강제) — 출구까지 개입 없음
  kRetreat = 4      // 정지 자리가 위협받아 경로를 따라 후진
};

enum class GateReason
{
  kNone = 0,
  kClear = 1,             // 위협 없음
  kForward = 2,           // 방향을 아는 트랙이 앞으로 와서 닿는다
  kUnknownHeading = 3,    // 방향을 모르는 트랙이 원판 안에 둔다
  kReversal = 4,          // 멀어지는 트랙이 돌아서면 닿는다
  kForcedCommit = 5,      // 서는 자리가 노출 구간 안·진행선 위라 강제 커밋 (면제 없음)
  kHoldThreatened = 6,    // 정지 자리 자체가 위협받는다 (물러날 수 없음)
  kDecided = 7,           // 판정으로 커밋
  kTouching = 8           // 이미 반경 합 안
};

/// 주기 사이에 남기는 상태 (경로 창은 매 주기 바뀌므로 점은 코스트맵 프레임 좌표로 둔다)
struct GateState
{
  GatePhase phase{GatePhase::kOpen};
  bool has_hold{false};
  Point2D hold_point;
  bool has_exit{false};
  Point2D exit_point;
  bool forced{false};
  std::vector<int> exempt_ids;    ///< 판정한 트랙 (VO·TTC 면제) — 새로 나타난 트랙은 다시 판정한다
  bool has_retreat_origin{false};
  Point2D retreat_origin;
};

struct GateConflict
{
  std::size_t track{0};
  std::size_t first{0};       ///< 경로 정점 색인 (닿을 수 있는 첫 점)
  std::size_t last{0};
  GateReason reason{GateReason::kNone};
};

struct GateResult
{
  static constexpr double kInf = std::numeric_limits<double>::infinity();
  GatePhase phase{GatePhase::kOpen};
  GateReason reason{GateReason::kNone};
  double speed_limit{kInf};     ///< [m/s] 정지점까지의 속도 상한 (kApproach)
  bool hard_stop{false};        ///< v = 0, w = 0 (kHold)
  bool retreat{false};          ///< 경로를 따라 후진 (kRetreat)
  double retreat_speed{0.0};
  double s_hold{kInf};          ///< 정지점 호길이
  double s_in{kInf};            ///< 첫 노출 구간 시작·끝 호길이
  double s_out{kInf};
  double window_s{-1.0};        ///< [s] 지금 상태에서 구간 출구까지 주행 시간
  int n_threats{0};             ///< 닿을 수 있는 트랙 수
  std::vector<GateConflict> conflicts;
  /// 이번 주기에 VO·TTC 에서 뺄 트랙 (접근·정지: 닿을 수 있는 트랙, 판정 커밋: 판정한 트랙 전부)
  std::vector<int> exempt_ids;
  GateState next;               ///< 다음 주기에 넘길 상태
};

/// 뒤쪽 behind [m] 만큼 떨어진 점에 되돌아오는 최단 시간: 정지(v/a) + 회전 + 재가속.
double reversalReturnTime(double behind, double speed, const GateConfig & cfg);

/// 점 q 에서 기다리는 로봇이 트랙 t 에 노출되는가 (정지점 배제): 방향을 아는 트랙은 진행선 통로 안
/// (앞 전부 + hold_exclusion_horizon 안에 되돌아올 수 있는 뒤), 모르는 트랙은 hold_threat_horizon
/// 원판.
bool waitingSpotThreatened(const GateTrack & t, const Point2D & q, const GateConfig & cfg);

/// 트랙이 시각 tau [s] 까지 점 q 에 (몸이) 닿을 수 있는가. why 에 이유를 남긴다.
bool canReach(
  const GateTrack & t, const Point2D & q, double tau, const GateConfig & cfg,
  GateReason * why = nullptr);

/// 한 주기의 게이트 판정. path/cum_s 는 로봇 근처부터의 코스트맵 프레임 경로, s_robot 은 투영
/// 호길이,
/// v_robot 은 전진 속도(음수면 0), prev 는 직전 주기의 next.
GateResult evaluateGate(
  const std::vector<Pose2D> & path, const std::vector<double> & cum_s, const Pose2D & robot,
  double s_robot, double v_robot, const std::vector<GateTrack> & tracks, const GateConfig & cfg,
  const GateState & prev);

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__CROSSING_GATE_HPP_
