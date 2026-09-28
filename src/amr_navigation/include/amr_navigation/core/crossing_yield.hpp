// 동적 장애물의 예측 통로(corridor)와 가상 정지선 — ROS 비의존 (명세 4.7 동적 장애물 회피).
//
// 왜 필요한가 (리뷰 결함 08-1): VO/TTC 만으로는 "접촉 전에 설 수 있는 속도" 만 지키므로, 로봇이
// 횡단 작업자의 **통로 안에서** 멈춰 설 수 있다. 통합 시나리오 08 에서 27 건의 접촉이 모두 이렇게
// 났다 (접촉 순간 지면 진실 로봇 속도 ≤ 0.02 m/s — 스스로 부딪친 것이 아니라, 차선 안에 선 로봇에
// 비키지 않는 actor 가 걸어 들어왔다). 사람은 비키지 않는다고 보는 것이 맞으므로, 차가 횡단보도
// 앞에서 서듯 **통로 밖에** 서야 한다.
//
// 모델: 등속(CV) 원판 장애물 o(t) = o + u·t 가 지평 T_y 동안 쓸고 가는 영역
//   corridor = { q : |(q − o)·n̂| ≤ R_c,  −R_c ≤ (q − o)·û ≤ |u|·T_y + R_c },
//   û = u/|u|, n̂ = û 의 좌수직, R_c = r_robot + r_obs + corridor_margin.
// 기준 경로가 이 영역에 들어가는 구간 [s_in, s_out] 이 교차 구간이고, 그 구간을 장애물이 점유하는
// 시각은 α 좌표로 [t_in, t_out] = [(α_min − R_c)/|u|, (α_max + R_c)/|u|] 이다.
//
// 판정 (로봇 호길이 s_r, 속도 v):
//   t_out ≤ 0                              장애물이 교차 구간을 이미 지났다 → 통과 (kClear)
//   t_robot_out + margin < t_in            로봇이 먼저 빠져나간다 → 통과 (kClear)
//   t_out + margin < t_robot_in            장애물이 먼저 빠져나간다 → 통과 (kClear)
//   그 밖 + 로봇이 이미 구간 안       이미 들어섰다 → 멈추면 차선 안이므로 그대로 통과 (kCommitted)
//   그 밖                            가상 정지선 d = s_in − s_r − stop_margin 앞에 선다 (kYield),
//                                   속도 상한 = d_stop⁻¹(d) (SpeedProfile::maxSpeedForStop)
// kYield 의 속도 상한은 DWA 의 외부 속도 제한과 같은 자리에 들어간다 (동적 창 상한 + v_des) — 저크·
// 지연을 넣은 정지거리의 역함수라 정지선에서 정확히 0 이 된다. 장애물이 지나가면 t_out ≤ 0 이 되어
// 상한이 풀리고, 로봇은 원래 경로 위에서 그대로 재출발한다 (이탈 0).
#ifndef AMR_NAVIGATION__CORE__CROSSING_YIELD_HPP_
#define AMR_NAVIGATION__CORE__CROSSING_YIELD_HPP_

#include <cstddef>
#include <limits>
#include <vector>

#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/velocity_obstacle.hpp"

namespace amr_navigation
{
namespace core
{

struct YieldConfig
{
  bool enable{true};
  double robot_radius{0.361};       // [m] 로봇 덮개 원 (외접 반경)
  double corridor_margin{0.50};     // [m] 통로 반폭 여유 (critical zone 거리와 같게)
  double stop_margin{0.25};         // [m] 통로 입구 앞 정지선 여유 (경로 호길이)
  double clear_margin{1.0};         // [s] 먼저 빠져나간다고 볼 시간 여유
  // [s] 장애물 예측 지평. 통로 길이 = max(|u|, nominal_speed)·horizon
  double horizon{8.0};
  /// [m/s] 통로 길이의 속도 바닥. 반환점에서 돌아서는 작업자는 |u| 가 0.01~0.08 까지 떨어져
  /// 통로가 사라졌다가 가속하면 폭발한다 — 그때는 이미 로봇이 안에 있다. 명세 4.1 의 동적
  /// 장애물 속도 범위(0.3~1.5 m/s) 중간값.
  double nominal_speed{1.0};
  /// [m] hold 가 몸 원통 경계에서 이만큼 **더 앞에** 선다. 경계에 딱 맞춰 서면 추적 오차
  /// (실측 0.1~0.25 m) 하나로 접촉이 된다 — 08 실측에서 hold 정지 위치와 접촉 위치의 간격이
  /// 14 cm 였다. 정지는 이탈 0 이라 여유를 두는 비용이 시행 시간뿐이다.
  double hold_standoff{0.35};
  double lookahead{4.0};            // [m] 경로에서 교차 구간을 찾는 최대 거리
  double max_zone{6.0};             // [m] 교차 구간 길이 상한 (넘으면 나란한 주행 — 정지선 없음)
  double min_speed{0.2};            // [m/s] 이보다 느린 트랙은 정적 (코스트맵이 처리)
  // 로봇 한계 (도달 시각 예측·정지거리)
  double accel{1.0};
  double decel{1.0};
  double jerk{2.0};
  double latency{0.3};              // [s] 명령 → 실속도 지연 (approach_latency 와 같게)
  double v_max{1.0};                // [m/s]
  // --- 통로 안(kCommitted) 진입 시점의 결정 go / hold / retreat (아래 YieldDecision) ---
  // 위험 구간 = 몸 원통 |β| < r_robot + r_obs + body_margin. R_c 통로(반폭 + corridor_margin)는
  // "언제 양보를 시작할지" 의 여유이고, 접촉이 실제로 나는 곳은 몸 원통이다. 두 반폭 사이(08 기하:
  // 경로로 1.2~1.5 m)에서 무엇을 할지가 이 결정이다.
  bool decision_enable{true};
  double body_margin{0.15};         // [m] 몸 원통 여유 (추적 가로 오차·발자국 방향 여유)
  double go_margin{0.0};            // [s] go 의 시간 여유 하한: 로봇이 위험 구간을 다 지날 때
                                    //     작업자 앞이 최소 이만큼 뒤 (0 = 앞서기만 하면 된다)
  double go_clearance{0.40};        // [m] go 진입: 스칠 때 몸 사이 **예측 여유** 하한. 여유는
                                    //     교차각에 달렸다 — 출구 뒤 로봇은 v·sin θ 로 축에서
                                    //     멀어지고 작업자는 (여유·|u| + r_body)/(닫힘 속도) 뒤에
                                    //     온다: 26.6° 에서 0.15 + 0.236·(여유 + 0.661), 90° 에서
                                    //     0.81 + 여유
  double go_hold_clearance{0.30};   // [m] go 유지 하한 (이 아래로 줄면 go 를 버린다 — 이력)
  double go_speed_floor{1.0};       // [m/s] go 의 "로봇이 먼저" 여유를 잴 때 작업자 속도 하한. 차선
                                    //     끝에서 돌아선 직후 평활 속도(τ 0.5 s)는 0.3 → 1.0 으로
                                    //     오르는 중이라, 그때 잰 여유는 실제보다 최대 2 배 크다.
                                    //     "작업자가 먼저" 쪽은 느린 추정이 이미 보수적이라 그대로
  double hold_slack{0.3};           // [m] hold 가능 판정 여유: d_stop ≤ 구간 입구까지 거리 + slack
                                    //     (제동 중 바깥으로 조향하면 안쪽 표류가 그만큼 준다)
  double decision_min_sin{0.34};    // 결정을 내리는 교차각 하한 sin θ (20°). 더 얕으면(정면·같은
                                    //     차선 추종) 몸 원통이 경로를 따라 길게 이어져 go/hold/
                                    //     retreat 가 뜻이 없다 — 예전 그대로 VO/TTC·원통 안 탈출이
                                    //     맡는다
};

enum class YieldState
{
  kClear,        // 양보 불필요 (교차 없음 / 먼저 빠져나간다 / 장애물이 지나갔다)
  kYield,        // 통로 밖 가상 정지선 앞에 선다
  kCommitted     // 이미 통로 안이거나 정지선 앞에 설 수 없다 — 멈추지 말고 빠져나간다
};

/// kCommitted 에서 무엇을 할지 — 통합 08 접촉 4건(j8c t6·t21, reg1 t12·t13)의 공통 기제는
/// "정지선 없이 통로 안에 들어선 채 점진 감속 + 선회" 였다. 넷 다 진입 순간에 옳은 행동이 따로
/// 있었다: 직진(여유 있음), 즉시 정지(몸 원통 밖에 설 수 있음), 축에서 멀어지기(그 밖). 감속하며
/// 선회하는 것은 가로 권한(∝ v)을 잃고 작업자 앞을 가로지르며 게이트·랙 포켓에 걸린다.
enum class YieldDecision
{
  kNone = 0,     // 결정 없음 (kCommitted 가 아니거나, 작업자가 이미 지났거나, 구간이 창 밖)
  kGo = 1,       // 감속 없이 지나간다 — 이 장애물의 TTC₀ 벌점을 끈다 (VO 는 유지)
  kHold = 2,     // 몸 원통 입구 앞에 선다 (속도 상한 = d_stop⁻¹(입구까지)) + 바깥으로 조향
  kRetreat = 3   // 축에서 멀어지는 쪽으로만 비켜선다 (가로지르지 않는다), 속도는 유지
};

struct YieldResult
{
  YieldState state{YieldState::kClear};
  static constexpr double kInf = std::numeric_limits<double>::infinity();
  double speed_limit{kInf};      // [m/s] 정지선까지의 속도 상한
  double stop_distance{kInf};    // [m] 로봇 → 정지선
  double zone_entry{kInf};       // [m] 로봇 → 교차 구간 입구
  double zone_exit{kInf};        // [m] 로봇 → 교차 구간 출구
  double obstacle_in{kInf};      // [s] 장애물 점유 시작
  double obstacle_out{-kInf};    // [s] 장애물 점유 끝
  std::ptrdiff_t obstacle{-1};   // 상한을 정한 장애물의 색인 (없으면 −1). kCommitted 면 축이 가장
                                 // 가까운(|β| 최소) 장애물 — 근거리 중복 트랙이 있어도 진짜 몸 쪽.
  // --- kCommitted 결정 (YieldDecision) ---
  YieldDecision decision{YieldDecision::kNone};
  double margin{-kInf};          // [s] go 여유 = max(로봇이 먼저 빠져나가는 여유, 작업자가 먼저
                                 //     지나는 여유)
  double clearance{-kInf};       // [m] 그 여유로 지나갈 때 스칠 순간 몸 사이 예측 여유 (go 판정
                                 //     기준)
  double body_beta{0.0};         // [m] 결정 장애물 축까지 로봇의 부호 있는 거리 (좌 +)
  double body_entry{kInf};       // [m] 로봇 → 몸 원통 입구 (이미 안이면 0, 창 밖이면 +inf)
  std::vector<bool> decided;     // 장애물별 "결정 층이 맡는다"(go/hold/retreat 를 받은 kCommitted
                                 // 장애물, 중복 트랙 포함) — DWA 는 이들의 TTC₀ 벌점을 끈다.
                                 // obstacles 와 같은 길이
};

/// 거리 d 를 속도 v0 에서 가속 a 로 v_max 까지 올려 가며 달리는 데 걸리는 시간 [s].
double travelTime(double d, double v0, double a, double v_max);

/// 기준 경로 위 교차 구간과 가상 정지선. path 는 로봇 근처부터의 코스트맵 프레임 경로,
/// cum_s 는 그 누적 호길이, s_robot 은 로봇의 경로 투영 호길이, v_robot 은 현재 전진 속도.
/// 장애물 여러 개면 속도 상한이 가장 낮은(가장 이른 정지선) 것이 결과가 된다.
/// 다만 이미 들어선 통로가 있으면 그 kCommitted 가 정지선보다 우선한다
/// (남의 차선 안에 서지 않는다 — 장애물 순서와 무관한 결론).
/// prev_decision: 직전 주기의 결정 (go 이력: 진입 go_margin, 유지 go_hold_margin).
YieldResult evaluateYield(
  const std::vector<Pose2D> & path, const std::vector<double> & cum_s, const Pose2D & robot,
  double s_robot, double v_robot, const std::vector<DynamicObstacle> & obstacles,
  const YieldConfig & config, YieldDecision prev_decision = YieldDecision::kNone);

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__CROSSING_YIELD_HPP_
