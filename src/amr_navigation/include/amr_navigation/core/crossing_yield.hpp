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
  double horizon{8.0};              // [s] 장애물 예측 지평 (통로 길이 = |u|·horizon)
  double lookahead{4.0};            // [m] 경로에서 교차 구간을 찾는 최대 거리
  double max_zone{6.0};             // [m] 교차 구간 길이 상한 (넘으면 나란한 주행 — 정지선 없음)
  double min_speed{0.2};            // [m/s] 이보다 느린 트랙은 정적 (코스트맵이 처리)
  // 로봇 한계 (도달 시각 예측·정지거리)
  double accel{1.0};
  double decel{1.0};
  double jerk{2.0};
  double latency{0.3};              // [s] 명령 → 실속도 지연 (approach_latency 와 같게)
  double v_max{1.0};                // [m/s]
};

enum class YieldState
{
  kClear,        // 양보 불필요 (교차 없음 / 먼저 빠져나간다 / 장애물이 지나갔다)
  kYield,        // 통로 밖 가상 정지선 앞에 선다
  kCommitted     // 이미 통로 안이거나 정지선 앞에 설 수 없다 — 멈추지 말고 빠져나간다
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
  std::ptrdiff_t obstacle{-1};   // 상한을 정한 장애물의 색인 (없으면 −1)
};

/// 거리 d 를 속도 v0 에서 가속 a 로 v_max 까지 올려 가며 달리는 데 걸리는 시간 [s].
double travelTime(double d, double v0, double a, double v_max);

/// 기준 경로 위 교차 구간과 가상 정지선. path 는 로봇 근처부터의 코스트맵 프레임 경로,
/// cum_s 는 그 누적 호길이, s_robot 은 로봇의 경로 투영 호길이, v_robot 은 현재 전진 속도.
/// 장애물 여러 개면 속도 상한이 가장 낮은(가장 이른 정지선) 것이 결과가 된다.
/// 다만 이미 들어선 통로가 있으면 그 kCommitted 가 정지선보다 우선한다
/// (남의 차선 안에 서지 않는다 — 장애물 순서와 무관한 결론).
YieldResult evaluateYield(
  const std::vector<Pose2D> & path, const std::vector<double> & cum_s, const Pose2D & robot,
  double s_robot, double v_robot, const std::vector<DynamicObstacle> & obstacles,
  const YieldConfig & config);

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__CROSSING_YIELD_HPP_
