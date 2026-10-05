// 2D 기하 유틸리티 — ROS 비의존.
//
// 경로(폴리라인)의 길이·투영·호길이 보간·이산 곡률과, 차동구동 원호 적분(정확해)을 둔다.
// 모든 각도는 [rad], 거리는 [m]. 부호 규약: CTE 는 경로 진행 방향 기준 좌측 +
//   (docs/algorithms/path_tracking.md).
#ifndef AMR_NAVIGATION__CORE__GEOMETRY_HPP_
#define AMR_NAVIGATION__CORE__GEOMETRY_HPP_

#include <cstddef>
#include <vector>

namespace amr_navigation
{
namespace core
{

constexpr double kPi = 3.14159265358979323846;

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

/// (-π, π] 로 감는다.
double wrapAngle(double a);

double distance(const Point2D & a, const Point2D & b);
double distance(const Pose2D & a, const Pose2D & b);

/// 폴리라인 총 길이.
double pathLength(const std::vector<Pose2D> & path);

/// 각 정점까지의 누적 호길이 s_i (s_0 = 0).
std::vector<double> cumulativeLength(const std::vector<Pose2D> & path);

/// 폴리라인 위 최근접점 투영 결과.
struct Projection
{
  std::size_t segment{0};   // 선분 [segment, segment+1]
  double t{0.0};            // 선분 내 매개변수 [0,1]
  double s{0.0};            // 누적 호길이 [m]
  Point2D point;            // 투영점
  double cte{0.0};          // 부호 있는 수직 거리 (좌측 +) [m]
  double heading{0.0};      // 투영 선분의 방향 [rad]
};

/// [begin, end) 정점 창 안에서 최근접 선분을 찾는다 (end = 0 이면 끝까지).
/// cum_s 가 비어 있으면 s 는 창 시작 기준 상대값이다.
Projection projectOntoPath(
  const std::vector<Pose2D> & path, const Point2D & p, std::size_t begin = 0, std::size_t end = 0,
  const std::vector<double> * cum_s = nullptr);

/// 호길이 s 에서의 경로 위 점과 접선 방향 (s 는 [0, 전체 길이] 로 포화).
Pose2D interpolateAt(
  const std::vector<Pose2D> & path, const std::vector<double> & cum_s, double s);

/// 정점별 이산 곡률 κ_i = Δθ_i / ((Δs_i + Δs_{i+1})/2) [1/m] (부호 = 좌회전 +). 양 끝점은 0.
/// window_m > 0 이면 정점 i 기준 앞뒤 window_m 떨어진 점으로 Δθ 를 잰다(양자화 잡음 억제).
std::vector<double> discreteCurvature(const std::vector<Pose2D> & path, double window_m = 0.0);

/// 차동구동 원호 정확 적분: (v, ω) 를 dt 동안 유지했을 때의 다음 자세.
/// |ω| < 1e-9 이면 직선. 식: x' = x + v/ω (sin(θ+ωdt) − sinθ), y' = y − v/ω (cos(θ+ωdt) − cosθ).
Pose2D integrateArc(const Pose2D & p, double v, double w, double dt);

/// 전역 → 로컬(로봇) 좌표 변환: 로봇 자세 기준 점의 좌표.
Point2D toRobotFrame(const Pose2D & robot, const Point2D & world);

/// 강체 변환 합성: T (프레임 A 에서 본 프레임 B 의 자세) 로 B 좌표의 자세 p 를 A 좌표로.
Pose2D compose(const Pose2D & T, const Pose2D & p);
/// 역변환.
Pose2D inverse(const Pose2D & T);

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__GEOMETRY_HPP_
