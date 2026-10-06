// 계약 기하 — rl/contract/amr_rl_contract/geometry.py 와 observation.py 의 선별 함수 사본.
//
// 원호 적분·VO 시각·chord 속도는 amr_navigation_core 의 원본 함수를 그대로 쓴다
// (amr_navigation/core/{geometry,velocity_obstacle}.hpp). 여기에는 계약에만 있는 정의
// (전역 최근접 경로 투영·동률 규칙, 스캔 끝점, 풋프린트 여유거리, 트랙 선별)만 둔다.
#ifndef AMR_RL_CONTROLLER__CORE__GEOMETRY_HPP_
#define AMR_RL_CONTROLLER__CORE__GEOMETRY_HPP_

#include <cstdint>
#include <vector>

#include "amr_rl_controller/core/state.hpp"

namespace amr_rl_controller
{

/// 각을 [−π, π) 로 감는다 (계약 wrap_angle).
double wrapAngle(double a);

/// 경로 점과 누적 호길이 (geometry.path_arrays).
class ContractPath
{
public:
  explicit ContractPath(const std::vector<Point2D> & pts);
  const std::vector<Point2D> & points() const {return pts_;}
  const std::vector<double> & cum() const {return cum_;}
  double length() const {return cum_.empty() ? 0.0 : cum_.back();}

  struct Projection
  {
    double s{0.0};          ///< 투영 호길이
    double dist{0.0};       ///< 최근접 거리 (부호 없음)
    double signed_e{0.0};   ///< 진행 방향 왼쪽 +
    double seg_yaw{0.0};    ///< 투영 선분 방향
  };
  /// 전역 최근접점 투영, 동률(1e-12 안)이면 작은 호길이 (geometry.project_onto_path).
  Projection project(const Point2D & p) const;
  /// 호길이 s 의 점 (양 끝에서 자른다, geometry.interpolate_at).
  Point2D interpolate(double s) const;

private:
  std::vector<Point2D> pts_;
  std::vector<double> cum_;
};

/// 지도 점 → 로봇 좌표.
Point2D toRobot(const Pose2D & pose, const Point2D & p);
/// 지도 벡터 → 로봇 축 (이동 없음).
Point2D rotateToRobot(double theta, const Point2D & v);
/// 로봇 좌표 점에서 풋프린트 사각형까지 거리 (안쪽 0).
double footprintClearance(const Point2D & p);
/// 빔 i 의 각 = angle_min + i·inc.
double beamAngle(int i);
/// 유효 빔 = 유한 && range_min ≤ r ≤ range_max.
bool beamValid(double r);
/// 유효 빔 끝점 (로봇 좌표). 빔 수가 720 이 아니면 std::invalid_argument.
std::vector<Point2D> scanPoints(const std::vector<double> & ranges);
/// 로봇 좌표 점들 중 하나라도 로봇 좌표계 자세 pose 의 풋프린트(+pad) 안에 있는가.
bool pointsInFootprint(const std::vector<Point2D> & pts, const Pose2D & pose);

/// 관측·VO 에 쓰는 선별 트랙 (observation.select_tracks 의 행).
struct SelectedTrack
{
  double dist{0.0};
  int64_t id{0};
  Point2D p;      ///< 로봇 좌표 위치 (나이만큼 외삽)
  Point2D v;      ///< 로봇 축 속도 (지도 속도를 회전)
  bool is_dynamic{false};
  double speed{0.0};
};
/// is_dynamic 이거나 속력 ≥ 0.5, 나이 외삽, 거리 ≤ max_range, (거리, id) 오름차순.
std::vector<SelectedTrack> selectTracks(const ContractState & s, double max_range);
/// 관측용 트랙 TTC (전방 직진 v_meas, 장애물 등속, 반경 외접 + 0.30, 지평 5 s), 없으면 +inf.
double trackTtc(const Point2D & p, const Point2D & v_obs_robot, double v_meas);

}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__CORE__GEOMETRY_HPP_
