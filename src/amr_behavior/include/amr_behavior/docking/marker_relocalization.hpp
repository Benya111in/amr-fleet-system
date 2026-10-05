// 마커 관측 → 로봇 자세 역산 (ROS 비의존). 명세 4.3 "납치 시 스스로 위치 복구" 의 보조 경로.
//
// 왜 필요한가 (통합 시나리오 11 실측): 로봇을 형태가 같은 평행 통로로 6 m 옮기면
// 스캔이 옛 추정 자세에서도 그대로 잘 맞아 (aliasing) kidnap_monitor 의 어떤 기준에도
// 걸리지 않는다 — 공분산·점프·inlier 모두 정상.
// 로봇은 6 m 틀어진 채 주행했고, 도크 앞에서 다른 도크의 마커를 보고 도킹을 거부해 작업이 실패했다.
// ArUco 마커는 지도에서 **유일한** 랜드마크라 이 애매함을 끊을 수 있다: 등록된 마커를
// 하나 보면 그 관측만으로 로봇 자세가 정해진다 (현장에서 reflector·마커 기반
// 재위치추정을 쓰는 이유와 같다).
//
// 규약 (계약 C3): 마커 모델 프레임은 +x = 판 바깥 법선, +z = 위. 관측
// MarkerObservation 은 base 프레임에서 본 마커 위치 (x, y) 와 그 법선 방향
// normal_yaw 다 (docking_controller.hpp).
//   T_map_base = T_map_marker ∘ (T_base_marker)⁻¹
// 위치 추정이 틀어져 있어도 base→마커 관측은 로봇 내부 TF 와 카메라만 쓰므로 영향을 받지 않는다.
#ifndef AMR_BEHAVIOR__DOCKING__MARKER_RELOCALIZATION_HPP_
#define AMR_BEHAVIOR__DOCKING__MARKER_RELOCALIZATION_HPP_

#include <cmath>

#include "amr_behavior/docking/docking_controller.hpp"

namespace amr_behavior
{
namespace docking
{

/// 평면 자세 (map 프레임).
struct MapPose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// [-π, π) 로 감는다.
inline double wrapAngle(double a)
{
  while (a >= M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

/// 마커 관측(base 프레임)과 그 마커의 지도 자세로 로봇(base) 의 지도 자세를 역산한다.
inline MapPose2D impliedRobotPose(const MarkerObservation & obs, const MapPose2D & marker_map)
{
  // T_base_marker = (obs.x, obs.y, obs.normal_yaw) → 역변환 T_marker_base
  const double c = std::cos(obs.normal_yaw);
  const double s = std::sin(obs.normal_yaw);
  const double tx = -(obs.x * c + obs.y * s);
  const double ty = obs.x * s - obs.y * c;
  const double mc = std::cos(marker_map.yaw);
  const double ms = std::sin(marker_map.yaw);
  return MapPose2D{
    marker_map.x + mc * tx - ms * ty,
    marker_map.y + ms * tx + mc * ty,
    wrapAngle(marker_map.yaw - obs.normal_yaw)};
}

/// 두 자세의 평면 거리 [m].
inline double poseDistance(const MapPose2D & a, const MapPose2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace docking
}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__DOCKING__MARKER_RELOCALIZATION_HPP_
