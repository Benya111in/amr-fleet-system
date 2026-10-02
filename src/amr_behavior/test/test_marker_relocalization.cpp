// 마커 관측 → 로봇 자세 역산 (계약 C3 규약, 통합 시나리오 11 의 aliasing 복구 경로).
#include <gtest/gtest.h>

#include <cmath>

#include "amr_behavior/docking/marker_relocalization.hpp"

using amr_behavior::docking::MapPose2D;
using amr_behavior::docking::MarkerObservation;
using amr_behavior::docking::impliedRobotPose;
using amr_behavior::docking::poseDistance;
using amr_behavior::docking::wrapAngle;

namespace
{
/// 지도 자세 robot 에서 지도 자세 marker 를 봤을 때의 관측 (base 프레임) — 역산의 정방향.
MarkerObservation observe(const MapPose2D & robot, const MapPose2D & marker)
{
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);
  const double dx = marker.x - robot.x;
  const double dy = marker.y - robot.y;
  return MarkerObservation{c * dx + s * dy, -s * dx + c * dy, wrapAngle(marker.yaw - robot.yaw)};
}
}  // namespace

TEST(MarkerRelocalization, RecoversPoseFromItsOwnObservation)
{
  // dock_2 마커 (−29.99, 13.0, 0°): 판 바깥 법선이 +x 라 로봇은 그 앞(+x 쪽)에서 마커를 마주 본다
  const MapPose2D marker{-29.99, 13.0, 0.0};
  for (const MapPose2D & robot :
    {MapPose2D{-27.78, 13.0, M_PI}, MapPose2D{-28.5, 13.4, 2.9}, MapPose2D{-27.0, 12.2, -2.6}})
  {
    const MapPose2D back = impliedRobotPose(observe(robot, marker), marker);
    EXPECT_NEAR(back.x, robot.x, 1e-9);
    EXPECT_NEAR(back.y, robot.y, 1e-9);
    EXPECT_NEAR(wrapAngle(back.yaw - robot.yaw), 0.0, 1e-9);
  }
}

TEST(MarkerRelocalization, WrongAisleGivesTheDistanceBetweenDocks)
{
  // 시나리오 11 상황: 로봇은 dock_2 앞(y = 13) 이라고 믿는데 실제로는 dock_1 앞(y = 17) 에 있다.
  // 그 자리에서 본 것은 dock_1 마커(id 0) 이므로, 그 마커의 지도 자세로 역산하면
  // 진짜 자세가 나온다.
  const MapPose2D dock1_marker{-29.99, 17.0, 0.0};
  const MapPose2D truth{-27.78, 17.0, M_PI};
  const MapPose2D believed{-27.78, 13.0, M_PI};
  const MapPose2D fix = impliedRobotPose(observe(truth, dock1_marker), dock1_marker);
  EXPECT_NEAR(poseDistance(fix, truth), 0.0, 1e-9);
  EXPECT_NEAR(poseDistance(fix, believed), 4.0, 1e-9);    // 도크 간격만큼 어긋나 있었다
}

TEST(MarkerRelocalization, HandlesRotatedMarkers)
{
  // 충전소 마커는 +y 를 향한다 (gen_warehouse_world.py: yaw = π/2)
  const MapPose2D marker{-26.0, -16.64, M_PI / 2.0};
  const MapPose2D robot{-26.0, -15.9, -M_PI / 2.0};
  const MarkerObservation obs = observe(robot, marker);
  EXPECT_NEAR(obs.x, 0.74, 1e-9);                          // 정면 0.74 m
  EXPECT_NEAR(obs.y, 0.0, 1e-9);
  const MapPose2D back = impliedRobotPose(obs, marker);
  EXPECT_NEAR(back.x, robot.x, 1e-9);
  EXPECT_NEAR(back.y, robot.y, 1e-9);
  EXPECT_NEAR(wrapAngle(back.yaw - robot.yaw), 0.0, 1e-9);
}

TEST(MarkerRelocalization, WrapAngleKeepsRange)
{
  EXPECT_NEAR(wrapAngle(3.0 * M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(-3.0 * M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(0.5), 0.5, 1e-12);
}
