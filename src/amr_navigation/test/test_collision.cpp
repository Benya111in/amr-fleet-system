// 풋프린트 충돌 검사와 Velocity Obstacle(VO)·예측 충돌 시각 단위 테스트.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "amr_navigation/core/footprint.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/velocity_obstacle.hpp"

using amr_navigation::core::DynamicObstacle;
using amr_navigation::core::FootprintChecker;
using amr_navigation::core::OwnedGrid;
using amr_navigation::core::Point2D;
using amr_navigation::core::Pose2D;
using amr_navigation::core::chordVelocity;
using amr_navigation::core::firstContactTime;
using amr_navigation::core::inVelocityObstacle;
using amr_navigation::core::inflate;
using amr_navigation::core::inflationCost;
using amr_navigation::core::integrateArc;
using amr_navigation::core::kInscribedInflated;
using amr_navigation::core::kLethalObstacle;
using amr_navigation::core::kNoInformation;
using amr_navigation::core::kPi;

TEST(Footprint, RectangleRadii)
{
  const auto fp = FootprintChecker::rectangle(0.60, 0.40);
  ASSERT_EQ(fp.size(), 4U);
  EXPECT_NEAR(FootprintChecker::inscribedRadius(fp), 0.20, 1e-12);
  EXPECT_NEAR(FootprintChecker::circumscribedRadius(fp), std::hypot(0.30, 0.20), 1e-12);
  EXPECT_NEAR(FootprintChecker::inscribedRadius({}), 0.0, 1e-12);
}

TEST(Footprint, ThreeStageCheck)
{
  // 치명 셀 하나 + Nav2 식 팽창
  OwnedGrid g(100, 100, 0.05);
  g.at(50, 50) = kLethalObstacle;
  inflate(g, 0.2, 1.0, 3.0);
  const auto fp = FootprintChecker::rectangle(0.60, 0.40);
  const uint8_t c_circ = inflationCost(FootprintChecker::circumscribedRadius(fp), 0.2, 3.0);
  const FootprintChecker chk(g.view(), fp, c_circ);
  const double ox = 50.5 * 0.05;   // 장애물 셀 중심
  const double oy = 50.5 * 0.05;
  // (1) 중심이 내접원 안 → 충돌
  EXPECT_TRUE(chk.collides({ox + 0.1, oy, 0.0}));
  // (2) 외접원보다 멀리 → 자유, 비용 = 중심 비용
  const Pose2D far{ox + 0.8, oy, 0.0};
  EXPECT_FALSE(chk.collides(far));
  EXPECT_DOUBLE_EQ(chk.cost(far), static_cast<double>(chk.centerCost(far.x, far.y)));
  // (3) 중간: 장애물이 긴 변(앞쪽 모서리) 안에 들어오는 자세 → 외곽선 검사로 충돌
  //     로봇 중심은 장애물에서 y 로 0.29 (내접 0.20 밖, 중심 비용 192 ≥ c_circ 155), 방향 0 →
  //     변 y = +0.2 는 장애물보다 0.09 아래 → 외곽선이 닿지 않는다 → 자유
  EXPECT_FALSE(chk.collides({ox, oy - 0.29, 0.0}));
  //     90° 회전하면 긴 변(반길이 0.3)이 y 방향 → 앞 변(y = +0.01)이 장애물 셀을 지난다 → 충돌
  EXPECT_TRUE(chk.collides({ox, oy - 0.29, kPi / 2.0}));
  // 외접 비용을 모르면(0) 항상 외곽선 검사 — 결과는 같다
  const FootprintChecker slow(g.view(), fp, 0);
  EXPECT_TRUE(slow.collides({ox, oy - 0.29, kPi / 2.0}));
  EXPECT_FALSE(slow.collides({ox, oy - 0.29, 0.0}));
  EXPECT_GE(slow.cost({ox, oy - 0.29, 0.0}), chk.cost({ox, oy - 0.29, 0.0}) - 1e-9);
}

TEST(Footprint, UnknownAndOutOfMap)
{
  OwnedGrid g(40, 40, 0.05, kNoInformation);
  const auto fp = FootprintChecker::rectangle(0.60, 0.40);
  const FootprintChecker strict(g.view(), fp, 100, false);
  const FootprintChecker lenient(g.view(), fp, 100, true);
  EXPECT_TRUE(strict.collides({1.0, 1.0, 0.0}));
  EXPECT_FALSE(lenient.collides({1.0, 1.0, 0.0}));
  // 지도 밖으로 나가는 풋프린트는 충돌(범위 밖 = 치명)
  EXPECT_TRUE(lenient.collides({0.1, 1.0, 0.0}));
  // 꼭짓점이 1개인 퇴화 풋프린트 → 중심 비용만
  const FootprintChecker point(g.view(), {{0.0, 0.0}}, 255, true);
  EXPECT_DOUBLE_EQ(point.cost({1.0, 1.0, 0.0}), 0.0);
}

TEST(VelocityObstacle, ChordVelocity)
{
  // ω = 0: 방향 θ, 크기 v
  const Point2D a = chordVelocity(0.3, 1.0, 0.0, 2.0);
  EXPECT_NEAR(a.x, std::cos(0.3), 1e-12);
  EXPECT_NEAR(a.y, std::sin(0.3), 1e-12);
  // 현 속도 × τ == 원호 끝점 변위 (정의)
  const double v = 1.2;
  const double w = 0.8;
  const double tau = 2.0;
  const Point2D c = chordVelocity(0.1, v, w, tau);
  const Pose2D end = integrateArc({0.0, 0.0, 0.1}, v, w, tau);
  EXPECT_NEAR(c.x * tau, end.x, 1e-12);
  EXPECT_NEAR(c.y * tau, end.y, 1e-12);
}

TEST(VelocityObstacle, ConeCases)
{
  const double R = 0.8;
  const double tau = 2.0;
  const Point2D p{3.0, 0.0};   // 장애물이 정면 3 m
  // 정면 접근: 상대 속도 (1.5, 0) → t* = 2 에서 거리 0 → VO 안
  EXPECT_TRUE(inVelocityObstacle(p, {1.5, 0.0}, R, tau));
  // 같은 방향이지만 느림: τ 안에 3 − 2·1.0 = 1.0 > R → 절단 원뿔 밖 (시간 지평)
  EXPECT_FALSE(inVelocityObstacle(p, {1.0, 0.0}, R, tau));
  // 멀어짐 → 밖
  EXPECT_FALSE(inVelocityObstacle(p, {-1.0, 0.0}, R, tau));
  // 정지 상대 운동 → 밖
  EXPECT_FALSE(inVelocityObstacle(p, {0.0, 0.0}, R, tau));
  // 원뿔 반각 asin(R/|p|) 경계: 각도가 반각보다 조금 작으면 안, 크면 밖 (속도가 충분히 클 때)
  const double half = std::asin(R / 3.0);
  const double s = 5.0;
  EXPECT_TRUE(
    inVelocityObstacle(
      p, {s * std::cos(half - 0.02), s * std::sin(half - 0.02)}, R,
      tau));
  EXPECT_FALSE(
    inVelocityObstacle(p, {s * std::cos(half + 0.02), s * std::sin(half + 0.02)}, R, tau));
  // 이미 합성 반경 안: 접근하면 VO, 멀어지면 허용
  const Point2D close{0.5, 0.0};
  EXPECT_TRUE(inVelocityObstacle(close, {0.1, 0.0}, R, tau));
  EXPECT_FALSE(inVelocityObstacle(close, {-0.1, 0.3}, R, tau));
}

TEST(VelocityObstacle, FirstContactTimeHeadOn)
{
  // 로봇 1 m/s 로 +x, 장애물 (5, 0) 에서 −1 m/s → 상대 접근
  //   2 m/s, 반경 R = 1 → 접촉 (5−1)/2 = 2.0 s
  std::vector<Pose2D> traj;
  const double dt = 0.1;
  for (int k = 0; k <= 50; ++k) {
    traj.push_back({k * dt * 1.0, 0.0, 0.0});
  }
  const DynamicObstacle obs{5.0, 0.0, -1.0, 0.0, 0.25};
  EXPECT_NEAR(firstContactTime(traj, dt, obs, 1.0), 2.0, 1e-9);
  // 옆으로 2 m 비껴 지나감 → 접촉 없음
  const DynamicObstacle side{5.0, 2.0, -1.0, 0.0, 0.25};
  EXPECT_TRUE(std::isinf(firstContactTime(traj, dt, side, 1.0)));
  // 이미 겹침 → 0
  const DynamicObstacle on{0.5, 0.0, 0.0, 0.0, 0.25};
  EXPECT_DOUBLE_EQ(firstContactTime(traj, dt, on, 1.0), 0.0);
  EXPECT_TRUE(std::isinf(firstContactTime({}, dt, obs, 1.0)));
  EXPECT_NEAR(obs.speed(), 1.0, 1e-12);
  EXPECT_NEAR(obs.predict(2.0).x, 3.0, 1e-12);
}
