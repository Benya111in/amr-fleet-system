// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// TTC 단위 테스트: 정면/횡단/이탈/정지/경로 추종/불확실성 팽창을 해석해와 비교.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "amr_perception/ttc.hpp"

using amr_perception::Mat2;
using amr_perception::ObstacleMotion;
using amr_perception::RobotMotionModel;
using amr_perception::TtcParams;
using amr_perception::Vec2;

namespace
{
TtcParams exactParams()
{
  TtcParams p;
  p.k_sigma = 0.0;  // 해석해 비교: 불확실성 팽창 없음
  return p;
}

ObstacleMotion obstacle(const Vec2 & p, const Vec2 & v, double r = 0.2)
{
  ObstacleMotion o;
  o.position = p;
  o.velocity = v;
  o.radius = r;
  return o;
}
}  // namespace

TEST(Ttc, HeadOnMatchesAnalytic)
{
  // 로봇 +x 1.0 m/s, 장애물 (5, 0) 에서 -x 1.0 m/s → 접근 속도 2, 반경 합 R = 0.361 + 0.2
  const TtcParams p = exactParams();
  const auto robot = RobotMotionModel::straightLine(Vec2::Zero(), 0.0, 1.0);
  const auto res = amr_perception::computeTimeToCollision(
    obstacle(Vec2(5.0, 0.0), Vec2(-1.0, 0.0)), robot, p);
  const double r = p.robot_radius + 0.2;
  EXPECT_NEAR(res.ttc, (5.0 - r) / 2.0, 1e-3);
  // 최근접(CPA) 은 충돌을 찾은 격자 시각까지만 본다 (충돌 직전 격자 2.3 s, 거리 5 - 2·2.3 = 0.4)
  EXPECT_NEAR(res.cpa_time, 2.3, 1e-9);
  EXPECT_NEAR(res.cpa_distance, 0.4, 1e-9);
}

TEST(Ttc, CrossingMatchesAnalytic)
{
  // 로봇 (τ, 0), 장애물 (3, -3 + τ) → |d| = √2 |3 - τ| = R → τ = 3 - R/√2
  const TtcParams p = exactParams();
  const auto robot = RobotMotionModel::straightLine(Vec2::Zero(), 0.0, 1.0);
  const auto res = amr_perception::computeTimeToCollision(
    obstacle(Vec2(3.0, -3.0), Vec2(0.0, 1.0)), robot, p);
  const double r = p.robot_radius + 0.2;
  EXPECT_NEAR(res.ttc, 3.0 - r / std::sqrt(2.0), 1e-3);
}

TEST(Ttc, RecedingAndMissingAreInfinite)
{
  const TtcParams p = exactParams();
  const auto robot = RobotMotionModel::straightLine(Vec2::Zero(), 0.0, 1.0);
  // 같은 방향으로 더 빨리 멀어짐
  const auto away = amr_perception::computeTimeToCollision(
    obstacle(Vec2(3.0, 0.0), Vec2(2.0, 0.0)), robot, p);
  EXPECT_TRUE(std::isinf(away.ttc));
  EXPECT_NEAR(away.cpa_time, 0.0, 1e-12);
  EXPECT_NEAR(away.cpa_distance, 3.0, 1e-12);
  // 옆 차선으로 스쳐 지나감 (최근접 1.0 m > R)
  const auto miss = amr_perception::computeTimeToCollision(
    obstacle(Vec2(4.0, 1.0), Vec2(-1.0, 0.0)), robot, p);
  EXPECT_TRUE(std::isinf(miss.ttc));
  EXPECT_NEAR(miss.cpa_distance, 1.0, 1e-9);
  EXPECT_NEAR(miss.cpa_time, 2.0, p.step);
  // 지평(5 s) 밖 충돌은 inf
  const auto far = amr_perception::computeTimeToCollision(
    obstacle(Vec2(20.0, 0.0), Vec2(-1.0, 0.0)), robot, p);
  EXPECT_TRUE(std::isinf(far.ttc));
}

TEST(Ttc, OverlappingIsZeroAndStationaryRobot)
{
  const TtcParams p = exactParams();
  const auto still = RobotMotionModel::stationary(Vec2(1.0, 1.0));
  EXPECT_DOUBLE_EQ(still.positionAt(3.0).x(), 1.0);
  EXPECT_DOUBLE_EQ(
    amr_perception::computeTimeToCollision(
      obstacle(Vec2(1.3, 1.0), Vec2::Zero()), still, p).ttc, 0.0);
  // 정지 로봇에 다가오는 장애물: (3 - R) / 1
  const auto res = amr_perception::computeTimeToCollision(
    obstacle(Vec2(4.0, 1.0), Vec2(-1.0, 0.0)), still, p);
  EXPECT_NEAR(res.ttc, 3.0 - (p.robot_radius + 0.2), 1e-3);
  // select: 경로 없음 + 속도 0 → 정지 모델
  const auto sel = RobotMotionModel::select({}, Vec2(1.0, 1.0), 0.0, 0.0, p);
  EXPECT_FALSE(sel.followsPath());
  EXPECT_DOUBLE_EQ(sel.speed(), 0.0);
}

TEST(Ttc, FollowsPathAroundCorner)
{
  // L 자 경로: (0,0) → (2,0) → (2,3). 장애물은 모퉁이 뒤 (2, 2) 에 정지.
  // 직선 가정이면 (x 축) 충돌 없음, 경로 추종이면 호 길이 2 + 2 - R 에서 충돌.
  const TtcParams p = exactParams();
  const std::vector<Vec2> path = {Vec2(0.0, 0.0), Vec2(2.0, 0.0), Vec2(2.0, 3.0)};
  RobotMotionModel m;
  ASSERT_TRUE(RobotMotionModel::fromPath(path, Vec2(0.0, 0.05), 1.0, 1.0, m));
  EXPECT_TRUE(m.followsPath());
  EXPECT_NEAR(m.startArcLength(), 0.0, 1e-12);
  EXPECT_NEAR(m.positionAt(2.5).x(), 2.0, 1e-9);
  EXPECT_NEAR(m.positionAt(2.5).y(), 0.5, 1e-9);
  EXPECT_NEAR(m.positionAt(100.0).y(), 3.0, 1e-9);  // 끝점에서 정지
  const auto res = amr_perception::computeTimeToCollision(
    obstacle(Vec2(2.0, 2.0), Vec2::Zero()), m, p);
  EXPECT_NEAR(res.ttc, 4.0 - (p.robot_radius + 0.2), 1e-3);
  const auto line = RobotMotionModel::straightLine(Vec2(0.0, 0.05), 0.0, 1.0);
  EXPECT_TRUE(
    std::isinf(
      amr_perception::computeTimeToCollision(
        obstacle(Vec2(2.0, 2.0), Vec2::Zero()), line, p).ttc));
  // 경로 추종 속도 하한 v_floor: 정지 로봇이라도 경로가 있으면 0.2 m/s 로 진행한다고 본다
  const auto sel = RobotMotionModel::select(path, Vec2(0.0, 0.0), 0.0, 0.0, p);
  EXPECT_TRUE(sel.followsPath());
  EXPECT_DOUBLE_EQ(sel.speed(), p.min_robot_speed);
}

TEST(Ttc, PathRejectedWhenFarOrDegenerate)
{
  const TtcParams p = exactParams();
  RobotMotionModel m;
  EXPECT_FALSE(RobotMotionModel::fromPath({Vec2(0.0, 0.0)}, Vec2::Zero(), 1.0, 1.0, m));
  const std::vector<Vec2> path = {Vec2(0.0, 5.0), Vec2(3.0, 5.0)};
  EXPECT_FALSE(RobotMotionModel::fromPath(path, Vec2::Zero(), 1.0, p.max_path_deviation, m));
  // 멀면 직선 모델로 대체 (실제 속도, 후진 부호 유지)
  const auto sel = RobotMotionModel::select(path, Vec2::Zero(), M_PI / 2.0, -0.3, p);
  EXPECT_FALSE(sel.followsPath());
  EXPECT_NEAR(sel.positionAt(1.0).y(), -0.3, 1e-9);
  // 중복 꼭짓점(길이 0 선분)과 중간 사영
  const std::vector<Vec2> dup = {Vec2(0.0, 0.0), Vec2(0.0, 0.0), Vec2(4.0, 0.0)};
  ASSERT_TRUE(RobotMotionModel::fromPath(dup, Vec2(1.5, 0.2), 0.5, 1.0, m));
  EXPECT_NEAR(m.startArcLength(), 1.5, 1e-9);
  EXPECT_NEAR(m.positionAt(1.0).x(), 2.0, 1e-9);
}

TEST(Ttc, UncertaintyInflationShortensTtcWithCap)
{
  TtcParams p;
  p.k_sigma = 1.0;
  p.sigma_cap = 0.5;
  const auto robot = RobotMotionModel::straightLine(Vec2::Zero(), 0.0, 1.0);
  ObstacleMotion o = obstacle(Vec2(5.0, 0.0), Vec2(-1.0, 0.0));
  const double exact = amr_perception::computeTimeToCollision(o, robot, exactParams()).ttc;
  o.P_vv = Mat2::Identity() * 0.04;  // σ_v = 0.2 m/s
  o.P_pp = Mat2::Identity() * 0.01;
  o.q = 0.25;
  const double inflated = amr_perception::computeTimeToCollision(o, robot, p).ttc;
  EXPECT_LT(inflated, exact);
  EXPECT_GE(inflated, exact - 0.5 / 2.0 - 1e-3);  // 팽창 상한 0.5 m / 접근 속도 2
  // 공분산 성장 폐형식
  o.P_pv = Mat2::Identity() * 0.005;
  const Mat2 c = o.covarianceAt(2.0);
  EXPECT_NEAR(c(0, 0), 0.01 + 2.0 * 2.0 * 0.005 + 4.0 * 0.04 + 0.25 * 8.0 / 3.0, 1e-12);
  EXPECT_NEAR(c(0, 1), 0.0, 1e-12);
  // clearance 부호
  EXPECT_GT(amr_perception::collisionClearance(o, robot, exactParams(), 0.0), 0.0);
  EXPECT_LT(amr_perception::collisionClearance(o, robot, exactParams(), 2.5), 0.0);
}
