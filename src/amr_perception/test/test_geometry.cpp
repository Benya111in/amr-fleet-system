// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 2D 기하 도우미 단위 테스트.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "amr_perception/geometry2d.hpp"

using amr_perception::Pose2D;
using amr_perception::Vec2;

TEST(Geometry, NormalizeAngleRange)
{
  EXPECT_NEAR(amr_perception::normalizeAngle(3.0 * M_PI), M_PI, 1e-12);
  EXPECT_NEAR(amr_perception::normalizeAngle(-3.0 * M_PI / 2.0), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(amr_perception::normalizeAngle(0.3), 0.3, 1e-12);
  for (double a = -20.0; a < 20.0; a += 0.37) {
    const double n = amr_perception::normalizeAngle(a);
    EXPECT_GT(n, -M_PI - 1e-12);
    EXPECT_LE(n, M_PI + 1e-12);
    EXPECT_NEAR(std::cos(n), std::cos(a), 1e-9);
    EXPECT_NEAR(std::sin(n), std::sin(a), 1e-9);
  }
}

TEST(Geometry, PoseComposeInverse)
{
  const Pose2D a{1.0, 2.0, 0.5};
  const Pose2D b{-0.3, 0.7, -1.2};
  const Vec2 p(0.4, -0.9);
  // (a ∘ b)(p) = a(b(p))
  const Vec2 lhs = a.compose(b).apply(p);
  const Vec2 rhs = a.apply(b.apply(p));
  EXPECT_NEAR((lhs - rhs).norm(), 0.0, 1e-12);
  const Pose2D id = a.compose(a.inverse());
  EXPECT_NEAR(id.x, 0.0, 1e-12);
  EXPECT_NEAR(id.y, 0.0, 1e-12);
  EXPECT_NEAR(id.yaw, 0.0, 1e-12);
  EXPECT_NEAR((a.inverse().apply(a.apply(p)) - p).norm(), 0.0, 1e-12);
  EXPECT_NEAR((a.rotate(Vec2(1.0, 0.0)) - Vec2(std::cos(0.5), std::sin(0.5))).norm(), 0.0, 1e-12);
}

TEST(Geometry, DistanceToRectangle)
{
  // 0.60 × 0.40 풋프린트
  EXPECT_DOUBLE_EQ(amr_perception::distanceToRectangle(Vec2(0.0, 0.0), 0.6, 0.4), 0.0);
  EXPECT_NEAR(amr_perception::distanceToRectangle(Vec2(1.3, 0.0), 0.6, 0.4), 1.0, 1e-12);
  EXPECT_NEAR(amr_perception::distanceToRectangle(Vec2(0.0, -0.5), 0.6, 0.4), 0.3, 1e-12);
  // 모서리 (0.3, 0.2) 에서 대각선 방향
  EXPECT_NEAR(amr_perception::distanceToRectangle(Vec2(0.6, 0.6), 0.6, 0.4), 0.5, 1e-12);
  // LiDAR 원점 전방 0.15 → 전면 모서리까지는 range - 0.15
  EXPECT_NEAR(amr_perception::distanceToRectangle(Vec2(0.15 + 1.0, 0.0), 0.6, 0.4), 0.85, 1e-12);
}

TEST(Geometry, PolygonInsideAndDistance)
{
  const std::vector<Vec2> sq{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  EXPECT_TRUE(amr_perception::pointInPolygon(Vec2(0.5, 0.5), sq));
  EXPECT_FALSE(amr_perception::pointInPolygon(Vec2(1.5, 0.5), sq));
  EXPECT_DOUBLE_EQ(amr_perception::distanceToPolygon(Vec2(0.5, 0.5), sq), 0.0);
  EXPECT_NEAR(amr_perception::distanceToPolygon(Vec2(1.5, 0.5), sq), 0.5, 1e-12);
  EXPECT_NEAR(amr_perception::distanceToPolygon(Vec2(2.0, 2.0), sq), std::sqrt(2.0), 1e-12);
  EXPECT_FALSE(amr_perception::pointInPolygon(Vec2(0.0, 0.0), {}));
  EXPECT_TRUE(std::isinf(amr_perception::distanceToPolygon(Vec2(0.0, 0.0), {})));
  EXPECT_NEAR(amr_perception::distanceToPolygon(Vec2(3.0, 4.0), {Vec2(0.0, 0.0)}), 5.0, 1e-12);
  EXPECT_NEAR(
    amr_perception::distanceToSegment(Vec2(0.5, 1.0), Vec2(0.0, 0.0), Vec2(0.0, 0.0)),
    std::hypot(0.5, 1.0), 1e-12);
}
