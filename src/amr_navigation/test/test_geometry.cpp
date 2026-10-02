// 2D 기하 유틸리티 단위 테스트: 각도 감기, 투영·CTE 부호, 호길이 보간, 이산 곡률, 원호 정확 적분.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "amr_navigation/core/geometry.hpp"

using amr_navigation::core::Point2D;
using amr_navigation::core::Pose2D;
using amr_navigation::core::Projection;
using amr_navigation::core::compose;
using amr_navigation::core::cumulativeLength;
using amr_navigation::core::discreteCurvature;
using amr_navigation::core::integrateArc;
using amr_navigation::core::interpolateAt;
using amr_navigation::core::inverse;
using amr_navigation::core::kPi;
using amr_navigation::core::pathLength;
using amr_navigation::core::projectOntoPath;
using amr_navigation::core::toRobotFrame;
using amr_navigation::core::wrapAngle;

namespace
{
std::vector<Pose2D> straight(double x0, double x1, double step)
{
  std::vector<Pose2D> p;
  for (double x = x0; x <= x1 + 1e-9; x += step) {
    p.push_back({x, 0.0, 0.0});
  }
  return p;
}

std::vector<Pose2D> circle(double R, double arc, double step)
{
  std::vector<Pose2D> p;
  const int n = static_cast<int>(std::round(arc / step));
  for (int i = 0; i <= n; ++i) {
    const double a = i * step / R;
    // 원점 (0, R) 중심, (0,0) 에서 +x 방향 출발, 좌회전
    p.push_back({R * std::sin(a), R - R * std::cos(a), a});
  }
  return p;
}
}  // namespace

TEST(Geometry, WrapAngle)
{
  EXPECT_NEAR(wrapAngle(3.0 * kPi), kPi, 1e-12);
  EXPECT_NEAR(wrapAngle(-kPi), kPi, 1e-12);   // (-π, π]
  EXPECT_NEAR(wrapAngle(0.5), 0.5, 1e-12);
  EXPECT_NEAR(wrapAngle(-2.0 * kPi - 0.25), -0.25, 1e-12);
}

TEST(Geometry, LengthAndCumulative)
{
  const auto p = straight(0.0, 2.0, 0.5);
  EXPECT_NEAR(pathLength(p), 2.0, 1e-12);
  const auto s = cumulativeLength(p);
  ASSERT_EQ(s.size(), p.size());
  EXPECT_NEAR(s.front(), 0.0, 1e-12);
  EXPECT_NEAR(s[2], 1.0, 1e-12);
  EXPECT_NEAR(pathLength({}), 0.0, 1e-12);
}

TEST(Geometry, ProjectionCteSignLeftPositive)
{
  const auto p = straight(0.0, 4.0, 1.0);
  const auto s = cumulativeLength(p);
  const Projection left = projectOntoPath(p, {1.5, 0.3}, 0, 0, &s);
  EXPECT_NEAR(left.cte, 0.3, 1e-12);        // 진행 방향(+x) 기준 좌측 = +y
  EXPECT_NEAR(left.s, 1.5, 1e-12);
  EXPECT_EQ(left.segment, 1U);
  EXPECT_NEAR(left.t, 0.5, 1e-12);
  const Projection right = projectOntoPath(p, {2.2, -0.4}, 0, 0, &s);
  EXPECT_NEAR(right.cte, -0.4, 1e-12);
  EXPECT_NEAR(right.heading, 0.0, 1e-12);
  // 창 제한: [2, 4) 정점만 보면 x=0.5 점도 선분 2 시작점에 투영된다
  const Projection win = projectOntoPath(p, {0.5, 0.0}, 2, 4, &s);
  EXPECT_EQ(win.segment, 2U);
  EXPECT_NEAR(win.s, 2.0, 1e-12);
}

TEST(Geometry, InterpolateSaturates)
{
  const auto p = straight(0.0, 2.0, 1.0);
  const auto s = cumulativeLength(p);
  const Pose2D mid = interpolateAt(p, s, 1.25);
  EXPECT_NEAR(mid.x, 1.25, 1e-12);
  EXPECT_NEAR(mid.theta, 0.0, 1e-12);
  EXPECT_NEAR(interpolateAt(p, s, -1.0).x, 0.0, 1e-12);
  EXPECT_NEAR(interpolateAt(p, s, 10.0).x, 2.0, 1e-12);
}

TEST(Geometry, DiscreteCurvatureOfCircle)
{
  const double R = 2.0;
  const auto p = circle(R, 3.0, 0.05);
  const auto k0 = discreteCurvature(p);
  const auto kw = discreteCurvature(p, 0.25);
  ASSERT_EQ(k0.size(), p.size());
  EXPECT_DOUBLE_EQ(k0.front(), 0.0);
  EXPECT_DOUBLE_EQ(k0.back(), 0.0);
  for (std::size_t i = 10; i + 10 < p.size(); ++i) {
    EXPECT_NEAR(k0[i], 1.0 / R, 1e-3);
    EXPECT_NEAR(kw[i], 1.0 / R, 1e-3);
  }
  // 우회전 = 음수
  auto mirrored = p;
  for (auto & q : mirrored) {
    q.y = -q.y;
  }
  EXPECT_LT(discreteCurvature(mirrored)[20], 0.0);
}

TEST(Geometry, IntegrateArcIsExact)
{
  // 한 번의 원호 적분 == 매우 작은 오일러 스텝 적분 (1e-5 이내)
  const Pose2D start{1.0, -2.0, 0.7};
  const double v = 1.3;
  const double w = -0.9;
  const double T = 2.0;
  const Pose2D exact = integrateArc(start, v, w, T);
  Pose2D e = start;
  const int n = 200000;
  const double h = T / n;
  for (int i = 0; i < n; ++i) {
    const double th = e.theta + 0.5 * w * h;   // 중점 방향
    e.x += v * std::cos(th) * h;
    e.y += v * std::sin(th) * h;
    e.theta += w * h;
  }
  EXPECT_NEAR(exact.x, e.x, 1e-6);
  EXPECT_NEAR(exact.y, e.y, 1e-6);
  EXPECT_NEAR(wrapAngle(exact.theta - e.theta), 0.0, 1e-9);
  // 원 한 바퀴: 제자리로 돌아온다, 반지름 v/ω
  const Pose2D full = integrateArc({0.0, 0.0, 0.0}, 1.0, 0.5, 2.0 * kPi / 0.5);
  EXPECT_NEAR(full.x, 0.0, 1e-9);
  EXPECT_NEAR(full.y, 0.0, 1e-9);
  const Pose2D half = integrateArc({0.0, 0.0, 0.0}, 1.0, 0.5, kPi / 0.5);
  EXPECT_NEAR(half.y, 2.0 * (1.0 / 0.5), 1e-9);
  // ω = 0 은 직선
  const Pose2D line = integrateArc({0.0, 0.0, kPi / 2.0}, 2.0, 0.0, 1.5);
  EXPECT_NEAR(line.x, 0.0, 1e-12);
  EXPECT_NEAR(line.y, 3.0, 1e-12);
}

TEST(Geometry, FrameTransforms)
{
  const Pose2D robot{1.0, 1.0, kPi / 2.0};
  const Point2D local = toRobotFrame(robot, {1.0, 2.0});
  EXPECT_NEAR(local.x, 1.0, 1e-12);   // 로봇 정면 1 m
  EXPECT_NEAR(local.y, 0.0, 1e-12);
  const Pose2D T{2.0, -1.0, 0.3};
  const Pose2D p{0.5, 0.25, -0.2};
  const Pose2D back = compose(inverse(T), compose(T, p));
  EXPECT_NEAR(back.x, p.x, 1e-12);
  EXPECT_NEAR(back.y, p.y, 1e-12);
  EXPECT_NEAR(wrapAngle(back.theta - p.theta), 0.0, 1e-12);
}
