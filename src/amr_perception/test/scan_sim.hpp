// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 테스트용 2D 레이캐스팅 스캔 생성기 (원 + 선분 장애물, 가우시안 거리 잡음).

#ifndef SCAN_SIM_HPP_
#define SCAN_SIM_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "amr_perception/geometry2d.hpp"
#include "amr_perception/scan_clustering.hpp"

namespace scan_sim
{

using amr_perception::LaserScanData;
using amr_perception::Pose2D;
using amr_perception::Vec2;

struct Circle
{
  Vec2 center;
  double radius;
};

struct Segment
{
  Vec2 a;
  Vec2 b;
};

/// 광선 o + t·d (|d| = 1) 와 원의 최소 양의 교점 t
inline double rayCircle(const Vec2 & o, const Vec2 & d, const Circle & c)
{
  const Vec2 oc = o - c.center;
  const double b = oc.dot(d);
  const double cc = oc.squaredNorm() - c.radius * c.radius;
  const double disc = b * b - cc;
  if (disc < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double s = std::sqrt(disc);
  const double t1 = -b - s;
  const double t2 = -b + s;
  if (t1 > 1e-9) {
    return t1;
  }
  if (t2 > 1e-9) {
    return t2;
  }
  return std::numeric_limits<double>::infinity();
}

/// 광선과 선분의 교점 t
inline double raySegment(const Vec2 & o, const Vec2 & d, const Segment & s)
{
  const Vec2 e = s.b - s.a;
  const double den = d.x() * e.y() - d.y() * e.x();
  if (std::abs(den) < 1e-12) {
    return std::numeric_limits<double>::infinity();
  }
  const Vec2 w = s.a - o;
  const double t = (w.x() * e.y() - w.y() * e.x()) / den;
  const double u = (w.x() * d.y() - w.y() * d.x()) / den;
  if (t > 1e-9 && u >= 0.0 && u <= 1.0) {
    return t;
  }
  return std::numeric_limits<double>::infinity();
}

/// 센서 자세(추적 프레임)에서 360° 스캔 생성. noise: 거리 σ [m]
inline LaserScanData makeScan(
  const Pose2D & sensor, const std::vector<Circle> & circles,
  const std::vector<Segment> & segments, double noise, std::mt19937 * rng, int samples = 720,
  double range_max = 25.0)
{
  LaserScanData scan;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / samples;
  scan.range_min = 0.1;
  scan.range_max = range_max;
  scan.ranges.resize(samples);
  std::normal_distribution<double> n01(0.0, 1.0);
  for (int i = 0; i < samples; ++i) {
    const double a = scan.angle_min + i * scan.angle_increment + sensor.yaw;
    const Vec2 d(std::cos(a), std::sin(a));
    const Vec2 o = sensor.translation();
    double best = std::numeric_limits<double>::infinity();
    for (const auto & c : circles) {
      best = std::min(best, rayCircle(o, d, c));
    }
    for (const auto & s : segments) {
      best = std::min(best, raySegment(o, d, s));
    }
    if (std::isfinite(best) && best <= range_max) {
      if (rng != nullptr && noise > 0.0) {
        best += noise * n01(*rng);
      }
      scan.ranges[i] = static_cast<float>(best);
    } else {
      scan.ranges[i] = std::numeric_limits<float>::infinity();
    }
  }
  return scan;
}

}  // namespace scan_sim

#endif  // SCAN_SIM_HPP_
