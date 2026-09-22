// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 안전 게이트 단위 테스트: 접근 영역(스윕 풋프린트) 기하, 강건 통계, 존/속도 상한, 0.3 m 정지,
// 0.60 m 통로(측면 벽 + LiDAR 잡음) 무정지, 제자리 회전, 운동 전환 해제(갇힘 없음), 자기 차체 반사,
// 접촉 가드 탈출, E-stop 래치·reset 계약, 센서 고장 디바운스, TTC 감속, 곡률 유지, 도킹 예외(C2),
// 깊이 점군 저상 장애물, 히스테리시스.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "amr_perception/safety_gate.hpp"
#include "scan_sim.hpp"

using amr_perception::Motion;
using amr_perception::Pose2D;
using amr_perception::SafetyGate;
using amr_perception::SafetyParams;
using amr_perception::SafetyStatus;
using amr_perception::SafetyZone;
using amr_perception::SensorFailureAction;
using amr_perception::SensorWatch;
using amr_perception::SweptFootprint;
using amr_perception::Vec2;
using amr_perception::kthSmallest;
using amr_perception::physicalWindowStatistic;
using amr_perception::windowedOrderStatistic;

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
const double kNan = std::numeric_limits<double>::quiet_NaN();

/// 전면 모서리(x = +0.30)에서 거리 d 인 판 (y ∈ [-hw, hw], n 개 연속 빔)
std::vector<Vec2> plate(double d, double hw = 0.25, int n = 41)
{
  std::vector<Vec2> out;
  for (int i = 0; i < n; ++i) {
    out.emplace_back(0.30 + d, -hw + 2.0 * hw * i / (n - 1));
  }
  return out;
}

/// base_link x 범위 [x0, x1] 의 y = y0 벽 (n 개 연속 빔)
std::vector<Vec2> wall(double y0, double x0, double x1, int n = 61)
{
  std::vector<Vec2> out;
  for (int i = 0; i < n; ++i) {
    out.emplace_back(x0 + (x1 - x0) * i / (n - 1), y0);
  }
  return out;
}

std::vector<Vec2> concat(std::vector<Vec2> a, const std::vector<Vec2> & b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

/// LiDAR(base_link x = 0.15) 360° 720 빔 레이캐스팅 → base_link 빔 (무효 = NaN)
std::vector<Vec2> simulateBeams(
  const std::vector<scan_sim::Segment> & segments, const std::vector<scan_sim::Circle> & circles,
  double noise, std::mt19937 * rng)
{
  const Pose2D lidar{0.15, 0.0, 0.0};
  const auto scan = scan_sim::makeScan(lidar, circles, segments, noise, rng);
  std::vector<Vec2> beams(scan.ranges.size(), Vec2(kNan, kNan));
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (std::isfinite(r)) {
      const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      beams[i] = lidar.apply(Vec2(r * std::cos(a), r * std::sin(a)));
    }
  }
  return beams;
}

bool hasReason(const SafetyStatus & st, const std::string & r)
{
  return std::find(st.reasons.begin(), st.reasons.end(), r) != st.reasons.end();
}

bool hasCause(const SafetyStatus & st, const std::string & c)
{
  return std::find(st.stop_causes.begin(), st.stop_causes.end(), c) != st.stop_causes.end();
}

/// 명령을 넣고 즉시 평가
SafetyStatus drive(SafetyGate & g, double v, double w, double t)
{
  g.setCommand(v, w, t);
  return g.evaluate(t);
}

const Vec2 kLidar(0.15, 0.0);           // base_link 의 LiDAR 원점 (sensors.yaml)
const double kInc = 2.0 * M_PI / 720.0;  // 빔 간 각 0.5°

/// 새 스캔 프레임 + 명령 + 평가
SafetyStatus frame(SafetyGate & g, const std::vector<Vec2> & beams, double v, double w, double t)
{
  g.setScan(beams, kLidar, kInc, false, t);
  return drive(g, v, w, t);
}

double pointSpeed(const SafetyStatus & st)
{
  return SweptFootprint::maxPointSpeed(0.30, 0.20, Motion{st.command.linear, st.command.angular});
}
}  // namespace

// ---------------------------------------------------------------- 기하·통계

TEST(SafetyGate, ClearanceLimitAndZoneClassification)
{
  const SafetyParams p;
  // robot_params.yaml 주석의 수치: D = 2.6 → 2.0, 1.0 → 1.04, 0.5 → 0.50
  EXPECT_NEAR(SafetyGate::clearanceSpeedLimit(2.6, p), 2.0, 1e-9);
  EXPECT_NEAR(SafetyGate::clearanceSpeedLimit(1.0, p), 1.0427, 1e-3);
  EXPECT_NEAR(SafetyGate::clearanceSpeedLimit(0.5, p), 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(SafetyGate::clearanceSpeedLimit(0.2, p), 0.0);
  EXPECT_TRUE(std::isinf(SafetyGate::clearanceSpeedLimit(INFINITY, p)));
  // 도킹 예외 정지 거리 0.10 기준: D = 0.25 → -0.15 + √(0.0225 + 0.3)
  EXPECT_NEAR(SafetyGate::clearanceSpeedLimit(0.25, 0.10, p), -0.15 + std::sqrt(0.3225), 1e-12);
  // 제동 거리 d(1.0) = 0.15 + 0.5
  EXPECT_NEAR(SafetyGate::brakingDistance(1.0, p), 0.65, 1e-12);
  EXPECT_NEAR(SafetyGate::brakingDistance(-2.0, p), 2.30, 1e-12);
  EXPECT_EQ(SafetyGate::classifyZone(0.30, p), SafetyZone::kStop);
  EXPECT_EQ(SafetyGate::classifyZone(0.31, p), SafetyZone::kCritical);
  EXPECT_EQ(SafetyGate::classifyZone(0.50, p), SafetyZone::kCritical);
  EXPECT_EQ(SafetyGate::classifyZone(0.51, p), SafetyZone::kWarning);
  EXPECT_EQ(SafetyGate::classifyZone(1.00, p), SafetyZone::kWarning);
  EXPECT_EQ(SafetyGate::classifyZone(1.01, p), SafetyZone::kClear);
  EXPECT_STREQ(amr_perception::zoneName(SafetyZone::kClear), "CLEAR");
  EXPECT_STREQ(amr_perception::zoneName(SafetyZone::kWarning), "WARNING");
  EXPECT_STREQ(amr_perception::zoneName(SafetyZone::kCritical), "CRITICAL");
  EXPECT_STREQ(amr_perception::zoneName(SafetyZone::kStop), "STOP");
}

TEST(SafetyGate, FootprintDistanceUsesEdgeNotRange)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  EXPECT_NEAR(g.footprintDistance(Vec2(1.0, 0.0)), 0.70, 1e-12);   // 전면 모서리 x = 0.30
  EXPECT_NEAR(g.footprintDistance(Vec2(0.0, 0.5)), 0.30, 1e-12);   // 측면 모서리 y = 0.20
  EXPECT_NEAR(g.footprintDistance(Vec2(0.6, 0.6)), std::hypot(0.3, 0.4), 1e-12);  // 꼭짓점
  EXPECT_NEAR(g.circumscribedRadius(), std::hypot(0.3, 0.2), 1e-12);
  EXPECT_NEAR(SweptFootprint::maxPointSpeed(0.3, 0.2, Motion{1.0, 0.0}), 1.0, 1e-12);
  EXPECT_NEAR(
    SweptFootprint::maxPointSpeed(0.3, 0.2, Motion{0.0, 1.0}), std::hypot(0.3, 0.2), 1e-12);
  EXPECT_NEAR(
    SweptFootprint::maxPointSpeed(0.3, 0.2, Motion{1.0, 1.0}), std::hypot(1.2, 0.3), 1e-12);
}

TEST(SafetyGate, SweptFootprintStraightCorridorAndReverse)
{
  SafetyParams p;
  p.swept_margin = 0.02;
  SafetyGate g(p, {}, 0.0);
  const auto r = g.sweptRegion(Motion{0.5, 0.0});
  ASSERT_FALSE(r.empty());
  EXPECT_NEAR(r.approach(Vec2(0.8, 0.0)), 0.5, 1e-12);
  EXPECT_NEAR(r.approach(Vec2(0.8, 0.21)), 0.5, 1e-12);   // 좌우 여유 0.02 안
  EXPECT_TRUE(std::isinf(r.approach(Vec2(0.8, 0.30))));   // 0.60 m 통로 벽 = 통로 밖
  EXPECT_TRUE(std::isinf(r.approach(Vec2(0.0, 0.30))));   // 옆
  EXPECT_TRUE(std::isinf(r.approach(Vec2(0.0, 0.21))));   // 옆 여유 안 = 지금 닿음이 아니다
  EXPECT_TRUE(std::isinf(r.approach(Vec2(-0.8, 0.0))));   // 뒤
  EXPECT_DOUBLE_EQ(r.approach(Vec2(0.29, 0.0)), 0.0);     // 풋프린트 안
  // 영역 길이 (0.5 m/s): max(1.0 + 0.05, 0.3 + d(0.5) = 0.5) + 0.1 = 1.15
  EXPECT_NEAR(r.approach(Vec2(0.30 + 1.14, 0.0)), 1.14, 1e-12);
  EXPECT_TRUE(std::isinf(r.approach(Vec2(0.30 + 1.16, 0.0))));
  // 2.0 m/s: 포락선 0.3 + 0.3 + 2.0 = 2.6 → 2.7
  const auto fast = g.sweptRegion(Motion{2.0, 0.0});
  EXPECT_NEAR(fast.approach(Vec2(0.30 + 2.65, 0.1)), 2.65, 1e-12);
  EXPECT_TRUE(std::isinf(fast.approach(Vec2(0.30 + 2.75, 0.0))));
  // 후진
  const auto rev = g.sweptRegion(Motion{-0.3, 0.0});
  EXPECT_NEAR(rev.approach(Vec2(-0.7, 0.1)), 0.4, 1e-12);
  EXPECT_TRUE(std::isinf(rev.approach(Vec2(0.7, 0.0))));
  // 운동 없음 = 빈 영역
  EXPECT_TRUE(g.sweptRegion(Motion{0.0, 0.0}).empty());
  EXPECT_TRUE(g.sweptRegion(Motion{0.005, 0.02}).empty());
  EXPECT_TRUE(std::isinf(SweptFootprint().approach(Vec2(0.5, 0.0))));
}

TEST(SafetyGate, SweptFootprintRotationSweepsCornersOnly)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  const auto rot = g.sweptRegion(Motion{0.0, 0.5});
  // 0.60 m 통로 벽 (측면 0.10 m): 꼭짓점(외접원 0.361 m)이 쓸고 지나간다
  // (0, 0.30) 은 시계 방향으로 돌아 윗변 y = 0.20 에 x = √(0.09 - 0.04) 에서 닿는다
  const double contact = std::atan2(0.20, std::sqrt(0.09 - 0.04));
  EXPECT_NEAR(rot.approach(Vec2(0.0, 0.30)), 0.30 * (M_PI / 2 - contact), 1e-9);
  EXPECT_LT(rot.approach(Vec2(0.15, 0.30)), 0.2);
  EXPECT_TRUE(std::isinf(rot.approach(Vec2(0.25, 0.30))));  // 꼭짓점 반경(0.361) 밖 벽 점
  // 외접원 밖 = 회전으로 닿지 않는다 (전방 장애물 앞에서 방향 전환 허용)
  EXPECT_TRUE(std::isinf(rot.approach(Vec2(0.0, 0.45))));
  EXPECT_TRUE(std::isinf(rot.approach(Vec2(0.6, 0.0))));
  // 회전 중심 위의 점은 움직이지 않는다
  const auto arc = g.sweptRegion(Motion{0.5, 1.0});
  EXPECT_TRUE(std::isinf(arc.approach(Vec2(0.0, 0.5))));
}

TEST(SafetyGate, SweptFootprintArcMatchesBruteForce)
{
  // 해석해(원-사각형 교점 + 접선 꼬리)를 시간 적분 표본과 비교
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> uv(-0.5, 2.0);
  std::uniform_real_distribution<double> uw(-1.5, 1.5);
  std::uniform_real_distribution<double> up(-2.5, 2.5);
  const double a = 0.30;
  const double b = 0.22;
  int compared = 0;
  int finite = 0;
  int mismatched = 0;
  for (int trial = 0; trial < 400; ++trial) {
    const Motion m{uv(rng), uw(rng)};
    if (std::abs(m.v) < 0.05 || std::abs(m.w) < 0.05) {
      continue;
    }
    const double arc_travel = 0.6;
    const double total = 1.6;
    const SweptFootprint sf(a, b, 0.0, m, arc_travel, total, 0.01, 0.05);
    const double u = SweptFootprint::maxPointSpeed(a, b, m);
    const double t_arc = arc_travel / u;
    const double t_end = t_arc + (total - std::abs(m.v) * t_arc) / std::abs(m.v);
    const double R = m.v / m.w;
    const Pose2D end{m.v / m.w * std::sin(m.w * t_arc), m.v / m.w * (1.0 - std::cos(m.w * t_arc)),
      m.w * t_arc};
    for (int k = 0; k < 10; ++k) {
      const Vec2 p(up(rng), up(rng));
      if (std::abs(p.x()) <= a && std::abs(p.y()) <= b) {
        continue;
      }
      const double dt = 2e-4;
      double s_ref = kInf;
      for (double t = 0.0; t <= t_end; t += dt) {
        Pose2D pose;
        if (t <= t_arc) {
          pose = Pose2D{R * std::sin(m.w * t), R * (1.0 - std::cos(m.w * t)), m.w * t};
        } else {
          pose = end.compose(Pose2D{m.v * (t - t_arc), 0.0, 0.0});
        }
        const Vec2 q = pose.inverse().apply(p);
        if (std::abs(q.x()) <= a && std::abs(q.y()) <= b) {
          s_ref = t <= t_arc ? std::hypot(p.x(), p.y() - R) * std::abs(m.w) * t :
            std::abs(m.v) * t;
          break;
        }
      }
      if (s_ref > total) {
        s_ref = kInf;
      }
      const double s = sf.approach(p);
      // 지평 경계·스침 접촉은 표본 간격에 민감하므로 비교에서 뺀다
      if ((std::isfinite(s_ref) && s_ref > total - 0.01) ||
        (std::isfinite(s) && s > total - 0.01))
      {
        continue;
      }
      ++compared;
      if (std::isfinite(s_ref) != std::isfinite(s)) {
        ++mismatched;
        continue;
      }
      if (std::isfinite(s)) {
        ++finite;
        EXPECT_NEAR(s, s_ref, 3e-3) << "v=" << m.v << " w=" << m.w << " p=" << p.transpose();
      }
    }
  }
  EXPECT_GT(compared, 1000);
  EXPECT_GT(finite, 100);
  EXPECT_LE(mismatched, compared / 200);  // 스침 접촉만 (0.5 % 이하)
}

TEST(SafetyGate, WindowedOrderStatisticRejectsIsolatedBeams)
{
  std::vector<double> v(20, kInf);
  v[5] = 0.1;  // 단일 빔 잡음
  EXPECT_TRUE(std::isinf(windowedOrderStatistic(v, 5, 3, false)));
  v[6] = 0.12;
  EXPECT_TRUE(std::isinf(windowedOrderStatistic(v, 5, 3, false)));
  v[8] = 0.2;  // 창 [5, 9] 에 3 개 → 세 번째로 작은 값
  EXPECT_NEAR(windowedOrderStatistic(v, 5, 3, false), 0.2, 1e-12);
  std::vector<double> c(20, kInf);
  c[18] = 0.3;
  c[19] = 0.3;
  c[0] = 0.3;  // 360° 이음매를 넘는 물체
  EXPECT_NEAR(windowedOrderStatistic(c, 5, 3, true), 0.3, 1e-12);
  EXPECT_TRUE(std::isinf(windowedOrderStatistic(c, 5, 3, false)));
  EXPECT_NEAR(windowedOrderStatistic({0.5, 0.4, 0.6}, 5, 3, false), 0.6, 1e-12);
  EXPECT_TRUE(std::isinf(windowedOrderStatistic({}, 5, 3, false)));
  EXPECT_NEAR(kthSmallest({3.0, 1.0, 2.0}, 2), 2.0, 1e-12);
  EXPECT_TRUE(std::isinf(kthSmallest({1.0}, 3)));
}

TEST(SafetyGate, PhysicalWindowGrowsNearTheSensor)
{
  // 0.3 m 거리: 4 cm 창 = 0.04 / (0.3 · 0.5°) ≈ 16 빔 → 17 빔, 60 % = 11 빔이 가까워야 한다
  const double inc = 2.0 * M_PI / 720.0;
  std::vector<double> vals(100, kInf);
  std::vector<double> near(100, 0.3);
  for (int i = 40; i < 45; ++i) {
    vals[i] = 0.05;  // 5 빔 (1.3 cm) — 가까운 거리에서는 잡음 수준
  }
  EXPECT_TRUE(std::isinf(physicalWindowStatistic(vals, near, inc, 0.04, 5, 0.6, false)));
  for (int i = 40; i < 55; ++i) {
    vals[i] = 0.05;  // 15 빔 (3.9 cm) — 물체
  }
  EXPECT_NEAR(physicalWindowStatistic(vals, near, inc, 0.04, 5, 0.6, false), 0.05, 1e-12);
  // 3 m 거리: 창은 최소 5 빔 → 3 빔 물체 (7.9 cm) 검출
  std::vector<double> far(100, 3.0);
  std::vector<double> v2(100, kInf);
  v2[10] = v2[11] = v2[12] = 1.0;
  EXPECT_NEAR(physicalWindowStatistic(v2, far, inc, 0.04, 5, 0.6, false), 1.0, 1e-12);
  v2[12] = kInf;
  EXPECT_TRUE(std::isinf(physicalWindowStatistic(v2, far, inc, 0.04, 5, 0.6, false)));
  // 이음매 순환, 길이 불일치
  std::vector<double> c(100, kInf);
  c[99] = c[0] = c[1] = 2.0;
  EXPECT_NEAR(physicalWindowStatistic(c, far, inc, 0.04, 5, 0.6, true), 2.0, 1e-12);
  EXPECT_TRUE(std::isinf(physicalWindowStatistic(c, far, inc, 0.04, 5, 0.6, false)));
  EXPECT_TRUE(std::isinf(physicalWindowStatistic(c, {1.0}, inc, 0.04, 5, 0.6, false)));
}

// ---------------------------------------------------------------- 존과 정지

TEST(SafetyGate, ClearWarningCriticalSpeedCaps)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  // CLEAR: 여유 1.5 m → 연속 상한 -0.15 + √(0.0225 + 2·1.2) = 1.4065
  auto st = frame(g, plate(1.5), 2.0, 0.0, 0.0);
  EXPECT_EQ(st.zone, SafetyZone::kClear);
  EXPECT_NEAR(st.command.linear, -0.15 + std::sqrt(0.0225 + 2.4), 1e-9);
  EXPECT_FALSE(st.estop_active);
  // WARNING (0.8 m): 0.5 m/s 상한
  st = frame(g, plate(0.8), 1.0, 0.0, 0.1);
  EXPECT_EQ(st.zone, SafetyZone::kWarning);
  EXPECT_NEAR(st.command.linear, 0.5, 1e-12);
  EXPECT_TRUE(hasReason(st, "warning_zone"));
  // CRITICAL (0.4 m): 0.2 m/s 상한
  st = frame(g, plate(0.4), 1.0, 0.0, 0.2);
  EXPECT_EQ(st.zone, SafetyZone::kCritical);
  EXPECT_NEAR(st.command.linear, 0.2, 1e-12);
  EXPECT_TRUE(hasReason(st, "critical_zone"));
  EXPECT_FALSE(st.estop_active);
  // 빈 스캔: 제한 없음 (하드웨어 한계로만 자름)
  st = frame(g, {}, 3.0, 2.0, 0.3);
  EXPECT_NEAR(st.command.linear, 2.0, 1e-12);   // limits.max_linear_velocity
  EXPECT_NEAR(st.command.angular, 1.5, 1e-12);  // limits.max_angular_velocity
}

TEST(SafetyGate, ApproachingObstacleStopsAndIsNotAnEstop)
{
  // 50 Hz 로 스캔이 오고 장애물이 1.0 m/s 로 다가온다. D <= 0.30 인 첫 주기에 출력 0,
  // 두 번째 프레임 안에 STOP 확정. 근접 정지는 E-stop 이 아니다 (계약 C1).
  SafetyGate g(SafetyParams{}, {}, 0.0);
  const double dt = 0.02;
  int first_inside = -1;
  int first_zero = -1;
  int first_stop = -1;
  for (int i = 0; i < 100 && first_stop < 0; ++i) {
    const double t = i * dt;
    const double d = 1.21 - 1.0 * t;  // 경계값(정확히 0.30)을 피한 격자: 0.31 → 0.29
    const auto st = frame(g, plate(d), 0.5, 0.0, t);
    EXPECT_FALSE(st.estop_active);
    if (d <= 0.30 && first_inside < 0) {
      first_inside = i;
    }
    if (st.command.linear == 0.0 && first_zero < 0) {
      first_zero = i;
    }
    if (st.zone == SafetyZone::kStop) {
      first_stop = i;
      EXPECT_TRUE(st.proximity_stop);
      EXPECT_TRUE(hasReason(st, "proximity_stop"));
      EXPECT_TRUE(hasCause(st, "lidar"));
    }
    if (first_zero < 0) {
      EXPECT_GT(st.min_distance, 0.30);
    }
  }
  ASSERT_GE(first_inside, 0);
  EXPECT_EQ(first_zero, first_inside);        // 여유 거리 제한이 같은 주기에 0
  EXPECT_LE(first_stop, first_inside + 1);    // 2 프레임 확정
  // 장애물이 물러나도 같은 운동에서는 0.5 m (stop_release_distance) 를 넘기 전 정지 유지
  EXPECT_DOUBLE_EQ(frame(g, plate(0.45), 0.5, 0.0, 3.0).command.linear, 0.0);
  const auto released = frame(g, plate(0.55), 0.5, 0.0, 3.02);
  EXPECT_FALSE(released.proximity_stop);
  EXPECT_EQ(released.zone, SafetyZone::kWarning);
  EXPECT_NEAR(released.command.linear, 0.5, 1e-12);  // WARNING 상한 0.5
}

TEST(SafetyGate, NarrowAisleWallsNeitherStopNorCap)
{
  // 리뷰 결함 재현: 0.60 m 통로(풋프린트 옆 0.10 m) 벽 → omni 모드는 정지(D = 0.10)였다.
  // 접근 영역에서는 측면 벽이 통로 밖이고, σ 0.03 LiDAR 잡음에서도 STOP·감속이 없어야 한다.
  for (const double offset : {0.0, 0.03, -0.03}) {
    SafetyGate g(SafetyParams{}, {}, 0.0);
    std::mt19937 rng(17);
    std::normal_distribution<double> n01(0.0, 1.0);
    const std::vector<scan_sim::Segment> walls = {
      {Vec2(-4.0, 0.30 - offset), Vec2(4.0, 0.30 - offset)},
      {Vec2(-4.0, -0.30 - offset), Vec2(4.0, -0.30 - offset)}};
    int stops = 0;
    int capped = 0;
    for (int k = 0; k < 600; ++k) {
      const double t = 0.1 * k;
      // 측정 속도: 엔코더 50 Hz, 슬립 잡음 (각속도 σ 0.04 rad/s — wheel_odom 실측 수준)
      for (int j = 0; j < 5; ++j) {
        g.setMeasuredVelocity(1.0 + 0.01 * n01(rng), 0.04 * n01(rng), t - 0.08 + 0.02 * j);
      }
      g.setScan(simulateBeams(walls, {}, 0.03, &rng), kLidar, kInc, true, t);
      const double w_cmd = 0.02 * n01(rng) - 0.5 * offset;  // 중심선 추종 조향 (중심 쪽)
      const auto st = drive(g, 1.0, w_cmd, t);
      stops += st.proximity_stop ? 1 : 0;
      capped += st.command.linear < 1.0 - 1e-9 ? 1 : 0;
      EXPECT_FALSE(st.estop_active);
    }
    EXPECT_EQ(stops, 0) << "offset " << offset;
    if (offset == 0.0) {
      EXPECT_EQ(capped, 0);
    } else {
      EXPECT_LE(capped, 6) << "offset " << offset;  // 드문 WARNING (≤ 1 %)
    }
  }
  // 통로 입구: 벽이 범퍼 0.25 m 앞에서 시작해도 1.0 m/s 통과
  SafetyGate g(SafetyParams{}, {}, 0.0);
  const auto entry = concat(wall(0.30, 0.55, 4.0), wall(-0.30, 0.55, 4.0));
  for (int k = 0; k < 3; ++k) {
    const auto st = frame(g, entry, 1.0, 0.0, 0.1 * k);
    EXPECT_NEAR(st.command.linear, 1.0, 1e-12);
    EXPECT_EQ(st.zone, SafetyZone::kClear);
  }
}

TEST(SafetyGate, RotationInPlaceInsideAisleIsStopped)
{
  // 통로 안 제자리 회전은 꼭짓점이 벽을 긁는다 → 회전 명령 0, STOP
  SafetyGate g(SafetyParams{}, {}, 0.0);
  const auto aisle = concat(wall(0.30, -1.0, 1.0), wall(-0.30, -1.0, 1.0));
  auto st = frame(g, aisle, 0.0, 0.5, 0.0);
  EXPECT_DOUBLE_EQ(st.command.angular, 0.0);
  st = frame(g, aisle, 0.0, 0.5, 0.1);
  EXPECT_EQ(st.zone, SafetyZone::kStop);
  EXPECT_FALSE(st.estop_active);
  // 같은 통로에서 직진은 곧바로 풀린다
  st = frame(g, aisle, 0.8, 0.0, 0.2);
  EXPECT_FALSE(st.proximity_stop);
  EXPECT_NEAR(st.command.linear, 0.8, 1e-12);
}

TEST(SafetyGate, MovingAwayReleasesStopSoRobotIsNeverTrapped)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  const auto obstacle = plate(0.20);
  auto st = frame(g, obstacle, 0.3, 0.0, 0.0);  // 0.20 <= 0.25 → 한 프레임에 확정
  EXPECT_EQ(st.zone, SafetyZone::kStop);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  // 아직 앞으로 움직이는 중(측정 속도)이면 후진 명령도 0 — 측정 운동이 장애물에 다가간다
  g.setMeasuredVelocity(0.3, 0.0, 0.1);
  st = frame(g, obstacle, -0.4, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  // 멈추면(엔코더 50 Hz 0 → 저역 통과 출력이 감쇠) 후진은 장애물에서 멀어지는 운동 → 해제
  for (double t = 0.12; t <= 0.8; t += 0.02) {
    g.setMeasuredVelocity(0.0, 0.0, t);
  }
  st = frame(g, obstacle, -0.4, 0.0, 0.8);
  EXPECT_FALSE(st.proximity_stop);
  EXPECT_EQ(st.zone, SafetyZone::kClear);
  EXPECT_NEAR(st.command.linear, -0.4, 1e-12);
  // 제자리 회전도 전방 0.5 m 판에 닿지 않으므로 허용
  st = frame(g, obstacle, 0.0, 0.8, 0.9);
  EXPECT_NEAR(st.command.angular, 0.8, 1e-12);
  // 다시 전진하면 STOP
  st = frame(g, obstacle, 0.3, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  EXPECT_EQ(st.zone, SafetyZone::kStop);
}

TEST(SafetyGate, SelfReturnsDoNotBlockEscape)
{
  // 리뷰 결함 재현: 자기 차체 점 하나가 탈출 판정을 막았다 (probe2: (0.10, 0.05) 추가 시 v = 0).
  SafetyGate g(SafetyParams{}, {}, 0.0);
  auto beams = plate(0.20);
  // 슬롯 기둥 반사 (풋프린트 안쪽 연속 빔)
  for (int i = 0; i < 6; ++i) {
    beams.emplace_back(0.10, 0.05 + 0.01 * i);
  }
  beams.emplace_back(-0.2, 0.1);
  auto st = frame(g, beams, 0.3, 0.0, 0.0);
  EXPECT_EQ(st.zone, SafetyZone::kStop);
  EXPECT_NEAR(st.min_distance, 0.20, 1e-12);
  st = frame(g, beams, -0.2, 0.0, 0.1);
  EXPECT_NEAR(st.command.linear, -0.2, 1e-12);
  EXPECT_FALSE(st.proximity_stop);
  EXPECT_TRUE(std::isinf(st.contact_distance) || st.contact_distance > 0.1);
}

TEST(SafetyGate, ContactGuardAllowsOnlyMovingAway)
{
  // 왼쪽 측면 0.015 m 에 벽 (스윕 통로 밖, 접촉 가드 0.02 안)
  SafetyGate g(SafetyParams{}, {}, 0.0);
  const auto side = wall(0.215, -1.0, 1.0, 201);
  auto st = frame(g, side, 0.0, 0.0, 0.0);
  EXPECT_FALSE(st.proximity_stop);  // 1 프레임 — 확정 대기
  st = frame(g, side, 0.0, 0.0, 0.1);
  EXPECT_EQ(st.zone, SafetyZone::kStop);
  EXPECT_TRUE(hasCause(st, "contact"));
  EXPECT_FALSE(st.estop_active);
  EXPECT_NEAR(st.contact_distance, 0.015, 1e-9);
  // 벽 쪽으로 도는 원호, 꼬리가 벽을 치는 반대 원호, 제자리 회전 → 0
  EXPECT_DOUBLE_EQ(frame(g, side, 0.3, 0.5, 0.2).command.linear, 0.0);
  EXPECT_DOUBLE_EQ(frame(g, side, 0.3, -0.5, 0.3).command.linear, 0.0);
  EXPECT_DOUBLE_EQ(frame(g, side, 0.0, -0.5, 0.4).command.angular, 0.0);
  // 벽과 나란한 직진·후진은 멀어지지는 않아도 가까워지지 않는다 → 탈출 속도(0.2)로 통과
  st = frame(g, side, 0.5, 0.0, 0.5);
  EXPECT_TRUE(st.escaping);
  EXPECT_NEAR(st.command.linear, 0.2, 1e-12);
  st = frame(g, side, -0.3, 0.0, 0.6);
  EXPECT_NEAR(st.command.linear, -0.2, 1e-12);
  // 벽이 떨어지면 해제
  st = frame(g, wall(0.32, -1.0, 1.0, 201), 0.5, 0.0, 0.7);
  EXPECT_FALSE(st.proximity_stop);
  EXPECT_NEAR(st.command.linear, 0.5, 1e-12);
  // 탈출 끔
  SafetyParams no_escape;
  no_escape.allow_escape = false;
  SafetyGate g2(no_escape, {}, 0.0);
  frame(g2, side, 0.0, 0.0, 0.0);
  frame(g2, side, 0.0, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(frame(g2, side, 0.5, 0.0, 0.2).command.linear, 0.0);
}

TEST(SafetyGate, NoisyObstacleNearThresholdDoesNotTripStop)
{
  // 전방 0.40 m 판, σ 0.03 잡음 1000 프레임: 단일 빔 최솟값이면 자주 0.30 아래로 떨어지지만
  // 빔 창 순위 통계 + 2 프레임 확정은 STOP 을 걸지 않는다.
  SafetyGate g(SafetyParams{}, {}, 0.0);
  std::mt19937 rng(23);
  const std::vector<scan_sim::Segment> front = {{Vec2(0.70, -0.4), Vec2(0.70, 0.4)}};
  int stops = 0;
  int single_beam_below = 0;
  for (int k = 0; k < 1000; ++k) {
    const auto beams = simulateBeams(front, {}, 0.03, &rng);
    double raw = kInf;
    for (const auto & b : beams) {
      if (std::isfinite(b.x())) {
        raw = std::min(raw, g.footprintDistance(b));
      }
    }
    single_beam_below += raw <= 0.30 ? 1 : 0;
    g.setScan(beams, kLidar, kInc, true, 0.1 * k);
    stops += drive(g, 0.05, 0.0, 0.1 * k).proximity_stop ? 1 : 0;
  }
  EXPECT_GT(single_beam_below, 10);  // 단일 빔 판정이었다면 정지했을 프레임
  EXPECT_EQ(stops, 0);
}

TEST(SafetyGate, HoldMotionIntentKeepsStopUntilMotionChanges)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  frame(g, plate(0.2), 0.3, 0.0, 0.0);
  auto st = frame(g, plate(0.2), 0.0, 0.0, 0.1);  // 정지 명령 — 마지막 운동 방향 유지
  EXPECT_EQ(st.zone, SafetyZone::kStop);
  st = frame(g, plate(0.2), -0.2, 0.0, 0.2);
  EXPECT_FALSE(st.proximity_stop);
  SafetyParams p;
  p.hold_motion_intent = false;
  SafetyGate g2(p, {}, 0.0);
  frame(g2, plate(0.2), 0.3, 0.0, 0.0);
  EXPECT_EQ(frame(g2, plate(0.2), 0.0, 0.0, 0.1).zone, SafetyZone::kClear);
}

TEST(SafetyGate, MeasuredVelocityExtendsRegion)
{
  // 명령은 0 (상위 평활기가 감속 중) 이어도 실제로 앞으로 움직이면 전방이 접근 영역이다
  SafetyGate g(SafetyParams{}, {}, 0.0);
  g.setMeasuredVelocity(0.8, 0.0, 0.0);
  auto st = frame(g, plate(0.2), 0.0, 0.0, 0.0);
  EXPECT_EQ(st.zone, SafetyZone::kStop);
  // 측정 속도가 끊기면(0.2 s) 가설에서 빠진다 → 명령 운동도 없으므로 빈 영역
  SafetyGate g2(SafetyParams{}, {}, 0.0);
  g2.setMeasuredVelocity(0.8, 0.0, 0.0);
  EXPECT_EQ(frame(g2, plate(0.2), 0.0, 0.0, 0.5).zone, SafetyZone::kClear);
  // 저역 통과: 한 표본의 튐은 반만 반영된다 (Δt = τ = 0.1 s)
  SafetyGate g3(SafetyParams{}, {}, 0.0);
  g3.setMeasuredVelocity(0.0, 0.0, 0.0);
  g3.setMeasuredVelocity(1.0, 0.0, 0.1);
  g3.setMeasuredVelocity(NAN, 0.0, 0.15);  // 비유한 값은 무시
  EXPECT_EQ(frame(g3, plate(0.6), 0.0, 0.0, 0.1).zone, SafetyZone::kWarning);
}

// ---------------------------------------------------------------- E-stop

TEST(SafetyGate, EstopLatchReleasesOnlyOnExplicitFalseAndReset)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  g.setEstopSource("estop", true);
  auto st = drive(g, 0.5, 0.0, 0.0);
  EXPECT_TRUE(st.estop_latched);
  EXPECT_TRUE(st.estop_active);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  EXPECT_TRUE(hasReason(st, "estop_latched"));
  // false 만으로는 해제되지 않는다 (reset 필요)
  g.setEstopSource("estop", false);
  EXPECT_TRUE(drive(g, 0.5, 0.0, 0.1).estop_latched);
  // reset → 해제
  const auto r = g.requestReset();
  EXPECT_TRUE(r.success);
  st = drive(g, 0.5, 0.0, 0.2);
  EXPECT_FALSE(st.estop_latched);
  EXPECT_FALSE(st.estop_active);
  EXPECT_NEAR(st.command.linear, 0.5, 1e-12);
  // 래치 없을 때 reset 은 성공 (무동작)
  EXPECT_TRUE(g.requestReset().success);
}

TEST(SafetyGate, RejectedResetNeverReleasesLatch)
{
  // 리뷰 결함 재현: 거절된 reset 뒤 1 s 안에 false 가 오면 래치가 풀렸다 (probe_gate: v = 0.5).
  SafetyGate g(SafetyParams{}, {}, 0.0);
  g.setEstopSource("estop", true);
  const auto r = g.requestReset();
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.message.find("estop"), std::string::npos);
  g.setEstopSource("estop", false);  // 거절 0.8 s 뒤 false
  auto st = drive(g, 0.5, 0.0, 0.9);
  EXPECT_TRUE(st.estop_latched);
  EXPECT_TRUE(st.estop_active);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  EXPECT_TRUE(g.requestReset().success);  // false 뒤의 reset 만 해제한다
  EXPECT_NEAR(drive(g, 0.5, 0.0, 1.0).command.linear, 0.5, 1e-12);
  // 두 입력 중 하나라도 true 면 거절
  g.setEstopSource("estop", true);
  g.setEstopSource("fleet_estop", true);
  g.setEstopSource("estop", false);
  EXPECT_FALSE(g.requestReset().success);
  g.setEstopSource("fleet_estop", false);
  EXPECT_TRUE(g.estopLatched());
  EXPECT_TRUE(g.requestReset().success);
  EXPECT_FALSE(g.estopLatched());
}

TEST(SafetyGate, EstopWithoutResetRequirement)
{
  SafetyParams p;
  p.estop_release_requires_reset = false;
  SafetyGate g(p, {}, 0.0);
  g.setEstopSource("estop", true);
  EXPECT_TRUE(g.estopLatched());
  g.setEstopSource("estop", false);
  EXPECT_FALSE(g.estopLatched());
  EXPECT_FALSE(drive(g, 0.5, 0.0, 0.0).estop_active);
}

// ---------------------------------------------------------------- 센서 고장

TEST(SafetyGate, SensorFaultDebounceDerivedFromRate)
{
  // 휠 엔코더 50 Hz: 지연 경고 0.06 s (3 주기), 고장 0.2 s (10 주기).
  // 리뷰 결함 재현: 60 ms 한 번의 간격이 정지 + estop_active 였다.
  std::vector<SensorWatch> watches = {
    {"wheel_encoder", 0.06, 0.20, SensorFailureAction::kStop},
    {"lidar", 0.3, 0.3, SensorFailureAction::kStop},
    {"imu", 0.05, 0.20, SensorFailureAction::kDegraded},
  };
  SafetyGate g(SafetyParams{}, watches, 0.0);
  auto beat = [&g](double t) {
      g.sensorHeartbeat("wheel_encoder", t);
      g.sensorHeartbeat("lidar", t);
      g.sensorHeartbeat("imu", t);
    };
  // 기동 유예: 시작 시각에 본 것으로 간주
  auto st = drive(g, 1.0, 0.0, 0.02);
  EXPECT_TRUE(st.failed_sensors.empty());
  EXPECT_NEAR(st.command.linear, 1.0, 1e-12);
  beat(1.0);
  g.sensorHeartbeat("unknown", 1.0);  // 감시 대상 아님 → 무시
  // 한 번의 80 ms 간격 → 지연 경고만, 속도·estop_active 영향 없음
  st = drive(g, 1.0, 0.0, 1.08);
  EXPECT_FALSE(st.sensor_stop);
  EXPECT_FALSE(st.estop_active);
  EXPECT_NEAR(st.command.linear, 1.0, 1e-12);
  ASSERT_EQ(st.late_sensors.size(), 2U);  // wheel_encoder, imu
  EXPECT_TRUE(hasReason(st, "sensor_late"));
  beat(1.08);
  EXPECT_TRUE(drive(g, 1.0, 0.0, 1.09).late_sensors.empty());
  // 실제 끊김: 마지막 수신 후 0.2 s 를 넘는 첫 주기에 정지 + estop_active (계약 C1: 고장 정지)
  st = drive(g, 1.0, 0.0, 1.27);
  EXPECT_FALSE(st.sensor_stop);
  st = drive(g, 1.0, 0.0, 1.29);
  EXPECT_TRUE(st.sensor_stop);
  EXPECT_TRUE(st.estop_active);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  EXPECT_TRUE(hasReason(st, "sensor_failure_stop"));
  EXPECT_TRUE(st.degraded);  // IMU 는 저속형
  EXPECT_NE(
    std::find(st.failed_sensors.begin(), st.failed_sensors.end(), "imu"),
    st.failed_sensors.end());
  // 복구
  beat(1.3);
  st = drive(g, 1.0, 0.0, 1.31);
  EXPECT_FALSE(st.sensor_stop);
  EXPECT_FALSE(st.degraded);
  EXPECT_FALSE(st.estop_active);
  EXPECT_NEAR(st.command.linear, 1.0, 1e-12);
  // IMU 만 끊기면 저속 (0.2 m/s), estop_active 아님
  g.sensorHeartbeat("wheel_encoder", 1.5);
  g.sensorHeartbeat("lidar", 1.5);
  st = drive(g, 1.0, 0.0, 1.55);
  EXPECT_TRUE(st.degraded);
  EXPECT_FALSE(st.estop_active);
  EXPECT_NEAR(st.command.linear, 0.2, 1e-12);
  EXPECT_TRUE(hasReason(st, "sensor_failure_degraded"));
}

TEST(SafetyGate, StaleCommandIsZeroed)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  EXPECT_TRUE(g.evaluate(0.0).command_stale);  // 명령을 한 번도 안 받음
  g.setCommand(0.8, 0.3, 0.0);
  EXPECT_NEAR(g.evaluate(0.4).command.linear, 0.8, 1e-12);
  const auto st = g.evaluate(0.6);
  EXPECT_TRUE(st.command_stale);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(st.command.angular, 0.0);
  EXPECT_TRUE(hasReason(st, "command_stale"));
  // 비유한 입력은 0 으로
  g.setCommand(NAN, INFINITY, 1.0);
  EXPECT_DOUBLE_EQ(g.evaluate(1.0).command.linear, 0.0);
}

// ---------------------------------------------------------------- 속도 제한

TEST(SafetyGate, TtcContinuousDeceleration)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  // τ_crit 2.15 s 이하에서 v <= a·(TTC - t_react)
  g.setMinTtc(1.0, 0.0);
  auto st = drive(g, 1.5, 0.0, 0.0);
  EXPECT_NEAR(st.command.linear, 1.0 * (1.0 - 0.15), 1e-12);
  EXPECT_TRUE(hasReason(st, "ttc_limit"));
  EXPECT_NEAR(st.min_ttc, 1.0, 1e-12);
  g.setMinTtc(0.1, 0.1);
  EXPECT_DOUBLE_EQ(drive(g, 1.5, 0.0, 0.1).command.linear, 0.0);
  // τ_crit 보다 크면 제한 없음
  g.setMinTtc(3.0, 0.2);
  EXPECT_NEAR(drive(g, 1.5, 0.0, 0.2).command.linear, 1.5, 1e-12);
  // 오래된 TTC (> ttc_max_age 0.5 s) 는 무시, NaN 은 inf 로
  g.setMinTtc(0.5, 0.3);
  st = drive(g, 1.5, 0.0, 0.9);
  EXPECT_NEAR(st.command.linear, 1.5, 1e-12);
  EXPECT_TRUE(std::isinf(st.min_ttc));
  g.setMinTtc(NAN, 1.0);
  EXPECT_NEAR(drive(g, 1.5, 0.0, 1.0).command.linear, 1.5, 1e-12);
  // 거리 존 상한이 더 낮으면 존이 이긴다 (TTC 1.0 → 0.85, 존 WARNING → 0.5)
  g.setMinTtc(1.0, 1.1);
  st = frame(g, plate(0.8), 1.5, 0.0, 1.1);
  EXPECT_NEAR(st.command.linear, 0.5, 1e-12);
  EXPECT_FALSE(hasReason(st, "ttc_limit"));
  // 비활성
  SafetyParams off;
  off.ttc_limit_enabled = false;
  SafetyGate g2(off, {}, 0.0);
  g2.setMinTtc(0.3, 0.0);
  EXPECT_NEAR(drive(g2, 1.0, 0.0, 0.0).command.linear, 1.0, 1e-12);
}

TEST(SafetyGate, PointSpeedLimitPreservesCurvature)
{
  // 거리·저속 상한은 풋프린트 최고 점 속도에 걸고 v, ω 를 같은 비율로 줄인다
  std::vector<SensorWatch> watches = {{"imu", 0.05, 0.1, SensorFailureAction::kDegraded}};
  SafetyGate g(SafetyParams{}, watches, 0.0);
  auto st = drive(g, 1.0, 1.5, 0.5);  // IMU 고장 → 저속 0.2
  EXPECT_TRUE(st.degraded);
  EXPECT_NEAR(pointSpeed(st), 0.2, 1e-9);
  EXPECT_NEAR(st.command.angular / st.command.linear, 1.5, 1e-9);  // 곡률 유지
  st = drive(g, 0.0, 1.0, 0.6);  // 제자리 회전: 꼭짓점 속도 0.2 → ω = 0.2 / r_circ
  EXPECT_NEAR(st.command.angular, 0.2 / g.circumscribedRadius(), 1e-9);
  EXPECT_NEAR(st.angular_limit, 0.2 / g.circumscribedRadius(), 1e-9);
  // WARNING 존의 원호 명령도 점 속도 0.5 이하
  SafetyGate g2(SafetyParams{}, {}, 0.0);
  st = frame(g2, plate(0.8, 1.0, 81), 1.0, 0.5, 0.0);
  EXPECT_EQ(st.zone, SafetyZone::kWarning);
  EXPECT_LE(pointSpeed(st), 0.5 + 1e-9);
  EXPECT_NEAR(st.command.angular / st.command.linear, 0.5, 1e-9);
}

// ---------------------------------------------------------------- 도킹 예외 (C2)

TEST(SafetyGate, DockExclusionLetsDockedStandoffPassWithoutStop)
{
  // 도킹 판(마커 면)을 향해 0.1 m/s 로 접근해 범퍼-판 0.35 m (standoff 0.65) 에서 멈춘다.
  // 판 점은 σ 0.03 잡음. 예외 다각형이 있으면 판 점의 정지 거리는 0.10 m → STOP 없음.
  SafetyGate g(SafetyParams{}, {}, 0.0);
  std::mt19937 rng(5);
  const double plate_x = 2.0;  // 월드 판 면 x
  int stops = 0;
  double x_robot = 0.0;
  double t = 0.0;
  for (int k = 0; k < 400; ++k) {
    t = 0.05 * k;
    const double bumper_gap = plate_x - (x_robot + 0.30);
    // 다각형 (월드 고정: 판 앞 0.2 m ~ 판 뒤 0.5 m, 폭 1.0 m) → base_link
    const std::vector<Vec2> poly = {
      Vec2(plate_x - 0.2 - x_robot, -0.5), Vec2(plate_x + 0.5 - x_robot, -0.5),
      Vec2(plate_x + 0.5 - x_robot, 0.5), Vec2(plate_x - 0.2 - x_robot, 0.5)};
    g.setExclusionPolygon(poly, t);
    const std::vector<scan_sim::Segment> seg = {
      {Vec2(plate_x - x_robot, -0.3), Vec2(plate_x - x_robot, 0.3)}};
    g.setScan(simulateBeams(seg, {}, 0.03, &rng), kLidar, kInc, true, t);
    const double cmd = bumper_gap > 0.35 ? 0.1 : 0.0;
    const auto st = drive(g, cmd, 0.0, t);
    stops += st.zone == SafetyZone::kStop ? 1 : 0;
    x_robot += st.command.linear * 0.05;
    if (bumper_gap < 0.9) {
      EXPECT_TRUE(hasReason(st, "dock_exclusion"));
    }
  }
  EXPECT_EQ(stops, 0);
  EXPECT_NEAR(plate_x - (x_robot + 0.30), 0.35, 0.01);
  // 예외 안이라도 0.10 m 이내면 정지
  SafetyGate g2(SafetyParams{}, {}, 0.0);
  const std::vector<Vec2> poly = {
    Vec2(0.32, -0.5), Vec2(1.0, -0.5), Vec2(1.0, 0.5), Vec2(0.32, 0.5)};
  g2.setExclusionPolygon(poly, 0.0);
  auto st = frame(g2, plate(0.2), 0.5, 0.0, 0.0);
  EXPECT_FALSE(st.proximity_stop);
  EXPECT_NEAR(st.command.linear, 0.2, 1e-12);  // 예외 점이 영역에 있으면 CRITICAL 상한
  g2.setExclusionPolygon(poly, 0.1);
  st = frame(g2, plate(0.04), 0.1, 0.0, 0.1);
  EXPECT_TRUE(st.proximity_stop);
  EXPECT_TRUE(hasCause(st, "dock_exclusion"));
  // 다각형 만료 (0.3 s) → 일반 규칙 (0.2 m 판 → STOP)
  SafetyGate g3(SafetyParams{}, {}, 0.0);
  g3.setExclusionPolygon(poly, 0.0);
  EXPECT_TRUE(frame(g3, plate(0.2), 0.5, 0.0, 0.5).proximity_stop);
}

// ---------------------------------------------------------------- 깊이 점군

TEST(SafetyGate, LowObstacleSeenOnlyByDepthCloudStops)
{
  // 0.15 m 상자·지게차 포크는 LiDAR 평면(지면 0.20 m) 아래 → 깊이 점군만 본다
  SafetyGate g(SafetyParams{}, {}, 0.0);
  std::vector<Vec2> box;
  for (int i = 0; i < 7; ++i) {
    box.emplace_back(0.30 + 0.22, -0.15 + 0.05 * i);
  }
  g.setScan({}, kLidar, kInc, true, 0.0);
  g.setCloud(box, 0.0);
  auto st = drive(g, 0.3, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  EXPECT_EQ(st.zone, SafetyZone::kStop);
  EXPECT_TRUE(hasCause(st, "depth"));
  EXPECT_NEAR(st.cloud_distance, 0.22, 1e-12);
  EXPECT_TRUE(std::isinf(st.lidar_distance));
  // 영역 밖(옆) 점군은 무관
  SafetyGate g2(SafetyParams{}, {}, 0.0);
  std::vector<Vec2> beside;
  for (int i = 0; i < 7; ++i) {
    beside.emplace_back(0.3 + 0.05 * i, 0.40);
  }
  g2.setCloud(beside, 0.0);
  EXPECT_NEAR(drive(g2, 0.3, 0.0, 0.0).command.linear, 0.3, 1e-12);
  // 점 2 개(cloud_min_points 3 미만) = 잡음, 오래된 점군(> 0.3 s) 은 무시
  SafetyGate g3(SafetyParams{}, {}, 0.0);
  g3.setCloud({Vec2(0.5, 0.0), Vec2(0.5, 0.05)}, 0.0);
  EXPECT_NEAR(drive(g3, 0.3, 0.0, 0.0).command.linear, 0.3, 1e-12);
  g3.setCloud(box, 0.0);
  EXPECT_NEAR(drive(g3, 0.3, 0.0, 0.5).command.linear, 0.3, 1e-12);
}

TEST(SafetyGate, SensorLatencyIsCompensatedByMeasuredMotion)
{
  // 0.2 s 전에 찍힌 스캔(판 0.40 m)과 점군(상자 0.40 m):
  // 그동안 0.5 m/s 로 0.10 m 다가갔다 → 지금 0.30
  SafetyGate g(SafetyParams{}, {}, 0.0);
  for (double t = 0.0; t <= 0.2 + 1e-9; t += 0.02) {
    g.setMeasuredVelocity(0.5, 0.0, t);
  }
  g.setScan(plate(0.40), kLidar, kInc, false, 0.0);
  auto st = drive(g, 0.5, 0.0, 0.2);
  EXPECT_NEAR(st.lidar_distance, 0.30, 1e-9);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);  // 여유 거리 제한 v(0.30) = 0
  std::vector<Vec2> box;
  for (int i = 0; i < 5; ++i) {
    box.emplace_back(0.30 + 0.40, -0.1 + 0.05 * i);
  }
  g.setCloud(box, 0.0, 0.2);  // 촬영 0.0, 수신 0.2
  EXPECT_NEAR(drive(g, 0.5, 0.0, 0.2).cloud_distance, 0.30, 1e-9);
  // 끄면 촬영 당시 거리 그대로
  SafetyParams off;
  off.latency_compensation = false;
  SafetyGate g2(off, {}, 0.0);
  g2.setMeasuredVelocity(0.5, 0.0, 0.2);
  g2.setScan(plate(0.40), kLidar, kInc, false, 0.0);
  EXPECT_NEAR(drive(g2, 0.5, 0.0, 0.2).lidar_distance, 0.40, 1e-9);
}

TEST(SafetyGate, StaleRequiredDepthCloudLimitsForwardSpeed)
{
  // 깊이 점군이 필수인데 없거나 늦으면(촬영 후 0.4 s 초과) LiDAR 평면 아래를 못 본다 → 전진만 저속.
  // (Gazebo 부하 시험에서 점군 처리 지연으로 저상 상자에 닿은 사례의 재발 방지)
  SafetyParams p;
  p.cloud_required = true;
  SafetyGate g(p, {}, 0.0);
  auto st = drive(g, 1.0, 0.0, 0.0);  // 점군을 한 번도 받지 않음
  EXPECT_TRUE(st.cloud_stale);
  EXPECT_NEAR(st.command.linear, 0.2, 1e-12);
  EXPECT_TRUE(hasReason(st, "depth_cloud_stale"));
  EXPECT_NEAR(drive(g, -0.4, 0.0, 0.0).command.linear, -0.4, 1e-12);  // 후진은 카메라 밖
  g.setCloud({}, 0.10, 0.12);
  st = drive(g, 1.0, 0.0, 0.2);
  EXPECT_FALSE(st.cloud_stale);
  EXPECT_NEAR(st.command.linear, 1.0, 1e-12);
  g.setCloud({}, 0.10, 0.55);  // 촬영 후 0.45 s 에 도착 = 너무 늦다
  st = drive(g, 1.0, 0.0, 0.56);
  EXPECT_TRUE(st.cloud_stale);
  EXPECT_NEAR(st.command.linear, 0.2, 1e-12);
  EXPECT_FALSE(st.estop_active);
}

TEST(SafetyGate, ZoneHysteresis)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  // WARNING → 1.02 m (경계 + 히스테리시스 0.05 이내) 는 WARNING 유지, 1.06 에서 CLEAR
  EXPECT_EQ(frame(g, plate(0.8), 0.5, 0.0, 0.1).zone, SafetyZone::kWarning);
  EXPECT_EQ(frame(g, plate(1.02), 0.5, 0.0, 0.2).zone, SafetyZone::kWarning);
  EXPECT_EQ(frame(g, plate(1.06), 0.5, 0.0, 0.3).zone, SafetyZone::kClear);
  // CRITICAL → 0.52 유지, 0.56 에서 WARNING. 악화 방향은 즉시
  EXPECT_EQ(frame(g, plate(0.45), 0.5, 0.0, 0.4).zone, SafetyZone::kCritical);
  EXPECT_EQ(frame(g, plate(0.52), 0.5, 0.0, 0.5).zone, SafetyZone::kCritical);
  EXPECT_EQ(frame(g, plate(0.56), 0.5, 0.0, 0.6).zone, SafetyZone::kWarning);
  EXPECT_EQ(g.zone(), SafetyZone::kWarning);
  EXPECT_DOUBLE_EQ(g.params().emergency_stop_distance, 0.30);
}
