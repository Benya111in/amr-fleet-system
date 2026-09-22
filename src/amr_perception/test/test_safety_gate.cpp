// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 안전 게이트 단위 테스트: 존/속도 상한, 0.3 m 즉시 정지, 탈출, E-stop 래치, 센서 타임아웃,
// TTC 감속, 곡률 유지, 진행 방향 영역, 도킹 예외 다각형, 히스테리시스.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "amr_perception/safety_gate.hpp"

using amr_perception::SafetyGate;
using amr_perception::SafetyParams;
using amr_perception::SafetyStatus;
using amr_perception::SafetyZone;
using amr_perception::SensorFailureAction;
using amr_perception::SensorWatch;
using amr_perception::Vec2;
using amr_perception::ZoneRegion;

namespace
{
/// 전면 모서리(x = +0.30)에서 거리 d 인 점
Vec2 front(double d)
{
  return Vec2(0.30 + d, 0.0);
}

bool hasReason(const SafetyStatus & st, const std::string & r)
{
  return std::find(st.reasons.begin(), st.reasons.end(), r) != st.reasons.end();
}

/// 명령을 넣고 즉시 평가
SafetyStatus drive(SafetyGate & g, double v, double w, double t)
{
  g.setCommand(v, w, t);
  return g.evaluate(t);
}
}  // namespace

TEST(SafetyGate, ClearanceLimitAndZoneClassification)
{
  const SafetyParams p;
  // robot_params.yaml 주석의 수치: D = 2.6 → 2.0, 1.0 → 1.04, 0.5 → 0.50
  EXPECT_NEAR(SafetyGate::clearanceSpeedLimit(2.6, p), 2.0, 1e-9);
  EXPECT_NEAR(SafetyGate::clearanceSpeedLimit(1.0, p), 1.0427, 1e-3);
  EXPECT_NEAR(SafetyGate::clearanceSpeedLimit(0.5, p), 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(SafetyGate::clearanceSpeedLimit(0.2, p), 0.0);
  EXPECT_TRUE(std::isinf(SafetyGate::clearanceSpeedLimit(INFINITY, p)));
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
}

TEST(SafetyGate, ClearWarningCriticalSpeedCaps)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  // CLEAR: 여유 1.5 m → 연속 상한 -0.15 + √(0.0225 + 2·1.2) = 1.4065
  g.setScanPoints({front(1.5)}, 0.0);
  auto st = drive(g, 2.0, 0.0, 0.0);
  EXPECT_EQ(st.zone, SafetyZone::kClear);
  EXPECT_NEAR(st.command.linear, -0.15 + std::sqrt(0.0225 + 2.4), 1e-9);
  EXPECT_FALSE(st.estop_active);
  // WARNING (0.8 m): 0.5 m/s 상한
  g.setScanPoints({front(0.8)}, 0.1);
  st = drive(g, 1.0, 0.0, 0.1);
  EXPECT_EQ(st.zone, SafetyZone::kWarning);
  EXPECT_NEAR(st.command.linear, 0.5, 1e-12);
  EXPECT_TRUE(hasReason(st, "warning_zone"));
  // CRITICAL (0.4 m): 0.2 m/s 상한
  g.setScanPoints({front(0.4)}, 0.2);
  st = drive(g, 1.0, 0.0, 0.2);
  EXPECT_EQ(st.zone, SafetyZone::kCritical);
  EXPECT_NEAR(st.command.linear, 0.2, 1e-12);
  EXPECT_TRUE(hasReason(st, "critical_zone"));
  EXPECT_FALSE(st.estop_active);
  // 빈 스캔: 제한 없음 (속도 한계로만 자름)
  g.setScanPoints({}, 0.3);
  st = drive(g, 3.0, 2.0, 0.3);
  EXPECT_NEAR(st.command.linear, 2.0, 1e-12);   // limits.max_linear_velocity 로 자름
  EXPECT_NEAR(st.command.angular, 1.5, 1e-12);  // limits.max_angular_velocity 로 자름
}

TEST(SafetyGate, ApproachingObstacleStopsWithinOneCycle)
{
  // 50 Hz 주기로 장애물이 1.0 m/s 로 다가온다. D <= 0.30 인 첫 주기에 출력이 0 이어야 한다.
  SafetyGate g(SafetyParams{}, {}, 0.0);
  const double dt = 0.02;
  bool stopped = false;
  int stop_cycle = -1;
  int first_inside = -1;
  for (int i = 0; i < 100 && !stopped; ++i) {
    const double t = i * dt;
    const double d = 1.21 - 1.0 * t;  // 경계값(정확히 0.30)을 피한 격자: 0.31 → 0.29
    g.setScanPoints({front(d)}, t);
    const auto st = drive(g, 0.5, 0.0, t);
    if (d <= 0.30 && first_inside < 0) {
      first_inside = i;
    }
    if (st.command.linear == 0.0 && st.command.angular == 0.0) {
      stopped = true;
      stop_cycle = i;
      EXPECT_EQ(st.zone, SafetyZone::kStop);
      EXPECT_TRUE(st.estop_active);
      EXPECT_TRUE(st.proximity_stop);
      EXPECT_TRUE(hasReason(st, "proximity_stop"));
    } else {
      EXPECT_GT(st.min_distance, 0.30);
    }
  }
  ASSERT_TRUE(stopped);
  EXPECT_EQ(stop_cycle, first_inside);
  // 장애물이 물러나도 0.5 m (stop_release_distance) 를 넘기 전에는 정지 유지 (히스테리시스)
  g.setScanPoints({front(0.45)}, 3.0);
  EXPECT_DOUBLE_EQ(drive(g, 0.5, 0.0, 3.0).command.linear, 0.0);
  g.setScanPoints({front(0.55)}, 3.02);
  const auto released = drive(g, 0.5, 0.0, 3.02);
  EXPECT_FALSE(released.proximity_stop);
  EXPECT_FALSE(released.estop_active);
  EXPECT_NEAR(released.command.linear, 0.5, 1e-12);  // WARNING 상한 0.5
}

TEST(SafetyGate, EscapeAllowedOnlyWhenMovingAway)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  g.setScanPoints({front(0.25)}, 0.0);
  auto st = drive(g, 0.3, 0.0, 0.0);  // 다가가는 명령 → 정지
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  EXPECT_FALSE(st.escaping);
  st = drive(g, -0.4, 0.0, 0.02);  // 후진 = 멀어짐 → 탈출 허용 (CRITICAL 상한 0.2)
  EXPECT_TRUE(st.escaping);
  EXPECT_NEAR(st.command.linear, -0.2, 1e-12);
  EXPECT_TRUE(st.estop_active);  // 탈출 중에도 정지 상태 표시는 유지
  SafetyParams no_escape;
  no_escape.allow_escape = false;
  SafetyGate g2(no_escape, {}, 0.0);
  g2.setScanPoints({front(0.25)}, 0.0);
  EXPECT_DOUBLE_EQ(drive(g2, -0.4, 0.0, 0.0).command.linear, 0.0);
}

TEST(SafetyGate, EstopLatchReleasesOnlyOnExplicitFalseAndReset)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  g.setEstopSource("estop", true, 0.0);
  auto st = drive(g, 0.5, 0.0, 0.0);
  EXPECT_TRUE(st.estop_latched);
  EXPECT_TRUE(st.estop_active);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  EXPECT_TRUE(hasReason(st, "estop_latched"));
  // false 만으로는 해제되지 않는다 (reset 필요)
  g.setEstopSource("estop", false, 0.1);
  EXPECT_TRUE(drive(g, 0.5, 0.0, 0.1).estop_latched);
  // reset → 해제
  const auto r = g.requestReset(0.2);
  EXPECT_TRUE(r.success);
  st = drive(g, 0.5, 0.0, 0.2);
  EXPECT_FALSE(st.estop_latched);
  EXPECT_NEAR(st.command.linear, 0.5, 1e-12);
  // 래치 없을 때 reset 은 성공 (무동작)
  EXPECT_TRUE(g.requestReset(0.25).success);
}

TEST(SafetyGate, EstopResetBeforeFalseWaitsForGrace)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  g.setEstopSource("fleet_estop", true, 0.0);
  // 대시보드: false 발행 직후 reset — 서비스가 먼저 도착
  const auto r = g.requestReset(1.0);
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.message.find("fleet_estop"), std::string::npos);
  g.setEstopSource("fleet_estop", false, 1.3);  // grace(1.0 s) 안
  EXPECT_FALSE(g.estopLatched());
  // grace 가 지나 도착한 false 로는 해제되지 않는다
  g.setEstopSource("fleet_estop", true, 2.0);
  EXPECT_FALSE(g.requestReset(2.0).success);
  g.setEstopSource("fleet_estop", false, 3.5);
  EXPECT_TRUE(g.estopLatched());
  EXPECT_TRUE(g.requestReset(3.6).success);
  EXPECT_FALSE(g.estopLatched());
  // 두 입력 중 하나라도 true 면 해제 불가, reset 대기 중 새 true 는 대기 무효화
  g.setEstopSource("estop", true, 4.0);
  g.setEstopSource("fleet_estop", true, 4.0);
  g.setEstopSource("estop", false, 4.1);
  EXPECT_FALSE(g.requestReset(4.2).success);
  g.setEstopSource("estop", true, 4.3);   // 새 E-stop → 대기 무효
  g.setEstopSource("estop", false, 4.4);
  g.setEstopSource("fleet_estop", false, 4.5);
  EXPECT_TRUE(g.estopLatched());
}

TEST(SafetyGate, EstopWithoutResetRequirement)
{
  SafetyParams p;
  p.estop_release_requires_reset = false;
  SafetyGate g(p, {}, 0.0);
  g.setEstopSource("estop", true, 0.0);
  EXPECT_TRUE(g.estopLatched());
  g.setEstopSource("estop", false, 0.1);
  EXPECT_FALSE(g.estopLatched());
}

TEST(SafetyGate, SensorTimeoutsStopOrDegrade)
{
  std::vector<SensorWatch> watches = {
    {"lidar", 0.3, SensorFailureAction::kStop},
    {"imu", 0.05, SensorFailureAction::kDegraded},
  };
  SafetyGate g(SafetyParams{}, watches, 0.0);
  // 기동 유예: 시작 시각에 본 것으로 간주
  auto st = drive(g, 1.0, 0.0, 0.02);
  EXPECT_TRUE(st.failed_sensors.empty());
  EXPECT_NEAR(st.command.linear, 1.0, 1e-12);
  g.sensorHeartbeat("lidar", 1.0);
  g.sensorHeartbeat("imu", 1.0);
  g.sensorHeartbeat("unknown", 1.0);  // 감시 대상 아님 → 무시
  st = drive(g, 1.0, 0.0, 1.04);
  EXPECT_TRUE(st.failed_sensors.empty());
  // IMU 타임아웃 → 저속 0.2
  st = drive(g, 1.0, 0.0, 1.1);
  EXPECT_TRUE(st.degraded);
  EXPECT_FALSE(st.sensor_stop);
  EXPECT_FALSE(st.estop_active);
  EXPECT_NEAR(st.command.linear, 0.2, 1e-12);
  ASSERT_EQ(st.failed_sensors.size(), 1U);
  EXPECT_EQ(st.failed_sensors.front(), "imu");
  EXPECT_TRUE(hasReason(st, "sensor_failure_degraded"));
  // LiDAR 타임아웃 → 정지 + estop_active
  st = drive(g, 1.0, 0.0, 1.35);
  EXPECT_TRUE(st.sensor_stop);
  EXPECT_TRUE(st.estop_active);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  EXPECT_TRUE(hasReason(st, "sensor_failure_stop"));
  // 복구
  g.sensorHeartbeat("lidar", 1.4);
  g.sensorHeartbeat("imu", 1.4);
  st = drive(g, 1.0, 0.0, 1.41);
  EXPECT_FALSE(st.sensor_stop);
  EXPECT_FALSE(st.degraded);
  EXPECT_NEAR(st.command.linear, 1.0, 1e-12);
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
  g.setScanPoints({front(0.8)}, 1.1);
  g.setMinTtc(1.0, 1.1);
  st = drive(g, 1.5, 0.0, 1.1);
  EXPECT_NEAR(st.command.linear, 0.5, 1e-12);
  EXPECT_FALSE(hasReason(st, "ttc_limit"));
  // 비활성
  SafetyParams off;
  off.ttc_limit_enabled = false;
  SafetyGate g2(off, {}, 0.0);
  g2.setMinTtc(0.3, 0.0);
  EXPECT_NEAR(drive(g2, 1.0, 0.0, 0.0).command.linear, 1.0, 1e-12);
}

TEST(SafetyGate, AngularLimitPreservesCurvature)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  g.setScanPoints({front(0.8)}, 0.0);  // WARNING: v 0.5, ω·r_circ 0.5
  auto st = drive(g, 1.0, 1.5, 0.0);
  EXPECT_NEAR(st.command.linear, 0.5, 1e-12);
  EXPECT_NEAR(st.command.angular / st.command.linear, 1.5, 1e-9);  // 곡률 유지
  // 제자리 회전: CRITICAL 에서 꼭짓점 속도 0.2 → ω <= 0.2 / r_circ
  g.setScanPoints({front(0.4)}, 0.1);
  st = drive(g, 0.0, 1.0, 0.1);
  EXPECT_NEAR(st.command.angular, 0.2 / g.circumscribedRadius(), 1e-9);
  EXPECT_NEAR(st.angular_limit, 0.2 / g.circumscribedRadius(), 1e-9);
}

TEST(SafetyGate, MotionRegionIgnoresSideObjectsUnlessTouching)
{
  SafetyParams p;
  p.zone_region = ZoneRegion::kMotion;
  SafetyGate g(p, {}, 0.0);
  // 전진 중 측면 0.1 m 점 → 존 판정 제외 (lateral_stop_distance 0.05 만 적용)
  g.setScanPoints({Vec2(0.0, 0.30)}, 0.0);
  auto st = drive(g, 0.5, 0.0, 0.0);
  EXPECT_EQ(st.zone, SafetyZone::kClear);
  EXPECT_NEAR(st.command.linear, 0.5, 1e-12);
  // 측면 0.04 m → 정지
  g.setScanPoints({Vec2(0.0, 0.24)}, 0.02);
  st = drive(g, 0.5, 0.0, 0.02);
  EXPECT_TRUE(st.proximity_stop);
  EXPECT_DOUBLE_EQ(st.command.linear, 0.0);
  // 해제 후 후진 중에는 뒤쪽 점이 존 판정 대상
  g.setScanPoints({Vec2(-0.3 - 0.8, 0.0)}, 0.04);
  st = drive(g, -0.4, 0.0, 0.04);
  EXPECT_EQ(st.zone, SafetyZone::kWarning);
  EXPECT_NEAR(st.command.linear, -0.4, 1e-12);  // 후진 한계 -0.5, WARNING 0.5 → 그대로
  // 제자리 회전: 외접원이 쓸고 지나가는 거리 (0.6 - 0.3606 = 0.239 <= 0.3) → 정지
  g.setScanPoints({Vec2(0.0, 0.6)}, 0.06);
  st = drive(g, 0.0, 1.0, 0.06);
  EXPECT_TRUE(st.proximity_stop);
  // 정지 명령: 모서리 거리로 판정 (0.6 > 해제 거리 0.5 → 정지 해제, WARNING)
  g.setScanPoints({Vec2(0.0, 0.8)}, 0.08);
  st = drive(g, 0.0, 0.0, 0.08);
  EXPECT_FALSE(st.proximity_stop);
  EXPECT_EQ(st.zone, SafetyZone::kWarning);
}

TEST(SafetyGate, DockExclusionPolygonAllowsCloseApproach)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  // 전방 도킹 판 영역 (x 0.35 ~ 1.0) 을 예외로
  const std::vector<Vec2> poly = {
    Vec2(0.35, -0.5), Vec2(1.0, -0.5), Vec2(1.0, 0.5), Vec2(0.35, 0.5)};
  g.setExclusionPolygon(poly, 0.0);
  g.setScanPoints({front(0.2)}, 0.0);  // 평소라면 0.3 m 이내 정지
  auto st = drive(g, 0.5, 0.0, 0.0);
  EXPECT_FALSE(st.proximity_stop);
  EXPECT_NEAR(st.command.linear, 0.2, 1e-12);  // 예외 영역 점이 있으면 CRITICAL 상한
  // 예외 영역 안이라도 0.10 m 이내면 정지
  g.setExclusionPolygon(poly, 0.1);
  g.setScanPoints({front(0.08)}, 0.1);
  EXPECT_TRUE(drive(g, 0.1, 0.0, 0.1).proximity_stop);
  // 다각형 만료 (0.3 s) → 일반 규칙
  SafetyGate g2(SafetyParams{}, {}, 0.0);
  g2.setExclusionPolygon(poly, 0.0);
  g2.setScanPoints({front(0.2)}, 0.5);
  EXPECT_TRUE(drive(g2, 0.5, 0.0, 0.5).proximity_stop);
}

TEST(SafetyGate, SelfReflectionsIgnoredAndZoneHysteresis)
{
  SafetyGate g(SafetyParams{}, {}, 0.0);
  g.setScanPoints({Vec2(0.1, 0.0), Vec2(-0.2, 0.1)}, 0.0);  // 풋프린트 안쪽 = 자기 차체
  auto st = drive(g, 0.5, 0.0, 0.0);
  EXPECT_EQ(st.zone, SafetyZone::kClear);
  EXPECT_TRUE(std::isinf(st.min_distance));
  // WARNING → 1.02 m (경계 + 히스테리시스 0.05 이내) 는 WARNING 유지, 1.06 에서 CLEAR
  g.setScanPoints({front(0.8)}, 0.1);
  EXPECT_EQ(drive(g, 0.5, 0.0, 0.1).zone, SafetyZone::kWarning);
  g.setScanPoints({front(1.02)}, 0.2);
  EXPECT_EQ(drive(g, 0.5, 0.0, 0.2).zone, SafetyZone::kWarning);
  g.setScanPoints({front(1.06)}, 0.3);
  EXPECT_EQ(drive(g, 0.5, 0.0, 0.3).zone, SafetyZone::kClear);
  // CRITICAL → 0.52 유지, 0.56 에서 WARNING. 악화 방향은 즉시
  g.setScanPoints({front(0.45)}, 0.4);
  EXPECT_EQ(drive(g, 0.5, 0.0, 0.4).zone, SafetyZone::kCritical);
  g.setScanPoints({front(0.52)}, 0.5);
  EXPECT_EQ(drive(g, 0.5, 0.0, 0.5).zone, SafetyZone::kCritical);
  g.setScanPoints({front(0.56)}, 0.6);
  EXPECT_EQ(drive(g, 0.5, 0.0, 0.6).zone, SafetyZone::kWarning);
  EXPECT_EQ(g.zone(), SafetyZone::kWarning);
  EXPECT_DOUBLE_EQ(g.params().emergency_stop_distance, 0.30);
}
