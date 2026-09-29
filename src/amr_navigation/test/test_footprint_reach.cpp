// 발자국 도달거리 계약.
//
// 접촉 판정은 로봇 직사각형 0.60 × 0.40 과 장애물 원의 부호 거리인데, DWA 는 동적 장애물에 대해
// 자기를 외접원(0.361)으로만 본다. 외접원은 회전 불변이라 방위가 비용에 나타나지 않는다.
// 실측: 접촉 132 건의 침투가 중앙 6.0 mm · 최대 18.8 mm 인데 도달거리는 0.200~0.3606 m 로
// 변하므로, 방위가 균등하다면 84.8 % 가 최적 방위에서 회피된다.
#include <cmath>

#include "amr_navigation/core/footprint_reach.hpp"
#include <gtest/gtest.h>

using amr_navigation::core::footprintReach;
using amr_navigation::core::footprintReachMax;
using amr_navigation::core::footprintReachMin;
using amr_navigation::core::footprintReachTo;

namespace
{
constexpr double kHl = 0.30;   // config/robot_params.yaml footprint_length 0.60 / 2
constexpr double kHw = 0.20;   // footprint_width 0.40 / 2
constexpr double kPi = 3.14159265358979323846;
}  // namespace

// 정면·후방은 반길이, 정횡은 반폭. 이것이 방위 의존성의 전부다.
TEST(FootprintReach, AxisAlignedBearingsGiveHalfExtents)
{
  EXPECT_DOUBLE_EQ(kHl, footprintReach(kHl, kHw, 0.0));
  EXPECT_DOUBLE_EQ(kHl, footprintReach(kHl, kHw, kPi));
  EXPECT_DOUBLE_EQ(kHw, footprintReach(kHl, kHw, kPi / 2));
  EXPECT_DOUBLE_EQ(kHw, footprintReach(kHl, kHw, -kPi / 2));
}

// 모서리 방위에서 외접원과 같다 — 이것이 지금 DWA 가 **모든** 방위에 쓰는 값이다.
TEST(FootprintReach, CornerBearingEqualsCircumscribedRadius)
{
  const double corner = std::atan2(kHw, kHl);
  EXPECT_NEAR(std::hypot(kHl, kHw), footprintReach(kHl, kHw, corner), 1e-12);
  EXPECT_NEAR(0.3605551275463989, footprintReachMax(kHl, kHw), 1e-12);
}

// 도달거리는 항상 [반폭, 외접원] 안에 있다. 외접원 하나로 대신하면 최대 0.1606 m 를 낭비한다.
TEST(FootprintReach, AlwaysBetweenMinAndMax)
{
  for (int i = 0; i < 720; ++i) {
    const double th = -kPi + 2 * kPi * i / 720.0;
    const double r = footprintReach(kHl, kHw, th);
    EXPECT_GE(r, footprintReachMin(kHl, kHw) - 1e-12) << "th=" << th;
    EXPECT_LE(r, footprintReachMax(kHl, kHw) + 1e-12) << "th=" << th;
  }
  EXPECT_DOUBLE_EQ(kHw, footprintReachMin(kHl, kHw));
}

TEST(FootprintReach, IsSymmetricInAllFourQuadrants)
{
  for (int i = 0; i <= 90; ++i) {
    const double th = kPi / 2 * i / 90.0;
    const double r = footprintReach(kHl, kHw, th);
    EXPECT_NEAR(r, footprintReach(kHl, kHw, -th), 1e-12);
    EXPECT_NEAR(r, footprintReach(kHl, kHw, kPi - th), 1e-12);
    EXPECT_NEAR(r, footprintReach(kHl, kHw, th - kPi), 1e-12);
  }
}

// 핵심 계약: 관측된 최악 침투(18.8 mm)를 회전으로 덮을 수 있는 방위 비율이 실측 계산과 맞는다.
// 구제 불가능한 것은 정횡 ±24° 뿐이다.
TEST(FootprintReach, RotationRescuesMeasuredPenetrationsExceptNearBroadside)
{
  const double worst_pen = 0.0188;          // 접촉 132 건의 최대 침투
  const double min_reach = footprintReachMin(kHl, kHw);
  int ok = 0;
  const int n = 3600;
  double lo = 999.0, hi = -999.0;
  for (int i = 0; i < n; ++i) {
    const double th = 2 * kPi * i / n;
    const double gain = footprintReach(kHl, kHw, th) - min_reach;
    if (gain > worst_pen) {
      ++ok;
    } else {
      const double deg = std::abs(std::fmod(th * 180.0 / kPi, 180.0));
      lo = std::min(lo, deg);
      hi = std::max(hi, deg);
    }
  }
  EXPECT_NEAR(0.734, static_cast<double>(ok) / n, 0.01)
    << "최악 침투 기준 구제 방위 비율 (실측 계산 73.4 %)";
  EXPECT_NEAR(66.0, lo, 1.5) << "구제 불가 방위대 하한";
  EXPECT_NEAR(114.0, hi, 1.5) << "구제 불가 방위대 상한";
}

// Y8a 실측 접촉 1 건을 그대로 재현한다 (상대 방위 48.3°, 도달 0.268 m, 이득 67.7 mm > 침투 17.8 mm).
TEST(FootprintReach, ReproducesMeasuredContactY8aTrial0)
{
  const double rel = 0.844;                 // contacts.csv obstacle_bearing_rel_rad
  const double r = footprintReach(kHl, kHw, rel);
  EXPECT_NEAR(0.268, r, 1e-3);
  EXPECT_GT(r - footprintReachMin(kHl, kHw), 0.0178) << "최적 방위였다면 회피했어야 한다";
}

// 세계 좌표 판은 로봇 자세를 빼고 같은 값을 준다.
TEST(FootprintReach, WorldFrameMatchesRelativeBearing)
{
  const double yaw = -1.078;                // Y8a 실측 로봇 자세
  const double rel = 0.844;
  const double rx = 1.0, ry = 2.0, d = 3.0;
  const double ox = rx + d * std::cos(yaw + rel);
  const double oy = ry + d * std::sin(yaw + rel);
  EXPECT_NEAR(footprintReach(kHl, kHw, rel),
              footprintReachTo(kHl, kHw, rx, ry, yaw, ox, oy), 1e-9);
}

// 방어: 잘못된 치수는 0 (호출부가 외접원으로 되돌아가도록).
TEST(FootprintReach, NonPositiveExtentsReturnZero)
{
  EXPECT_DOUBLE_EQ(0.0, footprintReach(0.0, kHw, 0.3));
  EXPECT_DOUBLE_EQ(0.0, footprintReach(kHl, -1.0, 0.3));
}
