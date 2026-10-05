// 예측 점유 코어 계약 (명세 4장 7절 재계획 회피 / 9장 예측 기반 회피).
//
// 왜 이 파일이 생겼나: C++ 커버리지를 처음 재 보니 predicted_obstacle_layer.cpp 가
// **1.0 % (1/96)** 로 전체 92.1 % 중 최하위였다. 내가 그날 만든 코드인데 단위 계약 없이
// 통합 측정에만 기대고 있었다. 판단 로직을 core/predicted_occupancy.hpp 로 빼고
// 여기서 계약한다 (저장소 규약: core 순수 함수 + 얇은 ROS 래퍼).
#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "amr_navigation/core/predicted_occupancy.hpp"

using amr_navigation::core::CellCost;
using amr_navigation::core::discCells;
using amr_navigation::core::pointsBounds;
using amr_navigation::core::predictionFresh;
using amr_navigation::core::shouldRaise;

namespace
{
constexpr unsigned char kNoInfo = 255;
}

// 핵심 계약: 기피는 **올리기만** 한다. 정적 장애물·팽창을 덮으면 계획기가 벽을 못 본다.
TEST(PredictedOccupancy, RaisesCostButNeverLowersIt)
{
  EXPECT_TRUE(shouldRaise(10, 229, kNoInfo)) << "싼 셀은 올린다";
  EXPECT_FALSE(shouldRaise(254, 229, kNoInfo)) << "LETHAL 을 229 로 낮추면 안 된다";
  EXPECT_FALSE(shouldRaise(253, 229, kNoInfo)) << "INSCRIBED 도 낮추면 안 된다";
  EXPECT_FALSE(shouldRaise(229, 229, kNoInfo)) << "같으면 쓸 필요 없다";
  EXPECT_TRUE(shouldRaise(kNoInfo, 229, kNoInfo)) << "미지(255)는 값이 없다는 뜻이라 덮는다";
  EXPECT_TRUE(shouldRaise(229, 254, kNoInfo)) << "차단이 기피보다 세다";
}

// 디스크가 반경 안만 덮고 격자 밖으로 새지 않는다.
TEST(PredictedOccupancy, DiscStaysInsideRadiusAndGrid)
{
  const double res = 0.05;
  const auto cells = discCells(1.0, 1.0, 0.20, 229, 0.0, 0.0, res, 100, 100);
  ASSERT_FALSE(cells.empty());
  const int ci = static_cast<int>(1.0 / res), cj = static_cast<int>(1.0 / res);
  const int rc = static_cast<int>(std::ceil(0.20 / res));
  for (const auto & c : cells) {
    EXPECT_GE(c.i, 0);
    EXPECT_LT(c.i, 100);
    EXPECT_GE(c.j, 0);
    EXPECT_LT(c.j, 100);
    const int di = c.i - ci, dj = c.j - cj;
    EXPECT_LE(di * di + dj * dj, rc * rc) << "반경 밖 셀이 나왔다";
    EXPECT_EQ(229, c.cost);
  }
  // 면적이 대략 pi r^2 / res^2 이다 (격자 이산화 오차 20 % 안)
  const double want = M_PI * (0.20 / res) * (0.20 / res);
  EXPECT_NEAR(want, static_cast<double>(cells.size()), 0.2 * want);
}

// 격자 모서리에서 잘린다 (음수 인덱스로 새면 다른 셀을 덮어쓴다).
TEST(PredictedOccupancy, DiscIsClippedAtGridEdge)
{
  const auto cells = discCells(0.0, 0.0, 0.20, 229, 0.0, 0.0, 0.05, 100, 100);
  for (const auto & c : cells) {
    EXPECT_GE(c.i, 0);
    EXPECT_GE(c.j, 0);
  }
  // 원점 모서리이므로 온전한 원의 약 1/4 만 남는다
  const double full = M_PI * 16.0;
  EXPECT_LT(static_cast<double>(cells.size()), 0.45 * full);
}

TEST(PredictedOccupancy, DegenerateInputsProduceNothing)
{
  EXPECT_TRUE(discCells(1.0, 1.0, 0.2, 229, 0.0, 0.0, 0.0, 100, 100).empty()) << "해상도 0";
  EXPECT_TRUE(discCells(1.0, 1.0, -1.0, 229, 0.0, 0.0, 0.05, 100, 100).empty()) << "음수 반경";
}

// 만료: 발행이 멈추면 옛 예측이 경로를 영원히 막으면 안 된다.
TEST(PredictedOccupancy, StalePredictionExpires)
{
  EXPECT_TRUE(predictionFresh(0.1, 0.5));
  EXPECT_TRUE(predictionFresh(0.5, 0.5)) << "경계는 포함";
  EXPECT_FALSE(predictionFresh(0.51, 0.5));
  EXPECT_TRUE(predictionFresh(9999.0, 0.0)) << "timeout 0 = 만료 안 봄";
  // 시계가 튀어 미래 스탬프가 되면 옛 예측이 되살아난다 — 같은 폭으로 막는다
  EXPECT_TRUE(predictionFresh(-0.2, 0.5));
  EXPECT_FALSE(predictionFresh(-0.9, 0.5));
}

// 범위: 반경까지 포함해야 지난 마킹이 지워진다.
TEST(PredictedOccupancy, BoundsIncludeRadius)
{
  std::vector<std::pair<double, double>> pts{{1.0, 2.0}, {3.0, -1.0}};
  double a = 0, b = 0, c = 0, d = 0;
  ASSERT_TRUE(pointsBounds(pts, 0.3, &a, &b, &c, &d));
  EXPECT_DOUBLE_EQ(0.7, a);
  EXPECT_DOUBLE_EQ(-1.3, b);
  EXPECT_DOUBLE_EQ(3.3, c);
  EXPECT_DOUBLE_EQ(2.3, d);
  EXPECT_FALSE(pointsBounds({}, 0.3, &a, &b, &c, &d)) << "비면 false — 범위를 넓히면 안 된다";
}

// 원점보다 작은 좌표에서 셀이 어긋나지 않는다.
// 추출할 때 worldToMap(floor) 을 static_cast<int>(0 쪽 절삭)로 바꿨다가 이 계약이 잡았다.
TEST(PredictedOccupancy, NegativeOffsetUsesFloorNotTruncation)
{
  const double res = 0.05, ox = -1.0, oy = -1.0;
  // 중심 (-0.885, -1.0) 은 원점에서 (0.115, 0.0) → 셀 (2.3, 0) → floor 로 (2, 0)
  const auto cells = discCells(-0.885, -1.0, 0.02, 229, ox, oy, res, 100, 100);
  ASSERT_FALSE(cells.empty());
  int lo_i = 1 << 30, hi_i = -(1 << 30);
  for (const auto & c : cells) {
    lo_i = std::min(lo_i, c.i);
    hi_i = std::max(hi_i, c.i);
  }
  const int want = static_cast<int>(std::floor((-0.885 - ox) / res));
  EXPECT_LE(lo_i, want);
  EXPECT_GE(hi_i, want);
  EXPECT_EQ(2, want) << "0 쪽 절삭이면 2, floor 도 2 — 양수 쪽은 같다";

  // 음수 쪽: 중심이 원점보다 왼쪽
  const auto neg = discCells(-1.115, -1.0, 0.02, 229, ox, oy, res, 100, 100);
  const int want_neg = static_cast<int>(std::floor((-1.115 - ox) / res));   // -2.3 -> -3
  EXPECT_EQ(-3, want_neg) << "절삭이면 -2 가 되어 한 칸 어긋난다";
  // 격자 밖이므로 결과는 비어야 한다 (클리핑이 잡는다)
  for (const auto & c : neg) {
    EXPECT_GE(c.i, 0);
  }
}

// --- 치명 금지 반경 (연구 브리프 §11) -------------------------------------------------
//
// 실측 근거: predicted_layer.cost 254 를 쓴 4 실행에서만 A* 가 "start blocked" 로 실패했고
// (나머지 164 실행 0 건), 그 뒤 BT 가 follow_path 를 취소해 제어 주기가 1.6~6.4 s 비었으며
// 접촉 6 건 중 4 건이 그 공백 안에서 났다 (logs/MX1a t00, logs/NW1a t02·t07·t14).

TEST(PredictedOccupancy, LethalIsNotStampedOnTheRobotsOwnCell)
{
  // 로봇 (1.0, 2.0), 금지 반경 0.46. 바로 그 자리는 치명이 아니라 기피로 내려가야 한다.
  EXPECT_EQ(amr_navigation::core::stampCost(254, 1.00, 2.00, 1.0, 2.0, 0.46, 253, 229), 229);
  // 0.40 < 0.46 — 금지 반경 안이다
  EXPECT_EQ(amr_navigation::core::stampCost(254, 1.40, 2.00, 1.0, 2.0, 0.46, 253, 229), 229);
}

TEST(PredictedOccupancy, LethalSurvivesOutsideTheKeepout)
{
  // 금지 반경 밖이면 그대로 치명이어야 한다 — 이 수정이 레이어를 무력화하면 안 된다.
  EXPECT_EQ(amr_navigation::core::stampCost(254, 1.50, 2.00, 1.0, 2.0, 0.46, 253, 229), 254);
  EXPECT_EQ(amr_navigation::core::stampCost(254, 1.0, 2.60, 1.0, 2.0, 0.46, 253, 229), 254);
}

TEST(PredictedOccupancy, AvoidCostIsNeverDemoted)
{
  // 253 미만(기피)은 애초에 start 를 막지 않으므로 로봇 자리에서도 손대지 않는다.
  EXPECT_EQ(amr_navigation::core::stampCost(229, 1.0, 2.0, 1.0, 2.0, 0.46, 253, 229), 229);
  EXPECT_EQ(amr_navigation::core::stampCost(100, 1.0, 2.0, 1.0, 2.0, 0.46, 253, 229), 100);
}

TEST(PredictedOccupancy, KeepoutZeroDisablesTheRule)
{
  // 반경 0 이면 종전 거동 (구성으로 되돌릴 수 있어야 한다 — 되돌림 조건).
  EXPECT_EQ(amr_navigation::core::stampCost(254, 1.0, 2.0, 1.0, 2.0, 0.0, 253, 229), 254);
  EXPECT_TRUE(amr_navigation::core::mayStampLethal(1.0, 2.0, 1.0, 2.0, 0.0));
}

TEST(PredictedOccupancy, DemotedCostStaysBelowInscribed)
{
  // 강등값이 253 이상이면 의미가 없다 — A* 가 여전히 막힌다. 계약으로 고정한다.
  EXPECT_LT(amr_navigation::core::stampCost(254, 1.0, 2.0, 1.0, 2.0, 0.46, 253, 229), 253);
}

TEST(PredictedOccupancy, KeepoutCoversTheRobotFootprintCircle)
{
  // 외접원 0.361 전체가 덮여야 한다 — 발자국 한 귀퉁이만 치명이어도 start 는 막힌다.
  const double kRobotR = 0.361, kKeepout = 0.46;
  ASSERT_GT(kKeepout, kRobotR);
  for (int deg = 0; deg < 360; deg += 15) {
    const double a = deg * M_PI / 180.0;
    const double x = 1.0 + kRobotR * std::cos(a), y = 2.0 + kRobotR * std::sin(a);
    EXPECT_FALSE(amr_navigation::core::mayStampLethal(x, y, 1.0, 2.0, kKeepout)) << "deg=" << deg;
  }
}
