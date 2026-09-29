// 트랙 속도 평활 계약.
//
// 배경: `dwa_controller.cpp` 의 인라인 갱신이 "같은 메시지 재관측" 과 "시간 역행" 을 한 조건
// (`gap <= 0.0`) 으로 묶고 있었다. `obstaclesInFrame` 은 매 제어 주기(20 Hz) 호출되면서 캐시된
// 최신 트랙 메시지를 쓰므로, 추적기가 더 느리면 매 다른 틱마다 gap == 0 → 리셋이 되어 τ 평활이
// 사실상 꺼져 있었다. 이 평활값은 predVx()/predVy() 로 crossing_yield 의 통로 구성에 들어가므로
// committed 판정의 입력이고, 08 접촉 130 건 중 110 건이 committed 상태였다.
//
// 아래 `LegacyFilter` 는 **고치기 전 의미론을 그대로 옮긴 참조**다. 계약이 실제로 그 결함을
// 잡아내는지 보이기 위해 남긴다 (계약이 처음부터 통과하면 결함이 없는 것이므로).
#include <cmath>

#include "amr_navigation/core/track_velocity_filter.hpp"
#include <gtest/gtest.h>

using amr_navigation::core::FilterStep;
using amr_navigation::core::TrackVelocityFilter;

namespace
{
/// 고치기 전 dwa_controller.cpp:346-352 의 의미론 (gap <= 0.0 을 불연속으로 취급).
struct LegacyFilter
{
  double vx{0.0};
  double vy{0.0};
  double stamp{-1.0};
  void update(double vx_raw, double vy_raw, double stamp_s, double tau, double timeout)
  {
    const double gap = stamp > 0.0 ? stamp_s - stamp : 0.0;
    if (stamp <= 0.0 || gap <= 0.0 || gap > timeout) {
      vx = vx_raw;
      vy = vy_raw;
    } else {
      const double a = 1.0 - std::exp(-gap / tau);
      vx += a * (vx_raw - vx);
      vy += a * (vy_raw - vy);
    }
    stamp = stamp_s;
  }
};

constexpr double kTau = 0.5;        // nav2_params.yaml yield_velocity_tau
constexpr double kTimeout = 2.0;    // track_timeout
}  // namespace

// 같은 트랙 메시지를 두 번 보면 출력이 **정확히 같아야** 한다 — 새 정보가 없기 때문이다.
// 이것이 핵심 계약이다. 제어 주기(20 Hz)가 추적 주기(10 Hz)보다 빠른 한 반드시 일어난다.
TEST(TrackVelocityFilter, SameStampIsIdempotent)
{
  TrackVelocityFilter f;
  f.update(1.0, 0.0, 10.0, kTau, kTimeout);            // 첫 관측
  ASSERT_EQ(FilterStep::kSmooth, f.update(0.0, 0.0, 10.1, kTau, kTimeout));
  const double vx1 = f.vx;
  // 같은 스탬프를 다시 (제어 주기가 더 빠르다)
  EXPECT_EQ(FilterStep::kHold, f.update(0.0, 0.0, 10.1, kTau, kTimeout));
  EXPECT_DOUBLE_EQ(vx1, f.vx);
}

// 같은 결함을 legacy 가 실제로 갖고 있는지 — 계약이 헛돌지 않음을 보인다.
TEST(TrackVelocityFilter, LegacyResetsOnRepeatedStamp)
{
  LegacyFilter g;
  g.update(1.0, 0.0, 10.0, kTau, kTimeout);
  g.update(0.0, 0.0, 10.1, kTau, kTimeout);
  const double vx1 = g.vx;
  g.update(0.0, 0.0, 10.1, kTau, kTimeout);            // 같은 스탬프 → legacy 는 원시값으로 리셋
  EXPECT_NE(vx1, g.vx) << "legacy 는 같은 스탬프에서 상태를 버린다 (이 결함을 고치는 것이 목적)";
  EXPECT_DOUBLE_EQ(0.0, g.vx);
}

// 20 Hz 제어 × 10 Hz 추적을 그대로 재현: legacy 는 평활이 전혀 안 되고, 새 필터는 τ 대로 된다.
TEST(TrackVelocityFilter, ControlFasterThanTrackingStillSmooths)
{
  TrackVelocityFilter f;
  LegacyFilter g;
  f.update(1.0, 0.0, 0.0, kTau, kTimeout);
  g.update(1.0, 0.0, 0.0, kTau, kTimeout);
  // 트랙은 0.1 s 마다 갱신되고 제어는 0.05 s 마다 같은 메시지를 두 번씩 본다.
  for (int k = 1; k <= 5; ++k) {
    const double t = 0.1 * k;
    f.update(0.0, 0.0, t, kTau, kTimeout);
    f.update(0.0, 0.0, t, kTau, kTimeout);            // 같은 메시지 재관측
    g.update(0.0, 0.0, t, kTau, kTimeout);
    g.update(0.0, 0.0, t, kTau, kTimeout);
  }
  // 새 필터: 0.5 s 동안 τ=0.5 → exp(-1) ≈ 0.368 이 남는다.
  EXPECT_NEAR(std::exp(-1.0), f.vx, 1e-9);
  // legacy: 매 두 번째 호출이 리셋이라 평활이 남지 않는다.
  EXPECT_DOUBLE_EQ(0.0, g.vx);
}

TEST(TrackVelocityFilter, BackwardStampResets)
{
  TrackVelocityFilter f;
  f.update(1.0, 0.0, 10.0, kTau, kTimeout);
  EXPECT_EQ(FilterStep::kReset, f.update(0.25, 0.0, 9.5, kTau, kTimeout));
  EXPECT_DOUBLE_EQ(0.25, f.vx);
}

TEST(TrackVelocityFilter, StaleTrackResets)
{
  TrackVelocityFilter f;
  f.update(1.0, 0.0, 10.0, kTau, kTimeout);
  EXPECT_EQ(FilterStep::kReset, f.update(0.25, 0.0, 10.0 + kTimeout + 0.01, kTau, kTimeout));
  EXPECT_DOUBLE_EQ(0.25, f.vx);
}

// 전진 스탬프의 지수 갱신이 해석해와 맞는지 (τ 의 의미가 지켜지는지).
TEST(TrackVelocityFilter, ForwardStampMatchesAnalyticDecay)
{
  TrackVelocityFilter f;
  f.update(1.0, 0.0, 0.0, kTau, kTimeout);
  f.update(0.0, 0.0, kTau, kTau, kTimeout);           // 한 시정수만큼 전진
  EXPECT_NEAR(std::exp(-1.0), f.vx, 1e-12);
}

// 첫 관측은 평활하지 않고 그대로 받는다 (트랙이 막 생겼다).
TEST(TrackVelocityFilter, FirstObservationPassesThrough)
{
  TrackVelocityFilter f;
  EXPECT_EQ(FilterStep::kInit, f.update(0.7, -0.3, 5.0, kTau, kTimeout));
  EXPECT_DOUBLE_EQ(0.7, f.vx);
  EXPECT_DOUBLE_EQ(-0.3, f.vy);
}

// tau <= 0 이면 평활하지 않는다 (기능 끄기).
TEST(TrackVelocityFilter, ZeroTauPassesRawThrough)
{
  TrackVelocityFilter f;
  f.update(1.0, 0.0, 0.0, 0.0, kTimeout);
  EXPECT_EQ(FilterStep::kSmooth, f.update(0.25, 0.0, 0.1, 0.0, kTimeout));
  EXPECT_DOUBLE_EQ(0.25, f.vx);
}
