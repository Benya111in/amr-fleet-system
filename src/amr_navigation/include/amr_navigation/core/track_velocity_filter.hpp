// 트랙 속도 지수 평활 (통로 예측용).
// VO·TTC 는 원시 속도를 그대로 쓴다 — dwa_controller.cpp 주석 참조.
//
// 왜 분리했는가: 이 갱신 규칙이 `DWAController` 의 private 멤버 안에 인라인으로 있어 단위 계약을
// 걸 수 없었고, 그 사이 "같은 메시지 재관측" 과 "시간 역행" 을 한 조건(`gap <= 0.0`)으로 묶은
// 결함이 오래 숨어 있었다. `obstaclesInFrame` 은 **매 제어 주기(20 Hz)** 호출되면서 **캐시된
// 최신 트랙 메시지**를 쓰는데, 추적기가 그보다 느리면 매 다른 틱마다 gap == 0 이 되어 필터가
// 원시값으로 리셋됐다 — nav2_params.yaml 이 약속한 τ 평활이 한 번도 켜진 적이 없다는 뜻이다.
// 이 값은 predVx()/predVy() 로 crossing_yield 의 통로 구성에 직행하므로 committed 판정의 입력이다.
#ifndef AMR_NAVIGATION__CORE__TRACK_VELOCITY_FILTER_HPP_
#define AMR_NAVIGATION__CORE__TRACK_VELOCITY_FILTER_HPP_

#include <cmath>

namespace amr_navigation
{
namespace core
{

/// 갱신이 어느 갈래를 탔는지 — 계약이 분기를 직접 단언할 수 있게 노출한다.
enum class FilterStep
{
  kInit,     ///< 첫 관측 (상태 없음) → 현재 값으로 시작
  kHold,     ///< 같은 스탬프 = 새 정보 없음 → 상태 불변, 직전 출력 유지
  kReset,    ///< 스탬프 역행 또는 timeout 초과 = 불연속 → 현재 값으로 다시 시작
  kSmooth,   ///< 스탬프 전진 → 지수 갱신
};

struct TrackVelocityFilter
{
  double vx{0.0};
  double vy{0.0};
  double stamp{-1.0};   ///< 마지막으로 **갱신에 쓴** 트랙 메시지 스탬프 [s]. < 0 이면 상태 없음.

  /// tau <= 0 이면 평활하지 않는다 (원시값 통과). timeout 은 트랙 끊김 판정 [s].
  FilterStep update(double vx_raw, double vy_raw, double stamp_s, double tau, double timeout)
  {
    if (stamp < 0.0) {
      vx = vx_raw;
      vy = vy_raw;
      stamp = stamp_s;
      return FilterStep::kInit;
    }
    const double gap = stamp_s - stamp;
    // 같은 메시지를 다시 본 것(gap == 0)은 불연속이 아니다 — 새 정보가 없을 뿐이다.
    // 상태를 건드리지 않고 직전 출력을 그대로 유지한다.
    if (gap == 0.0) {
      return FilterStep::kHold;
    }
    if (gap < 0.0 || gap > timeout) {
      vx = vx_raw;
      vy = vy_raw;
      stamp = stamp_s;
      return FilterStep::kReset;
    }
    if (tau > 0.0) {
      const double a = 1.0 - std::exp(-gap / tau);
      vx += a * (vx_raw - vx);
      vy += a * (vy_raw - vy);
    } else {
      vx = vx_raw;
      vy = vy_raw;
    }
    stamp = stamp_s;
    return FilterStep::kSmooth;
  }
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__TRACK_VELOCITY_FILTER_HPP_
