// 저크 제한 속도 프로파일 필터 — ROS 비의존 (명세 4.5 "사다리꼴/S-curve
//   속도 프로파일, 최대 저크 제한").
//
// 상태 (v, a), 목표 v_ref, 한계 |a| ≤ a_max, |ȧ| ≤ j_max 인
//   이산 3차 필터 (Zanasi 2000 / Haschke 2008 계열,
// 연구 브리프 path-tracking §2.7). 스위칭면 σ(v, a) = (v_ref
//   − v) − a|a|/(2j) — "지금부터 최대 저크로
// a → 0 하면 v_ref 에 닿는가" — 에 다음 스텝에서 정확히 착지하는 가속도를 폐형식으로 구한다:
//   c   = v_ref − v − a·dt/2
//   a1* = sign(c)·j·(−dt/2 + √(dt²/4 + 2|c|/j))      (a1|a1|/(2j) + a1·dt/2 = c 의 해)
//   j_k = clamp((a1* − a)/dt, −j, j),  a1 = clamp(a + j_k·dt,
//   −a_max, a_max),  v ← v + (a + a1)·dt/2
//   종단: |v_ref − v| ≤ j·dt² 이고 |a| ≤ j·dt 이면 a = 0, v = v_ref (이 스텝의 저크도 ≤ j).
// 결과: 0 → 2 m/s (a 1, j 2) 가 2.5 s (이론 S-curve 와 같음), 오버슈트 0, |a| ≤ a_max, |ȧ| ≤ j_max.
// j_max ≤ 0 (또는 무한)이면 저크 제한 없는 사다리꼴(가속 한계만) 필터로 동작한다.
#ifndef AMR_NAVIGATION__CORE__JERK_LIMITER_HPP_
#define AMR_NAVIGATION__CORE__JERK_LIMITER_HPP_

namespace amr_navigation
{
namespace core
{

struct JerkLimits
{
  double v_min{-0.5};
  double v_max{2.0};
  double a_max{1.0};
  double j_max{2.0};      // ≤ 0 이면 사다리꼴
};

class JerkLimitedFilter
{
public:
  explicit JerkLimitedFilter(const JerkLimits & limits = JerkLimits());

  void setLimits(const JerkLimits & limits) {limits_ = limits;}
  const JerkLimits & limits() const {return limits_;}

  /// 상태 강제 설정 (재동기화).
  void reset(double v = 0.0, double a = 0.0);

  /// 한 스텝 전진. 반환: 새 속도.
  double step(double v_ref, double dt);

  double velocity() const {return v_;}
  double acceleration() const {return a_;}
  /// 직전 스텝의 저크 [m/s³] (진단용).
  double lastJerk() const {return last_jerk_;}

private:
  JerkLimits limits_;
  double v_{0.0};
  double a_{0.0};
  double last_jerk_{0.0};
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__JERK_LIMITER_HPP_
