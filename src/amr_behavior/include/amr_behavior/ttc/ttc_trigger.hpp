// TTC 기반 재계획 트리거 (ROS 비의존). IsTTCBelowThreshold BT 조건 플러그인의 판단 로직.
//
// perception/tracked_obstacles 의 트랙별 time_to_collision 중 조건(동적 여부·신뢰도)을 만족하는
// 최솟값이 threshold 미만이면 트리거. 메시지가 max_age 보다 오래되면 트리거하지 않는다(추적기 정지
// 시 재계획 폭주 방지). cooldown > 0 이면 한 번 트리거한 뒤 그 시간 동안은 다시 트리거하지
// 않는다(재계획 주기 제한).
#ifndef AMR_BEHAVIOR__TTC__TTC_TRIGGER_HPP_
#define AMR_BEHAVIOR__TTC__TTC_TRIGGER_HPP_

#include <limits>
#include <vector>

namespace amr_behavior
{

/// 트랙 하나의 TTC 관련 값 (amr_msgs/TrackedObstacle 에서 추림).
struct TtcSample
{
  double ttc{std::numeric_limits<double>::infinity()};   ///< [s], inf = 비충돌
  bool is_dynamic{true};
  double confidence{1.0};
};

struct TtcFilter
{
  bool only_dynamic{true};     ///< 정적 트랙 무시
  double min_confidence{0.0};  ///< 이 미만 신뢰도 트랙 무시
};

/// 조건을 만족하는 트랙의 최소 TTC [s]. 음수·NaN 은 무시, 없으면 +inf.
double minTimeToCollision(const std::vector<TtcSample> & samples, const TtcFilter & filter);

class TtcTrigger
{
public:
  struct Config
  {
    double threshold{2.0};   ///< [s]
    double max_age{0.5};     ///< [s] 메시지 유효 시간
    double cooldown{0.0};    ///< [s] 재트리거 금지 시간
  };

  /// 새 메시지 수신 (stamp = 수신 시각 [s]).
  void update(const std::vector<TtcSample> & samples, double stamp);
  /// now 시각의 판단. true 면 트리거 (cooldown 시작).
  bool evaluate(double now, const Config & config, const TtcFilter & filter);
  /// 마지막 evaluate 에서 쓴 최소 TTC [s] (메시지가 오래됐으면 +inf).
  double lastMinTtc() const {return last_min_ttc_;}

private:
  std::vector<TtcSample> samples_;
  double stamp_{-std::numeric_limits<double>::infinity()};
  double last_trigger_{-std::numeric_limits<double>::infinity()};
  double last_min_ttc_{std::numeric_limits<double>::infinity()};
};

}  // namespace amr_behavior

#endif  // AMR_BEHAVIOR__TTC__TTC_TRIGGER_HPP_
