// 계약 상태 스키마 — rl/contract/amr_rl_contract/state.py 의 C++ 사본 (ROS 비의존).
//
// 관측·마스크 함수의 입력이다. 무반사 빔·상한 없음·TTC 없음은 +inf (파이썬의 None).
#ifndef AMR_RL_CONTROLLER__CORE__STATE_HPP_
#define AMR_RL_CONTROLLER__CORE__STATE_HPP_

#include <array>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "amr_navigation/core/geometry.hpp"
#include "amr_rl_controller/core/contract.hpp"

namespace amr_rl_controller
{

using Point2D = amr_navigation::core::Point2D;
using Pose2D = amr_navigation::core::Pose2D;

constexpr double kInf = std::numeric_limits<double>::infinity();

/// 추적 객체 하나 (TrackedObstacle.msg 의 부분집합).
/// 상태와 같은 고정 프레임, 메시지 스탬프 시각 기준.
struct Track
{
  int64_t id{0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  bool is_dynamic{false};
};

/// safety node 상태 (직전 주기). speed_limit·min_ttc 의 +inf = 없음.
struct SafetyState
{
  int zone{contract::kZoneClear};
  double speed_limit{kInf};
  bool proximity_stop{false};
  double min_ttc{kInf};
};

/// 이탈 상태기계 (reward machine). in_dev 가 거짓이면 peak·t_since_peak 는 0.
struct DevState
{
  bool in_dev{false};
  double peak{0.0};
  double t_since_peak{0.0};
};

/// 가속도 쌍 (a_v [m/s²], a_ω [rad/s²]).
using Accel = std::pair<double, double>;

/// 결정 시각의 계약 상태.
struct ContractState
{
  Pose2D pose;                       ///< 고정 프레임 base 자세 (x, y, θ)
  std::vector<Point2D> path;         ///< A* 경로 (같은 프레임), 끝 = 목표
  std::vector<double> scan_ranges;   ///< 720 빔, 무반사 = NaN 또는 inf
  double v_meas{0.0};
  double w_meas{0.0};
  double a_v_meas{0.0};
  double a_w_meas{0.0};
  double v_cmd{0.0};                 ///< 직전 정책 명령 (적분 상태)
  double w_cmd{0.0};
  std::array<Accel, contract::kNPrevActions> prev_actions{};   ///< 최근 순, 물리 단위
  double v_cap{contract::kVMaxOp};
  double scan_age{0.0};
  std::vector<Track> tracks;
  double track_age{0.0};
  SafetyState safety;
  DevState dev;
};

}  // namespace amr_rl_controller

#endif  // AMR_RL_CONTROLLER__CORE__STATE_HPP_
