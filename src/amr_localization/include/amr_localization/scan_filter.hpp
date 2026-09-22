// 2D LiDAR 스캔 필터
// (명세 4.2 "LiDAR 포인트 클라우드 필터링(거리 필터, 각도 필터, 아웃라이어 제거)").
//
// 빔 수·각도 메타데이터(angle_min/increment)는 그대로 두고 값만 바꾼다
// (AMCL/slam_toolbox/costmap 이 빔 인덱스로 각도를 계산하므로 배열을 줄이면 안 된다).
// 무효 값 규약 (REP-117):
//   range < range_min → −Inf,  range > range_max 또는 미검출 → +Inf,
//   제거(각도/아웃라이어/섀도우) → NaN
// 단계 (docs/algorithms/kinematics.md §6):
//   1. 거리 필터  [range_min, range_max] (센서 값과 파라미터 중 좁은 쪽)
//   2. 각도 필터  angle_window 밖 + angle_mask 구간 제거 (적재물·구조물 가림 섹터)
//   3. 아웃라이어(스페클) 제거: 이웃 일관성. 빔 i 의 끝점 P_i 에 대해 ±outlier_window 빔 안에서
//      |P_i − P_j| ≤ outlier_thresh + outlier_range_gain·r_i·|i−j|·Δα 인 이웃이
//      outlier_min_neighbors 개 미만이면 고립점으로 제거.
//      임계의 r·Δα 항은 인접 빔 끝점 간격(거리에 비례)을 반영한다.
//   4. 섀도우(베일) 제거: 인접 빔 끝점을 잇는 선과 광선의 각
//      β = atan2(r_j sin Δα, r_i − r_j cos Δα) 가 shadow_min_angle 미만 또는
//      π − shadow_min_angle 초과이고, 거리 차가 잡음으로 설명되지 않을 때
//      (|r_i − r_j| > shadow_noise_factor·√2·σ_r) 먼 쪽 점 제거 (모서리 혼합 픽셀).
//      잡음 조건이 없으면 근거리(r·Δα ≪ σ_r)에서 같은 면의 잡음 차만으로 β 가 작아져
//      먼 쪽(잡음이 + 인 쪽)만 지워지고 남은 점이 짧게 치우친다
//      (σ 0.03 m, 0.5 m 벽: 59 % 유지, −1.5 cm).
// 360° 스캔이면(첫·끝 빔이 한 증분 간격) 이웃 탐색이 배열 끝에서 감긴다.

#ifndef AMR_LOCALIZATION__SCAN_FILTER_HPP_
#define AMR_LOCALIZATION__SCAN_FILTER_HPP_

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace amr_localization
{

/// 필터 파라미터 (config/scan_filter.yaml 에 의미·단위·근거).
struct ScanFilterParams
{
  double range_min{0.10};             ///< [m]
  double range_max{25.0};             ///< [m]
  double angle_window_min{-M_PI};     ///< [rad] 유지할 각도 창 하한 (센서 프레임)
  double angle_window_max{M_PI};      ///< [rad] 상한
  std::vector<std::pair<double, double>> angle_mask;  ///< 제거할 [시작, 끝] 구간들 [rad]
  int outlier_window{2};              ///< 이웃 탐색 반경 [빔], 0 이면 아웃라이어 제거 끔
  double outlier_thresh{0.15};        ///< 기본 거리 임계 [m]
  double outlier_range_gain{3.0};     ///< 끝점 간격 r·Δα 에 곱하는 배수 [무차원]
  int outlier_min_neighbors{1};       ///< 필요한 지지 이웃 수
  bool shadow_filter_enabled{true};
  double shadow_min_angle{0.17453292519943295};  ///< [rad] (10°)
  /// σ_r [m] LiDAR 거리 잡음 (sensors.yaml lidar.noise_stddev)
  double shadow_range_noise_stddev{0.03};
  /// k: 거리 차 |r_i − r_j| ≤ k·√2·σ_r 이면 잡음으로 보고 유지
  double shadow_noise_factor{3.0};
};

/// 단계별 제거 통계.
struct ScanFilterStats
{
  std::size_t input{0};         ///< 전체 빔 수
  std::size_t nan_input{0};     ///< 입력부터 NaN 이었던 빔
  std::size_t too_close{0};     ///< < range_min
  std::size_t too_far{0};       ///< > range_max 또는 +Inf (미검출)
  std::size_t angle_removed{0};
  std::size_t outliers{0};
  std::size_t shadows{0};
  std::size_t valid_output{0};  ///< 유한값으로 남은 빔
};

/// 스캔 필터 (상태 없음, 스레드 안전).
class ScanFilter
{
public:
  explicit ScanFilter(const ScanFilterParams & params = ScanFilterParams());

  /// 파라미터.
  const ScanFilterParams & params() const {return params_;}

  /// 필터 적용. out 은 in 과 같은 길이. sensor_range_min/max 는 입력 메시지의 값.
  ScanFilterStats apply(
    const std::vector<float> & in, double angle_min, double angle_increment,
    double sensor_range_min, double sensor_range_max, std::vector<float> & out) const;

  /// 실제 적용 거리 하한 = max(센서, 파라미터).
  double effectiveRangeMin(double sensor_range_min) const;

  /// 실제 적용 거리 상한 = min(센서, 파라미터).
  double effectiveRangeMax(double sensor_range_max) const;

  /// 각도 a 가 제거 대상인지 (창 밖 또는 마스크 안). 구간은 [시작 → 끝] 반시계 호로 해석하므로
  /// 시작 > 끝이면 ±π 를 넘어가는 구간이다 (예: 후방 [2.8, −2.8]). 폭이 2π 이상이면 전체.
  bool isAngleRemoved(double angle) const;

  /// angle 이 반시계 호 [start → end] 안인지.
  static bool inArc(double angle, double start, double end);

private:
  ScanFilterParams params_;
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__SCAN_FILTER_HPP_
