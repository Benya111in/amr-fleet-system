// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// LiDAR 스캔 → 정적 배경 분리 → 적응형 유클리드 클러스터링 → 클러스터 측정 모델 (ROS 비의존).
//
// 파이프라인 (docs/algorithms/tracking.md §1, 연구 브리프 perception-tracking §3.3–3.4 베이스라인):
//   1) projectScan      : 유효 빔(유한, [range_min, range_max])을 추적 프레임 점으로 변환
//   2) StaticMapDistance: /map 점유 셀까지의 유클리드 거리 LUT(정확 EDT).
//                         d <= r_bg 인 점 = 정적 배경
//   3) segmentScan      : 적응형 브레이크포인트(ABD) — 연속 빔 간격이
//                         D_th = r·sin(Δφ)/sin(λ-Δφ) + 3σ_r 를 넘거나 배경 라벨이 바뀌면 끊는다
//   4) 병합/재분할       : 최소 점간 간격 < merge_min_gap 인 같은 라벨 세그먼트 병합,
//                         PCA 장축 길이 > max_extent 이면 가장 큰 내부 간격에서 재분할
//   5) buildCluster     : 시선 편향 보정 측정 z = p̄ + μ_δ·u_LOS 와 공분산
//                         R = R_ψ diag(σ_r²/n + σ_δ², (rΔφ)²/24 + σ_lat²) R_ψ^T
//   6) 부분 가림 표시     : 각도 경계 바로 옆 빔이 margin 이상 가까우면(앞 물체가 가림)
//                         보이는 조각의 중심이 가로로 치우치므로 가로 분산에 σ_occ² 를
//                         더하고 occluded 로 표시한다

#ifndef AMR_PERCEPTION__SCAN_CLUSTERING_HPP_
#define AMR_PERCEPTION__SCAN_CLUSTERING_HPP_

#include <cstdint>
#include <vector>

#include "amr_perception/geometry2d.hpp"

namespace amr_perception
{

/// sensor_msgs/LaserScan 의 필요한 부분 (ROS 비의존 복사본)
struct LaserScanData
{
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
  std::vector<float> ranges;
};

/// 투영된 스캔 점
struct ScanPoint
{
  Vec2 position{Vec2::Zero()};  ///< 추적 프레임 좌표
  double range{0.0};            ///< 센서 거리 [m]
  double bearing{0.0};          ///< 센서 좌표계 빔 각도 [rad]
  int beam{0};                  ///< 빔 인덱스
  bool background{false};       ///< 정적 지도에 속함
  double map_distance{-1.0};    ///< 가장 가까운 점유 셀까지 거리 [m] (-1 = 지도 없음)
};

/// nav_msgs/OccupancyGrid 로부터 만든 "가장 가까운 점유 셀까지의 거리" LUT.
/// Felzenszwalb–Huttenlocher 분리형 정확 유클리드 거리 변환, O(W·H). 지도가 바뀔 때만 다시 만든다.
class StaticMapDistance
{
public:
  /// data: 행 우선(y 행, x 열), -1 unknown / 0..100 점유 확률.
  /// origin: 셀 (0,0) 모서리의 map 좌표 자세.
  void build(
    int width, int height, double resolution, const Pose2D & origin,
    const std::vector<int8_t> & data, int occupied_threshold);
  bool valid() const {return width_ > 0 && height_ > 0;}
  /// map 프레임 점에서 가장 가까운 점유 셀 중심까지 거리 [m]. 지도 밖/점유 셀 없음 → +inf.
  double distance(const Vec2 & p_map) const;
  int width() const {return width_;}
  int height() const {return height_;}

private:
  int width_{0};
  int height_{0};
  double resolution_{0.05};
  Pose2D origin_inv_;
  std::vector<float> dist_;  // [m]
};

/// 1차원 제곱 거리 변환 (테스트용으로 공개).
/// f: 비용(점유 0, 비점유 큰 값) → d[q] = min_p (q-p)^2 + f[p]
void distanceTransform1d(const std::vector<double> & f, std::vector<double> & d);

struct SegmentationParams
{
  double lambda{0.17453292519943295};  ///< ABD 보조각 λ [rad] (10°)
  double sigma_r{0.03};                ///< 거리 잡음 σ_r [m] (sensors.yaml lidar.noise_stddev)
  double merge_min_gap{0.10};          ///< 세그먼트 병합 최소 점간 간격 [m]
  int min_points_near{3};              ///< far_range 이내 클러스터 최소 점 수
  int min_points_far{2};               ///< far_range 밖 최소 점 수
  double far_range{6.0};               ///< [m]
  double max_extent{1.5};              ///< PCA 장축 길이 상한 [m], 넘으면 재분할
  double max_range{12.0};              ///< 이 거리 밖 점은 무시 [m]
  double background_radius{0.10};      ///< r_bg: 점유 셀까지 이 거리 이내면 배경 [m]
  double overlap_radius{0.25};         ///< r_ov: 지도 중첩률 계산 반경 [m]
};

struct ClusterModelParams
{
  double bias_mu{0.15};      ///< 시선 편향 μ_δ [m] (미지 클래스): 보이는 표면 → 물체 중심
  double sigma_delta{0.10};  ///< 편향 잔차 σ_δ [m]
  double sigma_lat{0.03};    ///< 가로 형상 잡음 σ_lat [m]
  double sigma_r{0.03};      ///< 거리 잡음 σ_r [m]
  double sigma_floor{0.02};  ///< 성분별 표준편차 하한 [m]
  double sigma_seg{0.03};    ///< 프레임 간 분할 지터 σ_seg [m] (속도 검정용)
  double min_radius{0.10};   ///< 외접원 반경 하한 [m]
  double sigma_occluded{0.20};  ///< 가림 경계 클러스터의 가로 σ 가산 [m] (중심 치우침)
  double occlusion_margin{0.30};  ///< 이웃 빔이 이만큼 더 가까우면 그쪽 경계는 가려진 것 [m]
};

/// 추적기 입력이 되는 클러스터 (측정)
struct Cluster
{
  std::vector<Vec2> points;          ///< 추적 프레임 점
  Vec2 centroid{Vec2::Zero()};       ///< 점 평균 p̄
  Vec2 measurement{Vec2::Zero()};    ///< 편향 보정 측정 z
  Mat2 R{Mat2::Identity()};          ///< 위치 갱신용 측정 공분산
  Mat2 R_jitter{Mat2::Identity()};   ///< 속도 기울기 검정용 프레임 간 지터 공분산
  double mean_range{0.0};            ///< r̄ [m]
  double bearing_min{0.0};           ///< 센서 좌표계 각도 범위 (가림 판정)
  double bearing_max{0.0};
  int num_points{0};
  double length{0.0};                ///< PCA 장축 방향 길이 [m]
  double width{0.0};                 ///< PCA 단축 방향 길이 [m]
  double radius{0.0};                ///< z 기준 외접원 반경 [m]
  double map_overlap{0.0};           ///< r_ov 이내에 점유 셀이 있는 점의 비율 ρ_map
  bool occluded{false};              ///< 경계 옆 빔이 더 가깝다 (부분 가림 → R 가로 팽창)
};

/// 유효 빔을 추적 프레임 점으로. sensor_pose: 추적 프레임에서의 LiDAR 자세.
std::vector<ScanPoint> projectScan(
  const LaserScanData & scan, const Pose2D & sensor_pose, double max_range);

/// 점마다 배경 라벨 (map_from_tracking: 추적 프레임 → map)
void labelBackground(
  std::vector<ScanPoint> & points, const StaticMapDistance & map,
  const Pose2D & map_from_tracking, double background_radius);

/// ABD 분할. points 는 빔 순서. 반환: 세그먼트별 점 인덱스.
/// full_circle: 360° 스캔이면 첫/끝 세그먼트를 이어 붙일 수 있는지 검사. total_beams: 스캔 빔 수.
std::vector<std::vector<int>> segmentScan(
  const std::vector<ScanPoint> & points, double angle_increment,
  const SegmentationParams & params, bool full_circle, int total_beams);

/// ABD 거리 임계 D_th(r, Δφ). Δφ >= λ 이면 +inf 대신 음수(-1)를 돌려 "항상 끊기" 를 뜻한다.
double abdThreshold(double range, double dphi, double lambda, double sigma_r);

/// 같은 라벨·최소 점간 간격 < merge_min_gap 인 세그먼트를 병합 (union-find)
std::vector<std::vector<int>> mergeSegments(
  const std::vector<ScanPoint> & points, const std::vector<std::vector<int>> & segments,
  double merge_min_gap);

/// PCA 장축 길이가 max_extent 를 넘으면 가장 큰 연속 점 간격에서 재귀 분할
std::vector<std::vector<int>> splitOversized(
  const std::vector<ScanPoint> & points, const std::vector<std::vector<int>> & segments,
  double max_extent);

/// 클러스터 측정 모델 (시선 편향 보정 + 공분산). sensor_origin: 추적 프레임 센서 위치.
Cluster buildCluster(
  const std::vector<ScanPoint> & points, const std::vector<int> & indices,
  const Vec2 & sensor_origin, double angle_increment, const ClusterModelParams & params);

/// 세그먼트의 각도 경계 바로 바깥 빔 중 하나라도 mean_range - margin 보다 가까우면 true (부분 가림)
bool isOccludedAtEdge(
  const LaserScanData & scan, const std::vector<ScanPoint> & points,
  const std::vector<int> & indices, double mean_range, bool full_circle, double margin);

/// 전체 파이프라인: 배경(다수결) 세그먼트를 버리고 최소 점 수를 넘는 전경 클러스터만 돌려준다.
/// map 이 nullptr 또는 invalid 이면 배경 분리를 건너뛴다.
std::vector<Cluster> extractClusters(
  const LaserScanData & scan, const Pose2D & sensor_pose, const StaticMapDistance * map,
  const Pose2D & map_from_tracking, const SegmentationParams & seg,
  const ClusterModelParams & model);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__SCAN_CLUSTERING_HPP_
