// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 다중 장애물 추적기 (ROS 비의존): 확장 행렬 GNN 연관 + 칼만 필터 + 생명주기 + 동적 판정.
//
// 한 스캔 주기(update) 의 순서 (docs/algorithms/tracking.md §2–§4):
//   1) 모든 트랙 예측 (dt = 스캔 스탬프 차)
//   2) 게이트: d² = ν^T S^-1 ν <= χ²₂(0.99) = 9.21, 물리 상한 |z - z_last| <= v_phys·Δt + 3√λmax(S)
//   3) 확장 비용 행렬 (n 트랙 + k 클러스터):
//        [ C_as (n×k)          | diag(c_miss) (n×n) ]     C_as = d² + ln det(2πS) - 2 ln P_D
//        [ diag(c_birth) (k×k) | 0 (k×n)            ]     c_miss = -2 ln(1-P_D),
//                                                        c_birth = -2 ln λ_B
//      → Hungarian 최소 비용 할당 (할당/미탐/탄생을 한 번에 베이즈 비교)
//   4) 갱신·미탐·탄생, 생명주기 (최근 5 스캔 중 3 히트 → 확정, 연속 미탐 > 5 → 삭제;
//      예측 위치가 더 가까운 클러스터 뒤에 가려져 있으면 > 15 까지 유지 — 기둥 뒤 통과)
//   5) 동적 판정: 최근 W 스캔 중심의 최소제곱 속도 기울기 v̂ 의 χ² 검정
//        T = v̂^T Cov(v̂)^-1 v̂ > χ²₂(0.999) 이고 |v̂| > v_min + 2σ_δ|ω_LOS| 가
//        N 스캔 연속 → is_dynamic
//
// 확장점: FilterFactory(IMM 등 다른 TrackFilter 주입),
//         TrackOutput.class_confidence(카메라 클래스 융합).

#ifndef AMR_PERCEPTION__OBSTACLE_TRACKER_HPP_
#define AMR_PERCEPTION__OBSTACLE_TRACKER_HPP_

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

#include "amr_perception/geometry2d.hpp"
#include "amr_perception/kalman_filter.hpp"
#include "amr_perception/scan_clustering.hpp"

namespace amr_perception
{

struct TrackerParams
{
  // 운동 모델
  double q{0.25};                  ///< CWNA 가속도 스펙트럼 밀도 [m^2/s^3]
  double init_velocity_std{1.5};   ///< 탄생 시 속도 표준편차 [m/s] (명세 장애물 최고속 1.5)
  // 연관
  double gate_chi2{9.21};          ///< χ²₂(0.99)
  double pd_visible{0.9};          ///< 검출 확률 (가시)
  double pd_occluded{0.5};         ///< 검출 확률 (가림 또는 원거리)
  double far_detection_range{8.0};  ///< 이 거리 밖이면 pd_occluded [m]
  double occlusion_margin{0.3};    ///< 가림 판정: 앞 클러스터가 이만큼 더 가까워야 [m]
  double lambda_birth{0.01};       ///< 탄생 밀도 λ_B [1/m^2]
  double big_cost{1.0e6};          ///< 금지 쌍 비용
  double v_phys{3.0};              ///< 물리 속도 상한 [m/s]
  // 생명주기
  int confirm_hits{3};             ///< 확정에 필요한 히트 수
  int confirm_window{5};           ///< 확정 판정 창 [스캔]
  double min_confirm_age{0.2};     ///< 첫 검출 후 최소 경과 [s]
  int max_misses{5};               ///< 확정 트랙 연속 미탐 허용 [스캔]
  /// 예측 위치가 가려져 있을 때의 연속 미탐 허용 [스캔] (기둥 뒤 통과 동안 트랙 유지)
  int max_misses_occluded{15};
  // 동적 판정
  int velocity_window{10};         ///< LS 기울기 창 [스캔]
  int velocity_min_samples{4};
  double velocity_chi2{13.82};     ///< χ²₂(0.999)
  double v_min{0.15};              ///< 최소 동적 속도 [m/s]
  double sigma_delta{0.10};        ///< 편향 잔차 (자차 운동 적응 임계 2σ_δ|ω_LOS|) [m]
  int dynamic_consecutive{2};      ///< 연속 발화 수 → is_dynamic
  int dynamic_release{5};          ///< 연속 미발화 수 → 해제
  // 출력
  double heading_min_speed{0.1};   ///< 이 속도 미만이면 직전 방향 유지 [m/s]
  std::array<double, 4> confidence_beta{{-2.0, 4.0, -0.5, 1.0}};  ///< 로지스틱 신뢰도 계수
  double confidence_sigma_ref{0.3};  ///< 신뢰도 공분산 기준 σ_ref [m]
  int quality_window{10};          ///< 히트 비율 계산 창 [스캔]
  double radius_smoothing{0.3};    ///< 반경 지수 평활 계수
  double max_dt{2.0};              ///< 스캔 간격이 이보다 크면 전체 리셋 [s]
};

/// 자차(센서) 운동 — 동적 판정의 시선 회전 보정용
struct EgoState
{
  Vec2 sensor_position{Vec2::Zero()};  ///< 추적 프레임 LiDAR 위치
  Vec2 sensor_velocity{Vec2::Zero()};  ///< 추적 프레임 LiDAR 속도 [m/s]
};

/// 트랙 출력 (추적 프레임)
struct TrackOutput
{
  uint32_t id{0};
  Vec2 position{Vec2::Zero()};
  Vec2 velocity{Vec2::Zero()};
  Mat4 covariance{Mat4::Identity()};
  double speed{0.0};
  double heading{0.0};             ///< atan2(vy, vx) [rad]
  double heading_std{M_PI};        ///< 방향 표준편차 [rad] (속도가 작으면 π)
  double confidence{0.0};          ///< 0~1
  double class_confidence{0.0};    ///< 확장점: 카메라 클래스 사후 최댓값 (베이스라인 0)
  bool is_dynamic{false};
  bool confirmed{false};
  double radius{0.1};              ///< 외접원 반경 [m]
  int hits{0};
  int misses{0};
  int consecutive_misses{0};
  double age{0.0};                 ///< 첫 검출 이후 [s]
  double ls_speed{0.0};            ///< 최근 LS 기울기 속도 크기 [m/s]
  double ls_chi2{0.0};             ///< 최근 LS 검정 통계량 T
};

/// 확장 행렬 GNN 결과
struct AssociationResult
{
  std::vector<int> track_to_cluster;  ///< -1 = 미탐
  std::vector<int> cluster_to_track;  ///< -1 = 탄생
};

/// 확장 비용 행렬 구성 (공개: 테스트용).
/// innovations[i][j], feasible[i][j], pd[i]. 크기 (n+k)×(k+n).
Eigen::MatrixXd buildAugmentedCostMatrix(
  const std::vector<std::vector<Innovation>> & innovations,
  const std::vector<std::vector<char>> & feasible, const std::vector<double> & pd,
  double lambda_birth, double big_cost);

/// 확장 행렬을 Hungarian 으로 풀어 할당 결과로 해석
AssociationResult solveGnnAssociation(
  const std::vector<std::vector<Innovation>> & innovations,
  const std::vector<std::vector<char>> & feasible, const std::vector<double> & pd,
  double lambda_birth, double big_cost);

/// 최소제곱 속도 기울기 검정 결과
struct VelocityTestResult
{
  bool valid{false};
  Vec2 velocity{Vec2::Zero()};
  double chi2{0.0};
};

/// (t_k, z_k) 표본의 LS 기울기 v̂ = Σ(t-t̄)(z-z̄)/S_tt 와 T = v̂^T (R/S_tt)^-1 v̂
VelocityTestResult leastSquaresVelocityTest(
  const std::vector<double> & t, const std::vector<Vec2> & z, const Mat2 & R_jitter,
  int min_samples);

class ObstacleTracker
{
public:
  using FilterFactory =
    std::function<std::unique_ptr<TrackFilter>(const Vec2 & z, const Mat2 & R)>;

  explicit ObstacleTracker(const TrackerParams & params, FilterFactory factory = FilterFactory());

  /// 한 스캔 주기 처리. stamp [s], clusters: 전경 클러스터 (추적 프레임)
  void update(double stamp, const std::vector<Cluster> & clusters, const EgoState & ego);

  /// 트랙 출력 (confirmed_only 면 확정 트랙만)
  std::vector<TrackOutput> outputs(bool confirmed_only = true) const;

  std::size_t size() const {return tracks_.size();}
  void reset();
  const TrackerParams & params() const {return params_;}
  double lastStamp() const {return last_stamp_;}

private:
  struct Sample
  {
    double t;
    Vec2 z;
  };

  struct Track
  {
    uint32_t id{0};
    std::unique_ptr<TrackFilter> filter;
    bool confirmed{false};
    double first_stamp{0.0};
    double last_update_stamp{0.0};
    Vec2 last_measurement{Vec2::Zero()};
    std::deque<bool> history;       // 최근 스캔 히트 여부 (앞 = 오래된 것)
    int hits{0};
    int misses{0};
    int consecutive_misses{0};
    std::deque<Sample> samples;     // LS 속도 검정 창
    Mat2 R_jitter{Mat2::Identity()};
    int fire_count{0};
    int quiet_count{0};
    bool is_dynamic{false};
    bool occluded{false};           // 이번 스캔에서 예측 위치가 가려져 있다
    double last_heading{0.0};
    bool heading_valid{false};
    double radius{0.1};
    double ls_speed{0.0};
    double ls_chi2{0.0};
  };

  /// 검출 확률. occluded 가 주어지면 "더 가까운 클러스터에 가려짐" 여부를 쓴다.
  double detectionProbability(
    const Track & track, const std::vector<Cluster> & clusters, const EgoState & ego,
    bool * occluded = nullptr) const;
  void updateDynamicState(Track & track, const EgoState & ego);
  TrackOutput makeOutput(const Track & track) const;
  static int countHits(const std::deque<bool> & history, int window);

  TrackerParams params_;
  FilterFactory factory_;
  std::vector<Track> tracks_;
  uint32_t next_id_{1};
  double last_stamp_{0.0};
  bool initialized_{false};
};

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__OBSTACLE_TRACKER_HPP_
