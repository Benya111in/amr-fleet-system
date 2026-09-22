// 정지 구간 평균 기반 IMU 바이어스 추정 (명세 4.2 "바이어스 보정", sensor_calibration.md §2.4).
//
//   b_g = mean(ω), b_a = mean(a) − g_ref, g_ref = (0, 0, +g) (Gazebo IMU 는 정지 시 z ≈ +g 를 보고)
//   표준오차 SE = s / √N (s = 표본 표준편차)
// 평균/분산은 Welford 누적으로 수치 안정하게 구한다. 정지 여부는 표본 표준편차가 잡음 수준의
// 몇 배 이내인지로 판정한다 (주행 중이면 각속도/가속도 분산이 커진다).

#ifndef AMR_LOCALIZATION__IMU_BIAS_ESTIMATOR_HPP_
#define AMR_LOCALIZATION__IMU_BIAS_ESTIMATOR_HPP_

#include <Eigen/Core>

#include <cstddef>

namespace amr_localization
{

/// 추정 결과.
struct ImuBiasResult
{
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};   ///< [rad/s]
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};  ///< [m/s²] (중력 제외)
  Eigen::Vector3d gyro_std{Eigen::Vector3d::Zero()};    ///< 표본 표준편차 [rad/s]
  Eigen::Vector3d accel_std{Eigen::Vector3d::Zero()};   ///< [m/s²]
  Eigen::Vector3d gyro_se{Eigen::Vector3d::Zero()};     ///< 평균의 표준오차 [rad/s]
  Eigen::Vector3d accel_se{Eigen::Vector3d::Zero()};    ///< [m/s²]
  std::size_t samples{0};
};

/// Welford 누적 평균/분산 기반 바이어스 추정기.
class ImuBiasEstimator
{
public:
  /// gravity = |g| [m/s²] (월드 SDF <gravity> 크기, Gazebo 기본 9.8).
  explicit ImuBiasEstimator(double gravity = 9.8);

  /// 누적 초기화.
  void reset();

  /// 샘플 추가 (자이로 [rad/s], 가속도 [m/s²], 중력 포함 원시값).
  void addSample(const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel);

  /// 누적 샘플 수.
  std::size_t count() const {return count_;}

  /// 현재까지의 추정 결과 (count < 2 이면 표준편차/SE 는 0).
  ImuBiasResult result() const;

  /// 정지 판정: 모든 축 표본 표준편차가 상한 이하이고 |mean(a)| 가 g ± accel_norm_tolerance 인지.
  bool isStationary(
    double gyro_std_max, double accel_std_max, double accel_norm_tolerance = 0.5) const;

  /// 중력 크기.
  double gravity() const {return gravity_;}

private:
  double gravity_;
  std::size_t count_{0};
  Eigen::Vector3d gyro_mean_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_m2_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_mean_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_m2_{Eigen::Vector3d::Zero()};
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__IMU_BIAS_ESTIMATOR_HPP_
