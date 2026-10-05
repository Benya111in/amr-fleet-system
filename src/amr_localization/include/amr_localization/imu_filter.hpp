// IMU 전처리 코어 (ROS 비의존): 바이어스 보정 + 1차 저역 통과 + 기동/요청 시 바이어스 추정.
//
//   ω' = LPF(ω_raw − b_g),   a' = LPF(a_raw − b_a)   (a 는 중력 포함 그대로 — EKF 가 중력을 뺀다)
// 바이어스 출처 (우선순위): calibrate 요청 결과 > 기동 정지 추정(startup_time) > 파라미터 값.
// 기동 추정 중에는 지금까지의 누적 평균을 바이어스로 쓰며 발행한다 (min_startup_samples 이후).
// 추정 창 안에서 움직임이 감지되면(표본 표준편차가 상한 초과) 추정을 멈추고, 움직임 직전까지 모은
// 샘플이 min_startup_samples 이상이면 그 평균을, 아니면 파라미터 바이어스를 쓴다.

#ifndef AMR_LOCALIZATION__IMU_FILTER_HPP_
#define AMR_LOCALIZATION__IMU_FILTER_HPP_

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <deque>
#include <string>

#include "amr_localization/imu_bias_estimator.hpp"
#include "amr_localization/low_pass_filter.hpp"

namespace amr_localization
{

/// IMU 필터 파라미터.
struct ImuFilterParams
{
  double lpf_cutoff_hz{20.0};                                  ///< [Hz] ≤ 0 이면 LPF 끔
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};          ///< [rad/s] 초기/파일 값
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};         ///< [m/s²]
  double gravity{9.8};                                         ///< [m/s²]
  double startup_time{60.0};                                   ///< [s] 기동 정지 추정 시간, 0 = 끔
  std::size_t min_startup_samples{20};                         ///< 발행 시작 전 최소 샘플
  double stationary_gyro_std_max{0.01};                        ///< [rad/s]
  double stationary_accel_std_max{0.2};                        ///< [m/s²]
  double accel_norm_tolerance{0.5};                            ///< [m/s²] |mean a| − g 허용
  double max_gap{0.5};                                         ///< [s] 공백 초과 시 LPF 재시작
};

/// 바이어스 추정 결과 (기동 추정 또는 calibrate 요청).
struct BiasEstimateOutcome
{
  bool success{false};
  bool startup{false};       ///< 기동 추정이면 true, calibrate 요청이면 false
  std::string message;
  ImuBiasResult result;
  double duration{0.0};      ///< 추정 창 길이 [s]
};

/// IMU 전처리 코어.
class ImuFilter
{
public:
  explicit ImuFilter(const ImuFilterParams & params);

  /// 한 샘플 처리. 발행할 값이면 true 와 함께 gyro_out/accel_out 을 채운다.
  bool process(
    double stamp, const Eigen::Vector3d & gyro_raw, const Eigen::Vector3d & accel_raw,
    Eigen::Vector3d & gyro_out, Eigen::Vector3d & accel_out);

  /// calibrate 요청: 다음 samples 개 원시 샘플을 정지 평균한다. 이미 진행 중이면 false.
  bool startCalibration(std::size_t samples);

  /// 추정(기동/요청)이 끝났으면 결과를 꺼낸다 (한 번만).
  bool takeOutcome(BiasEstimateOutcome & outcome);

  /// 기동 추정 중인지.
  bool estimatingStartup() const {return startup_active_;}

  /// calibrate 진행 중인지.
  bool calibrating() const {return calibration_active_;}

  /// 현재 바이어스.
  const Eigen::Vector3d & gyroBias() const {return gyro_bias_;}
  const Eigen::Vector3d & accelBias() const {return accel_bias_;}

  /// 바이어스 추정 분산 (SE², 축별). 공분산에 가산한다.
  const Eigen::Vector3d & gyroBiasVariance() const {return gyro_bias_var_;}
  const Eigen::Vector3d & accelBiasVariance() const {return accel_bias_var_;}

  /// 파라미터.
  const ImuFilterParams & params() const {return params_;}

private:
  void applyEstimate(const ImuBiasResult & r);
  void finishStartup(double stamp, bool force_fail, const std::string & reason);

  ImuFilterParams params_;
  std::array<FirstOrderLowPass, 6> lpf_;
  Eigen::Vector3d gyro_bias_;
  Eigen::Vector3d accel_bias_;
  Eigen::Vector3d gyro_bias_var_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_var_{Eigen::Vector3d::Zero()};
  bool has_stamp_{false};
  double last_stamp_{0.0};

  bool startup_active_{false};
  bool startup_started_{false};
  double startup_begin_{0.0};
  ImuBiasEstimator startup_estimator_;

  ImuBiasResult last_stationary_;  ///< 움직임 감지 직전까지의 추정 (기동 추정 중단 시 사용)

  bool calibration_active_{false};
  std::size_t calibration_target_{0};
  double calibration_begin_{0.0};
  ImuBiasEstimator calibration_estimator_;

  std::deque<BiasEstimateOutcome> outcomes_;
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__IMU_FILTER_HPP_
