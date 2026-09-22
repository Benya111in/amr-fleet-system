// 휠 오도메트리 코어 (ROS 비의존): 인코더 에뮬레이터 → 순기구학 → 원호 적분 → 공분산.
// wheel_odometry_node 는 이 클래스를 감싸 joint_states → wheel_odom 으로 연결하는 얇은 래퍼다.
//
// 자세는 다회전 누적 조인트 각(위치)에서 계산하므로 메시지가 빠져도
// (공백 0.3 s @ 2 m/s 포함) 이동량이 사라지지 않는다 (encoder_model.hpp 다회전 카운터).
// 분기를 정할 수 없는 구간(감긴 입력 + 속도 없음)은 그 구간 변위 분산을
// 한 바퀴 둘레² 로 키워 EKF 가 믿지 않게 한다.
// 트위스트는 발행 구간 T 의 누적 변위 / T 로 구하고, 공분산은 같은 구간의 변위 분산에서 계산한다.

#ifndef AMR_LOCALIZATION__WHEEL_ODOMETRY_HPP_
#define AMR_LOCALIZATION__WHEEL_ODOMETRY_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <limits>

#include "amr_localization/diff_drive_kinematics.hpp"
#include "amr_localization/encoder_model.hpp"
#include "amr_localization/odometry_covariance.hpp"

namespace amr_localization
{

/// 오도메트리 코어 파라미터.
struct WheelOdometryParams
{
  DiffDriveGeometry geometry;
  EncoderParams encoder;
  WheelNoiseParams noise;
  IntegrationMethod integration{IntegrationMethod::kExactArc};
  double wheel_param_rel_stddev{0.0};  ///< r_i/b 의 상대 표준편차 (보정 잔여). 0 = 공칭값 신뢰
  double lateral_stddev{0.02};         ///< σ_vy0 [m/s]
  double lateral_skid_coeff{0.0};      ///< κ_y [s]
  /// [s] 인코더 샘플·발행 주기 (사이 입력은 건너뜀). 0 = 입력마다
  double publish_period{0.02};
  std::uint64_t seed{1};               ///< 인코더 잡음 seed (좌 = seed, 우 = seed + 1)
};

/// 발행 1회분 결과.
struct WheelOdometryOutput
{
  double stamp{0.0};             ///< [s] 입력 스탬프
  double interval{0.0};          ///< 트위스트 구간 T [s]
  Pose2D pose;                   ///< odom 프레임 자세
  Eigen::Matrix3d pose_covariance{Eigen::Matrix3d::Zero()};  ///< (x, y, θ)
  BodyVelocity twist;            ///< base_footprint 기준 (v, ω)
  TwistCovariance twist_covariance;
  std::int64_t ticks_left{0};
  std::int64_t ticks_right{0};
  double tick_rate_left{0.0};    ///< [tick/s]
  double tick_rate_right{0.0};
  int ambiguous_steps{0};        ///< 이 구간에서 분기 모호·불가능 점프로 분산을 키운 샘플 수
};

/// 휠 오도메트리 코어.
class WheelOdometry
{
public:
  explicit WheelOdometry(const WheelOdometryParams & params);

  /// 조인트 각 [rad] (과 선택적 조인트 속도 [rad/s], 없으면 NaN) 을 넣는다.
  /// 발행 시점이면 out 을 채우고 true.
  bool update(
    double stamp, double left_angle, double right_angle, WheelOdometryOutput & out,
    double left_velocity = std::numeric_limits<double>::quiet_NaN(),
    double right_velocity = std::numeric_limits<double>::quiet_NaN());

  /// 기동 후 분기 모호·불가능 점프로 분산을 키운 샘플 수 (진단).
  std::int64_t ambiguousSteps() const {return ambiguous_total_;}

  /// 자세·공분산을 주어진 값으로 초기화 (인코더 기준점은 유지).
  void resetPose(const Pose2D & pose = Pose2D());

  /// 인코더 기준점까지 모두 초기화 (시간 역행 시).
  void resetAll();

  /// 현재 자세.
  const Pose2D & pose() const {return pose_;}

  /// 현재 파라미터.
  const WheelOdometryParams & params() const {return params_;}

  /// 확장점 — 온라인 캘리브레이션: 바퀴 반지름을 갱신한다 (b 는 공칭 유지).
  void setWheelRadii(double left_radius, double right_radius);

  /// 확장점 — 파라미터 공분산 Σ_ψψ 를 갱신한다.
  void setParameterCovariance(const Eigen::Matrix2d & sigma_psi);

  /// 공분산 전파기 (테스트/진단).
  const OdometryCovariance & covariance() const {return covariance_;}

private:
  void resetInterval(double stamp);

  WheelOdometryParams params_;
  WheelEncoderModel left_encoder_;
  WheelEncoderModel right_encoder_;
  OdometryCovariance covariance_;
  Pose2D pose_;
  bool initialized_{false};
  double last_stamp_{0.0};
  double interval_start_{0.0};
  double next_publish_{0.0};
  // 구간 누적
  double acc_ds_left_{0.0};
  double acc_ds_right_{0.0};
  double acc_dphi_left_{0.0};
  double acc_dphi_right_{0.0};
  double acc_slip_var_left_{0.0};
  double acc_slip_var_right_{0.0};
  std::int64_t interval_ticks_left_{0};
  std::int64_t interval_ticks_right_{0};
  int interval_ambiguous_{0};
  std::int64_t ambiguous_total_{0};
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__WHEEL_ODOMETRY_HPP_
