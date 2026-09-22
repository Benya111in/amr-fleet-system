// 휠 오도메트리 공분산 모델 (명세 4.3 "각 입력의 공분산 행렬 근거 문서화").
//
// 자세: 증강 상태 s = [p; ψ], p = (x, y, θ), ψ = (ψ_R, ψ_L) = (r_R/b, r_L/b) 로 1차 전파한다.
//   Σ_pp ← F_p Σ_pp F_pᵀ + F_p Σ_pψ F_ψᵀ + F_ψ Σ_ψp F_pᵀ + F_ψ Σ_ψψ F_ψᵀ + F_u Σ_u F_uᵀ
//   Σ_pψ ← F_p Σ_pψ + F_ψ Σ_ψψ
//   F_p = ∂p'/∂p, F_u = ∂p'/∂(Δs_R, Δs_L)   (정확한 원호 적분의 야코비안)
//   F_ψ = F_u diag(b Δφ_R, b Δφ_L)
//   Σ_u = diag(Var Δs_R, Var Δs_L) — 거리당 슬립 잡음 σ_s² ℓ_ref |Δs_i| + 물리 슬립 k|Δs_i|.
//   양자화는 Σ_u 에 넣지 않는다: 누적 카운트의 양자화 오차는 유계라 매 주기 백색으로 더하면
//   가짜 랜덤워크가 된다 → 발행 시 1 회만 더한다.
// 트위스트: 발행 구간 T 의 바퀴 변위 분산 σ_i²(T) 로부터
//   Var v = (σ_R² + σ_L²)/(4T²), Var ω = (σ_R² + σ_L²)/(b²T²), Cov(v, ω) = (σ_R² − σ_L²)/(2bT²)
//   (+ 파라미터 불확실성 hᵀ Σ_ψψ h). 유도는 docs/algorithms/kinematics.md §3.

#ifndef AMR_LOCALIZATION__ODOMETRY_COVARIANCE_HPP_
#define AMR_LOCALIZATION__ODOMETRY_COVARIANCE_HPP_

#include <Eigen/Core>

#include "amr_localization/diff_drive_kinematics.hpp"

namespace amr_localization
{

using Matrix32 = Eigen::Matrix<double, 3, 2>;

/// 한 주기 적분의 야코비안.
struct StepJacobians
{
  Eigen::Matrix3d F_p;  ///< ∂(x', y', θ')/∂(x, y, θ)
  Matrix32 F_u;         ///< ∂(x', y', θ')/∂(Δs_R, Δs_L)
};

/// 정확한 원호 적분(integrateExactArc)의 야코비안. theta = 주기 시작 헤딩.
StepJacobians exactArcJacobians(double theta, const BodyIncrement & increment, double separation);

/// 바퀴 변위 잡음 모델 파라미터 (config/sensors.yaml wheel_encoder, wheel_odometry.yaml).
struct WheelNoiseParams
{
  double slip_noise_stddev{0.01};    ///< σ_s: 기준 굴림 ℓ_ref 에서의 상대 슬립 σ [무차원]
  double slip_reference_distance{0.01};  ///< ℓ_ref [m]: 슬립 분산 σ_s² ℓ_ref |Δs| 의 기준 거리
  /// k [m]: 물리 슬립 분산 계수 Var = k|Δs| (사전값 0, 드리프트 실험으로 식별)
  double slip_distance_coeff{0.0};
};

/// 한 주기 바퀴 변위 분산 (자세 전파용, 양자화 제외): (σ_s² ℓ_ref + k)|Δs| [m²].
/// 거리에 비례하므로 누적 분산은 샘플 주기·속도와 무관하다.
double stepDisplacementVariance(const WheelNoiseParams & noise, double ds);

/// 양자화 유계항: 구간 양 끝 카운트의 반올림 오차 차 q_end − q_start 의 분산 δ²/6 [m²]
/// (δ = 1 틱 거리).
double quantizationVariance(double tick_distance);

/// 차동 구동 트위스트 공분산 성분 (base_footprint 기준).
struct TwistCovariance
{
  double var_v{0.0};    ///< Var(v_x) [m²/s²]
  double var_w{0.0};    ///< Var(ω)   [rad²/s²]
  double cov_vw{0.0};   ///< Cov(v_x, ω) [m rad/s²]
  double var_vy{0.0};   ///< Var(v_y) [m²/s²] — 비홀로노믹 구속(v_y ≡ 0)의 신뢰도
};

/// 트위스트 공분산 입력.
struct TwistCovarianceInput
{
  double var_ds_left{0.0};    ///< 구간 T 의 좌 바퀴 변위 분산 σ_L²(T) [m²]
  double var_ds_right{0.0};   ///< 구간 T 의 우 바퀴 변위 분산 σ_R²(T) [m²]
  double dt{0.02};            ///< 구간 길이 T [s]
  double separation{0.36};    ///< b [m]
  double phidot_left{0.0};    ///< 좌 바퀴 평균 각속도 [rad/s] (파라미터 항 h 계산)
  double phidot_right{0.0};   ///< 우 바퀴 평균 각속도 [rad/s]
  Eigen::Matrix2d sigma_psi{Eigen::Matrix2d::Zero()};  ///< Σ_ψψ (ψ = r/b 의 공분산)
  double lateral_stddev{0.02};      ///< σ_vy0 [m/s]
  double lateral_skid_coeff{0.0};   ///< κ_y [s]: 선회 시 횡미끄럼 Var 가산 (κ_y v ω)²
  double v{0.0};                    ///< 현재 v [m/s] (횡미끄럼 항)
  double w{0.0};                    ///< 현재 ω [rad/s]
};

/// 트위스트 공분산 계산 (식은 파일 머리 주석).
TwistCovariance computeTwistCovariance(const TwistCovarianceInput & in);

/// 증강 자세 공분산 전파기.
class OdometryCovariance
{
public:
  /// separation = 공칭 b (ψ 매개화의 b̄).
  explicit OdometryCovariance(double separation = 0.36);

  /// Σ_pp, Σ_pψ 를 0 으로 (Σ_ψψ 는 유지).
  void reset();

  /// 파라미터 공분산 Σ_ψψ 설정 (온라인 추정기 확장점: 추정기가 ψ 와 P 를 갱신하면 여기로 넘긴다).
  void setParameterCovariance(const Eigen::Matrix2d & sigma_psi);

  /// 파라미터 추정기 측정 갱신 후 교차공분산 보정: Σ_pψ ← Σ_pψ (I − K hᵀ)ᵀ.
  void applyParameterUpdate(
    const Eigen::Matrix2d & i_minus_k_ht, const Eigen::Matrix2d & sigma_psi);

  /// 한 주기 전파.
  /// @param theta 주기 시작 헤딩 [rad]
  /// @param increment 적분한 차체 증분
  /// @param dphi_left, dphi_right 바퀴 회전각 증분 [rad] (F_ψ)
  /// @param var_ds_left, var_ds_right 주기 바퀴 변위 분산 [m²] (Σ_u)
  void propagate(
    double theta, const BodyIncrement & increment, double dphi_left, double dphi_right,
    double var_ds_left, double var_ds_right);

  /// 전파된 Σ_pp.
  const Eigen::Matrix3d & poseCovariance() const {return sigma_pp_;}

  /// 교차공분산 Σ_pψ.
  const Matrix32 & crossCovariance() const {return sigma_ppsi_;}

  /// 파라미터 공분산 Σ_ψψ.
  const Eigen::Matrix2d & parameterCovariance() const {return sigma_psi_;}

  /// 발행용 Σ = Σ_pp + F_u⁰ diag(q_R, q_L) F_u⁰ᵀ (양자화 유계항 1회, F_u⁰ = Δs→0 극한).
  Eigen::Matrix3d publishedPoseCovariance(
    double theta, double quant_var_left, double quant_var_right) const;

private:
  double separation_;
  Eigen::Matrix3d sigma_pp_;
  Matrix32 sigma_ppsi_;
  Eigen::Matrix2d sigma_psi_;
};

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__ODOMETRY_COVARIANCE_HPP_
