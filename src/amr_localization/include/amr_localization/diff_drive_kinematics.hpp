// 차동 구동 순기구학 (명세 4.2 "순기구학 직접 구현"). ROS 비의존 순수 라이브러리.
//
// 기호 (docs/algorithms/kinematics.md §1)
//   r_L, r_R : 좌/우 바퀴 반지름 [m],  b : 바퀴 중심 간 거리 [m]
//   Δφ_i     : 한 주기 바퀴 회전각 [rad],  Δs_i = r_i Δφ_i : 바퀴 접지점 이동 거리 [m]
//   Δs = (Δs_R + Δs_L) / 2 : 차체 호 길이,  Δθ = (Δs_R − Δs_L) / b : 헤딩 변화
//   v  = (r_R ω_R + r_L ω_L) / 2,  ω = (r_R ω_R − r_L ω_L) / b   (ω_i = 바퀴 각속도)
// 적분은 주기 동안 (v, ω) 가 일정하다는 가정의 정확한 원호 해를 기본으로 한다 (오일러 아님).

#ifndef AMR_LOCALIZATION__DIFF_DRIVE_KINEMATICS_HPP_
#define AMR_LOCALIZATION__DIFF_DRIVE_KINEMATICS_HPP_

#include <string>

namespace amr_localization
{

/// 평면 자세. theta 는 누적각(정규화하지 않음)이며 발행 시에만 정규화한다.
struct Pose2D
{
  double x{0.0};      ///< [m]
  double y{0.0};      ///< [m]
  double theta{0.0};  ///< [rad]
};

/// 차동 구동 기하 파라미터.
/// 좌우 반지름을 따로 두어 캘리브레이션(E_d)과 온라인 추정 확장을 허용한다.
struct DiffDriveGeometry
{
  double left_wheel_radius{0.0825};   ///< [m]
  double right_wheel_radius{0.0825};  ///< [m]
  double wheel_separation{0.36};      ///< [m]

  /// 모든 값이 양의 유한수인지.
  bool valid() const;
};

/// 한 주기 차체 증분.
struct BodyIncrement
{
  double ds{0.0};      ///< 호 길이 [m]
  double dtheta{0.0};  ///< 헤딩 변화 [rad]
};

/// 차체 속도 (base_footprint 기준, 비홀로노믹이라 v_y = 0).
struct BodyVelocity
{
  double v{0.0};  ///< 전진 속도 [m/s]
  double w{0.0};  ///< 요 각속도 [rad/s]
};

/// 적분 방식. kExactArc 가 기본, 나머지는 비교(테스트/문서)용.
enum class IntegrationMethod
{
  kEuler,      ///< x += Δs cos θ          (1차, 곡선에서 편향)
  kMidpoint,   ///< x += Δs cos(θ + Δθ/2)  (2차, 상대 오차 ≤ Δθ²/24)
  kExactArc    ///< x += Δs sinc(Δθ/2) cos(θ + Δθ/2)  (등속 원호의 정확해)
};

/// "euler" / "midpoint" / "exact_arc" → enum. 모르는 이름이면 std::invalid_argument.
IntegrationMethod parseIntegrationMethod(const std::string & name);

/// enum → 파라미터 문자열.
std::string toString(IntegrationMethod method);

/// 바퀴 이동 거리 → 차체 증분: Δs = (Δs_R + Δs_L)/2, Δθ = (Δs_R − Δs_L)/b.
BodyIncrement wheelDisplacementsToIncrement(
  double ds_left, double ds_right, double wheel_separation);

/// 바퀴 회전각 증분 → 차체 증분 (Δs_i = r_i Δφ_i).
BodyIncrement wheelAnglesToIncrement(
  double dphi_left, double dphi_right, const DiffDriveGeometry & geometry);

/// 순기구학: 바퀴 각속도 [rad/s] → 차체 속도. v = (r_R ω_R + r_L ω_L)/2, ω = (r_R ω_R − r_L ω_L)/b.
BodyVelocity forwardKinematics(
  double omega_left, double omega_right, const DiffDriveGeometry & geometry);

/// 역기구학: 차체 속도 → 바퀴 각속도 [rad/s]. ω_R = (v + ω b/2)/r_R, ω_L = (v − ω b/2)/r_L.
void inverseKinematics(
  const BodyVelocity & velocity, const DiffDriveGeometry & geometry,
  double & omega_left, double & omega_right);

/// sin(x)/x (x → 0 에서 테일러 전개로 수치 안정).
double sinc(double x);

/// d/dx sinc(x) = (x cos x − sin x)/x² (x → 0 에서 −x/3 + x³/30).
double sincDerivative(double x);

/// 각을 (−π, π] 로 정규화.
double normalizeAngle(double angle);

/// 자세 적분 (방식 선택).
Pose2D integrate(const Pose2D & pose, const BodyIncrement & increment, IntegrationMethod method);

/// 오일러: x += Δs cos θ, y += Δs sin θ, θ += Δθ.
Pose2D integrateEuler(const Pose2D & pose, const BodyIncrement & increment);

/// 중점(2차 룽게-쿠타): φ = θ + Δθ/2 방향으로 Δs.
Pose2D integrateMidpoint(const Pose2D & pose, const BodyIncrement & increment);

/// 정확한 원호 해. 현(chord) 길이 = Δs·sinc(Δθ/2), 방향 = θ + Δθ/2.
/// Δθ ≠ 0 일 때 x += (Δs/Δθ)[sin(θ+Δθ) − sin θ] 와 같고, Δθ → 0 에서도 특이점이 없다.
Pose2D integrateExactArc(const Pose2D & pose, const BodyIncrement & increment);

}  // namespace amr_localization

#endif  // AMR_LOCALIZATION__DIFF_DRIVE_KINEMATICS_HPP_
