// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 등속(CV) 칼만 필터 — 연속 백색 가속도(CWNA) 공정 잡음 (ROS 비의존).
//
// 상태 x = [px, py, vx, vy]^T (추적 프레임 = odom), 관측 z = [px, py]^T.
//   F(dt) = [[I, dt·I], [0, I]]
//   Q(dt) = q · [[dt^3/3·I, dt^2/2·I], [dt^2/2·I, dt·I]]   (q: 가속도 스펙트럼 밀도 [m^2/s^3])
//   H     = [I 0]
// 갱신은 Joseph 형식 P = (I-KH) P (I-KH)^T + K R K^T (수치적으로 대칭·양정치 유지).
// 유도와 파라미터 근거는 docs/algorithms/tracking.md §2.
//
// 확장점: TrackFilter 인터페이스. 연구 브리프 P1(JTC-IMM, 5차원 CV/CT/정지 모드)은
// 같은 인터페이스를 구현해 추적기에 주입한다 (Tracker 의 filter_factory).

#ifndef AMR_PERCEPTION__KALMAN_FILTER_HPP_
#define AMR_PERCEPTION__KALMAN_FILTER_HPP_

#include <Eigen/Core>

#include <memory>

#include "amr_perception/geometry2d.hpp"

namespace amr_perception
{

using Vec4 = Eigen::Vector4d;
using Mat4 = Eigen::Matrix4d;

/// 혁신(innovation) ν = z - Hx⁻, 혁신 공분산 S = H P⁻ H^T + R
struct Innovation
{
  Vec2 nu{Vec2::Zero()};
  Mat2 S{Mat2::Identity()};
  /// 정규화 혁신 제곱 d^2 = ν^T S^-1 ν
  double mahalanobis2() const;
};

/// 추적 필터 인터페이스 (위치 관측 2차원)
class TrackFilter
{
public:
  virtual ~TrackFilter() = default;
  /// dt [s] 만큼 시간 전파
  virtual void predict(double dt) = 0;
  /// 관측 z (공분산 R) 에 대한 혁신 — 상태를 바꾸지 않는다
  virtual Innovation innovation(const Vec2 & z, const Mat2 & R) const = 0;
  /// 관측 갱신
  virtual void update(const Vec2 & z, const Mat2 & R) = 0;
  /// [px, py, vx, vy]
  virtual Vec4 state() const = 0;
  /// 4x4 상태 공분산
  virtual Mat4 covariance() const = 0;
  /// tau [s] 뒤 위치 예측 (상태 불변)
  virtual Vec2 predictPosition(double tau) const = 0;
  /// tau [s] 뒤 위치 공분산 P_pp(tau) = P_pp + tau(P_pv + P_vp) + tau^2 P_vv + q tau^3/3 I
  virtual Mat2 predictPositionCovariance(double tau) const = 0;
  virtual std::unique_ptr<TrackFilter> clone() const = 0;
};

/// CWNA 등속 칼만 필터 (베이스라인 B1)
class ConstantVelocityKalmanFilter : public TrackFilter
{
public:
  /// q: 가속도 스펙트럼 밀도 [m^2/s^3], z0/R0: 첫 관측, init_velocity_std: 초기 속도 표준편차 [m/s]
  ConstantVelocityKalmanFilter(
    double q, const Vec2 & z0, const Mat2 & R0,
    double init_velocity_std);

  static Mat4 transition(double dt);
  static Mat4 processNoise(double q, double dt);

  void predict(double dt) override;
  Innovation innovation(const Vec2 & z, const Mat2 & R) const override;
  void update(const Vec2 & z, const Mat2 & R) override;
  Vec4 state() const override {return x_;}
  Mat4 covariance() const override {return P_;}
  Vec2 predictPosition(double tau) const override;
  Mat2 predictPositionCovariance(double tau) const override;
  std::unique_ptr<TrackFilter> clone() const override;

  double q() const {return q_;}
  /// 테스트/초기화용 직접 설정
  void setState(const Vec4 & x, const Mat4 & P);

private:
  double q_;
  Vec4 x_;
  Mat4 P_;
};

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__KALMAN_FILTER_HPP_
