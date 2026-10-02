// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// CWNA 등속 칼만 필터 단위 테스트: 예측/갱신 식, 수렴, NEES 일관성, 예측 공분산 폐형식.

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <random>

#include "amr_perception/kalman_filter.hpp"

using amr_perception::ConstantVelocityKalmanFilter;
using amr_perception::Mat2;
using amr_perception::Mat4;
using amr_perception::Vec2;
using amr_perception::Vec4;

TEST(KalmanFilter, TransitionAndProcessNoise)
{
  const Mat4 f = ConstantVelocityKalmanFilter::transition(0.1);
  EXPECT_DOUBLE_EQ(f(0, 2), 0.1);
  EXPECT_DOUBLE_EQ(f(1, 3), 0.1);
  EXPECT_DOUBLE_EQ(f(0, 0), 1.0);
  const Mat4 q = ConstantVelocityKalmanFilter::processNoise(0.25, 0.1);
  EXPECT_NEAR(q(0, 0), 0.25 * 1e-3 / 3.0, 1e-15);
  EXPECT_NEAR(q(0, 2), 0.25 * 1e-2 / 2.0, 1e-15);
  EXPECT_NEAR(q(2, 2), 0.25 * 0.1, 1e-15);
  EXPECT_DOUBLE_EQ(q(0, 1), 0.0);
  EXPECT_TRUE(q.isApprox(q.transpose()));
}

TEST(KalmanFilter, PredictMovesMeanAndGrowsCovariance)
{
  ConstantVelocityKalmanFilter kf(0.25, Vec2(1.0, 2.0), Mat2::Identity() * 0.01, 1.0);
  Vec4 x;
  x << 1.0, 2.0, 0.5, -1.0;
  kf.setState(x, Mat4::Identity() * 0.01);
  const double tr0 = kf.covariance().trace();
  kf.predict(0.2);
  EXPECT_NEAR(kf.state()(0), 1.1, 1e-12);
  EXPECT_NEAR(kf.state()(1), 1.8, 1e-12);
  EXPECT_GT(kf.covariance().trace(), tr0);
  kf.predict(-1.0);  // 음수 dt 는 무시
  EXPECT_NEAR(kf.state()(0), 1.1, 1e-12);
}

TEST(KalmanFilter, UpdateReducesUncertaintyAndStaysSymmetric)
{
  ConstantVelocityKalmanFilter kf(0.25, Vec2(0.0, 0.0), Mat2::Identity() * 0.04, 1.5);
  kf.predict(0.1);
  const Mat4 p0 = kf.covariance();
  const auto inn = kf.innovation(Vec2(0.1, 0.0), Mat2::Identity() * 0.0009);
  EXPECT_NEAR(inn.nu.x(), 0.1, 1e-12);
  EXPECT_GT(inn.mahalanobis2(), 0.0);
  kf.update(Vec2(0.1, 0.0), Mat2::Identity() * 0.0009);
  const Mat4 p1 = kf.covariance();
  EXPECT_LT(p1(0, 0), p0(0, 0));
  EXPECT_TRUE(p1.isApprox(p1.transpose(), 1e-12));
  Eigen::SelfAdjointEigenSolver<Mat4> es(p1);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
  EXPECT_GT(kf.state()(0), 0.0);
  EXPECT_LT(kf.state()(0), 0.1 + 1e-12);
  EXPECT_DOUBLE_EQ(kf.q(), 0.25);
  auto copy = kf.clone();
  EXPECT_TRUE(copy->state().isApprox(kf.state()));
}

TEST(KalmanFilter, ConvergesToTrueVelocity)
{
  // 1.0 m/s 로 45° 방향 등속, 점 측정 σ 0.03 m, 10 Hz, 3 s — 200 회 몬테카를로.
  // q = 0.25 의 정상상태 이득에서 점 측정 잡음만으로 생기는 속력 RMS 는 ≈ 0.11 m/s 이다
  // (클러스터 측정은 여러 빔 평균이라 잡음이 더 작다 — ObstacleTracker 시나리오 테스트 참고).
  const Vec2 v_true(1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0));
  const Mat2 r = Mat2::Identity() * 0.03 * 0.03;
  std::mt19937 rng(7);
  std::normal_distribution<double> n(0.0, 0.03);
  const int runs = 200;
  Vec2 bias = Vec2::Zero();
  double speed_se = 0.0;
  double heading_se = 0.0;
  double nees = 0.0;
  for (int run = 0; run < runs; ++run) {
    ConstantVelocityKalmanFilter kf(0.25, Vec2(n(rng), n(rng)), r, 1.5);
    for (int k = 1; k <= 30; ++k) {
      kf.predict(0.1);
      kf.update(v_true * (0.1 * k) + Vec2(n(rng), n(rng)), r);
    }
    const Vec2 v_est = kf.state().tail<2>();
    const Vec2 e = v_est - v_true;
    bias += e / runs;
    speed_se += std::pow(v_est.norm() - 1.0, 2) / runs;
    heading_se += std::pow(std::atan2(v_est.y(), v_est.x()) - M_PI / 4.0, 2) / runs;
    nees += e.dot(kf.covariance().bottomRightCorner<2, 2>().ldlt().solve(e)) / runs;
  }
  EXPECT_LT(bias.norm(), 0.02);                          // 불편 (등속 모델이 참이면)
  EXPECT_LT(std::sqrt(speed_se), 0.15);
  EXPECT_LT(std::sqrt(heading_se), 10.0 * M_PI / 180.0);
  // 참 운동에 가속이 없으므로 CWNA 공분산은 보수적 (NEES < 차원 2)
  EXPECT_LT(nees, 2.0);
  EXPECT_GT(nees, 0.2);
}

TEST(KalmanFilter, NeesConsistentOnCwnaTruth)
{
  // 참값을 같은 CWNA 모델로 생성 → 평균 NEES ≈ 4 (상태 차원)
  std::mt19937 rng(11);
  std::normal_distribution<double> n01(0.0, 1.0);
  const double q = 0.25;
  const double dt = 0.1;
  const double sr = 0.03;
  const Mat2 r = Mat2::Identity() * sr * sr;
  const Mat4 qd = ConstantVelocityKalmanFilter::processNoise(q, dt);
  const Eigen::LLT<Mat4> llt(qd);
  const Mat4 lq = llt.matrixL();
  double nees_sum = 0.0;
  int count = 0;
  for (int run = 0; run < 200; ++run) {
    Vec4 x;
    x << 0.0, 0.0, n01(rng), n01(rng);
    ConstantVelocityKalmanFilter kf(q, Vec2(x(0) + sr * n01(rng), x(1) + sr * n01(rng)), r, 1.0);
    Mat4 p0 = kf.covariance();
    p0(2, 2) = p0(3, 3) = 1.0;
    Vec4 x0 = kf.state();
    kf.setState(x0, p0);
    for (int k = 0; k < 40; ++k) {
      Vec4 w;
      w << n01(rng), n01(rng), n01(rng), n01(rng);
      x = ConstantVelocityKalmanFilter::transition(dt) * x + lq * w;
      kf.predict(dt);
      kf.update(Vec2(x(0) + sr * n01(rng), x(1) + sr * n01(rng)), r);
      if (k >= 20) {
        const Vec4 e = x - kf.state();
        nees_sum += e.dot(kf.covariance().ldlt().solve(e));
        ++count;
      }
    }
  }
  const double mean_nees = nees_sum / count;
  // N=4000 표본 평균의 95 % 대역은 4 ± 0.09 정도. 표본 간 상관을 감안해 넉넉히
  EXPECT_NEAR(mean_nees, 4.0, 0.4);
}

TEST(KalmanFilter, PredictedPositionCovarianceMatchesPropagation)
{
  ConstantVelocityKalmanFilter kf(0.3, Vec2(1.0, 1.0), Mat2::Identity() * 0.01, 0.5);
  kf.predict(0.1);
  kf.update(Vec2(1.05, 1.0), Mat2::Identity() * 0.001);
  for (double tau : {0.0, 0.5, 1.5, 3.0}) {
    auto copy = kf.clone();
    copy->predict(tau);
    const Mat2 expect = copy->covariance().topLeftCorner<2, 2>();
    EXPECT_TRUE(kf.predictPositionCovariance(tau).isApprox(expect, 1e-9)) << "tau " << tau;
    EXPECT_TRUE((kf.predictPosition(tau) - copy->state().head<2>()).norm() < 1e-12);
  }
}
