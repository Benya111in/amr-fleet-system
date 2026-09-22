// 공분산 모델 단위 테스트: 야코비안(수치미분), 자세 공분산 전파 vs Monte-Carlo, 트위스트 공분산,
// 파라미터(ψ = r/b) 오차의 증강 전파 (Var θ ∝ L²), 양자화 유계항.

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <random>
#include <vector>

#include "amr_localization/diff_drive_kinematics.hpp"
#include "amr_localization/odometry_covariance.hpp"

using amr_localization::BodyIncrement;
using amr_localization::OdometryCovariance;
using amr_localization::Pose2D;

namespace
{
constexpr double kB = 0.36;
constexpr double kR = 0.0825;

Eigen::Vector3d stepPose(const Eigen::Vector3d & p, double ds_r, double ds_l)
{
  const BodyIncrement inc = amr_localization::wheelDisplacementsToIncrement(ds_l, ds_r, kB);
  const Pose2D out = amr_localization::integrateExactArc(Pose2D{p.x(), p.y(), p.z()}, inc);
  return Eigen::Vector3d(out.x, out.y, out.theta);
}

Eigen::Matrix3d sampleCovariance(const std::vector<Eigen::Vector3d> & s)
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & v : s) {
    mean += v;
  }
  mean /= static_cast<double>(s.size());
  Eigen::Matrix3d c = Eigen::Matrix3d::Zero();
  for (const auto & v : s) {
    c += (v - mean) * (v - mean).transpose();
  }
  return c / static_cast<double>(s.size() - 1);
}

/// 슬립 잡음만 있는 Monte-Carlo 와 1차 전파 비교.
void compareWithMonteCarlo(double v, double w, int steps, double dt, double sigma_s)
{
  const double ds_r = (v + 0.5 * w * kB) * dt;
  const double ds_l = (v - 0.5 * w * kB) * dt;
  amr_localization::WheelNoiseParams noise;
  noise.slip_noise_stddev = sigma_s;

  OdometryCovariance cov(kB);
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  for (int k = 0; k < steps; ++k) {
    const BodyIncrement inc = amr_localization::wheelDisplacementsToIncrement(ds_l, ds_r, kB);
    cov.propagate(
      p.z(), inc, ds_l / kR, ds_r / kR,
      amr_localization::stepDisplacementVariance(noise, ds_l),
      amr_localization::stepDisplacementVariance(noise, ds_r));
    p = stepPose(p, ds_r, ds_l);
  }

  std::mt19937_64 rng(1234);
  std::normal_distribution<double> n01(0.0, 1.0);
  std::vector<Eigen::Vector3d> finals;
  const int runs = 4000;
  for (int r = 0; r < runs; ++r) {
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    // 거리당 슬립: Δs + σ_s √(ℓ_ref |Δs|) N(0, 1) (인코더 모델과 같은 생성 과정)
    const double ref = noise.slip_reference_distance;
    for (int k = 0; k < steps; ++k) {
      q = stepPose(
        q, ds_r + sigma_s * std::sqrt(ref * std::abs(ds_r)) * n01(rng),
        ds_l + sigma_s * std::sqrt(ref * std::abs(ds_l)) * n01(rng));
    }
    finals.push_back(q);
  }
  const Eigen::Matrix3d mc = sampleCovariance(finals);
  const Eigen::Matrix3d & model = cov.poseCovariance();
  // 표본 분산의 상대 표준오차 ≈ √(2/4000) = 2.2 % → 대각 ±10 %
  for (int i = 0; i < 3; ++i) {
    if (model(i, i) > 1e-12) {
      EXPECT_NEAR(mc(i, i) / model(i, i), 1.0, 0.10) << "diag " << i;
    }
  }
  // 상관계수 비교 (비대각)
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      if (model(i, i) > 1e-12 && model(j, j) > 1e-12) {
        const double rho_model = model(i, j) / std::sqrt(model(i, i) * model(j, j));
        const double rho_mc = mc(i, j) / std::sqrt(mc(i, i) * mc(j, j));
        EXPECT_NEAR(rho_mc, rho_model, 0.06) << "corr " << i << j;
      }
    }
  }
}
}  // namespace

TEST(OdometryCovariance, JacobiansMatchFiniteDifferences)
{
  const double h = 1e-7;
  for (double theta : {0.0, 0.7, -2.5}) {
    for (double ds_r : {0.02, 0.031}) {
      for (double ds_l : {0.02, 0.012, -0.02}) {
        const Eigen::Vector3d p0(0.3, -0.2, theta);
        const BodyIncrement inc = amr_localization::wheelDisplacementsToIncrement(ds_l, ds_r, kB);
        const auto j = amr_localization::exactArcJacobians(theta, inc, kB);
        for (int c = 0; c < 3; ++c) {
          Eigen::Vector3d dp = Eigen::Vector3d::Zero();
          dp[c] = h;
          const Eigen::Vector3d num =
            (stepPose(p0 + dp, ds_r, ds_l) - stepPose(p0 - dp, ds_r, ds_l)) / (2.0 * h);
          EXPECT_LT((num - j.F_p.col(c)).norm(), 1e-7);
        }
        const Eigen::Vector3d num_r =
          (stepPose(p0, ds_r + h, ds_l) - stepPose(p0, ds_r - h, ds_l)) / (2.0 * h);
        const Eigen::Vector3d num_l =
          (stepPose(p0, ds_r, ds_l + h) - stepPose(p0, ds_r, ds_l - h)) / (2.0 * h);
        EXPECT_LT((num_r - j.F_u.col(0)).norm(), 1e-6);
        EXPECT_LT((num_l - j.F_u.col(1)).norm(), 1e-6);
      }
    }
  }
}

TEST(OdometryCovariance, StraightLineMatchesMonteCarlo)
{
  compareWithMonteCarlo(1.0, 0.0, 250, 0.02, 0.01);
}

TEST(OdometryCovariance, ArcMatchesMonteCarlo)
{
  compareWithMonteCarlo(0.5, 0.5, 300, 0.02, 0.01);
}

TEST(OdometryCovariance, RotationMatchesMonteCarlo)
{
  compareWithMonteCarlo(0.0, 1.0, 300, 0.02, 0.01);
}

TEST(OdometryCovariance, StraightHeadingVarianceClosedForm)
{
  // 직진 L: Var θ = 2 σ_s² ℓ_ref L / b² (docs/algorithms/kinematics.md §4) — 주기 Δs 와 무관
  amr_localization::WheelNoiseParams noise;
  OdometryCovariance cov(kB);
  const double ds = 0.02;
  const int steps = 1000;  // 20 m
  for (int k = 0; k < steps; ++k) {
    const double var = amr_localization::stepDisplacementVariance(noise, ds);
    cov.propagate(0.0, BodyIncrement{ds, 0.0}, ds / kR, ds / kR, var, var);
  }
  const double L = ds * steps;
  const double expected = 2.0 * 1e-4 * noise.slip_reference_distance * L / (kB * kB);
  EXPECT_NEAR(cov.poseCovariance()(2, 2) / expected, 1.0, 1e-9);
  // Var y ≈ 2 σ_s² ℓ_ref L³ / (3 b²)
  EXPECT_NEAR(cov.poseCovariance()(1, 1) / (expected * L * L / 3.0), 1.0, 0.01);
}

TEST(OdometryCovariance, SlipVarianceIndependentOfSampleRateAndSpeed)
{
  // 같은 20 m 직진을 주기 변위 0.005 / 0.02 / 0.04 m (속도·주기 조합) 로 적분해도 헤딩 분산이 같다.
  // 이전 주기당 곱셈 모델은 Δs 에 비례해 달라졌다 (리뷰: 0.5/1/2 m/s 에서 σ 2.17/3.02/4.41 mm).
  amr_localization::WheelNoiseParams noise;
  double first = 0.0;
  for (const double ds : {0.005, 0.02, 0.04}) {
    OdometryCovariance cov(kB);
    const int steps = static_cast<int>(std::lround(20.0 / ds));
    for (int k = 0; k < steps; ++k) {
      const double var = amr_localization::stepDisplacementVariance(noise, ds);
      cov.propagate(0.0, BodyIncrement{ds, 0.0}, ds / kR, ds / kR, var, var);
    }
    if (first == 0.0) {
      first = cov.poseCovariance()(2, 2);
    }
    EXPECT_NEAR(cov.poseCovariance()(2, 2) / first, 1.0, 1e-9) << "ds " << ds;
  }
}

TEST(OdometryCovariance, ParameterErrorGrowsQuadratically)
{
  // ψ 오차는 주기 간 완전 상관 → Var θ ∝ L² (증강 전파). MC 로 확인.
  const double rel = 0.001;  // 0.1 %
  OdometryCovariance cov(kB);
  Eigen::Matrix2d sp = Eigen::Matrix2d::Zero();
  sp(0, 0) = std::pow(rel * kR / kB, 2);
  sp(1, 1) = std::pow(rel * kR / kB, 2);
  cov.setParameterCovariance(sp);
  const double ds = 0.02;
  const double dphi = ds / kR;
  const int steps = 1000;
  std::vector<double> var_theta;
  for (int k = 0; k < steps; ++k) {
    cov.propagate(0.0, BodyIncrement{ds, 0.0}, dphi, dphi, 0.0, 0.0);
    if (k == steps / 2 - 1 || k == steps - 1) {
      var_theta.push_back(cov.poseCovariance()(2, 2));
    }
  }
  // 거리 2배 → 분산 4배
  EXPECT_NEAR(var_theta[1] / var_theta[0], 4.0, 1e-6);

  std::mt19937_64 rng(99);
  std::normal_distribution<double> n01(0.0, 1.0);
  std::vector<Eigen::Vector3d> finals;
  for (int r = 0; r < 4000; ++r) {
    const double er = rel * n01(rng);
    const double el = rel * n01(rng);
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    for (int k = 0; k < steps; ++k) {
      q = stepPose(q, ds * (1.0 + er), ds * (1.0 + el));
    }
    finals.push_back(q);
  }
  const Eigen::Matrix3d mc = sampleCovariance(finals);
  EXPECT_NEAR(mc(2, 2) / cov.poseCovariance()(2, 2), 1.0, 0.1);
  EXPECT_NEAR(mc(1, 1) / cov.poseCovariance()(1, 1), 1.0, 0.1);
}

TEST(OdometryCovariance, ParameterUpdateScalesCrossCovariance)
{
  OdometryCovariance cov(kB);
  Eigen::Matrix2d sp = Eigen::Matrix2d::Identity() * 1e-6;
  cov.setParameterCovariance(sp);
  cov.propagate(0.0, BodyIncrement{0.02, 0.0}, 0.2, 0.2, 0.0, 0.0);
  const amr_localization::Matrix32 before = cov.crossCovariance();
  EXPECT_GT(before.norm(), 0.0);
  const Eigen::Matrix2d ikh = 0.5 * Eigen::Matrix2d::Identity();
  cov.applyParameterUpdate(ikh, 0.25 * sp);
  EXPECT_LT((cov.crossCovariance() - 0.5 * before).norm(), 1e-15);
  EXPECT_DOUBLE_EQ(cov.parameterCovariance()(0, 0), 0.25e-6);
  cov.reset();
  EXPECT_DOUBLE_EQ(cov.poseCovariance().norm(), 0.0);
  EXPECT_DOUBLE_EQ(cov.crossCovariance().norm(), 0.0);
}

TEST(OdometryCovariance, QuantizationTermAddedOnceAtPublish)
{
  OdometryCovariance cov(kB);
  const double delta = kR * 2.0 * M_PI / 4096.0;
  const double q = amr_localization::quantizationVariance(delta);
  EXPECT_DOUBLE_EQ(q, delta * delta / 6.0);
  const Eigen::Matrix3d pub = cov.publishedPoseCovariance(0.0, q, q);
  EXPECT_NEAR(pub(2, 2), 2.0 * q / (kB * kB), 1e-18);
  EXPECT_NEAR(pub(0, 0), 0.5 * q, 1e-18);   // (1/2)²(q + q)
  EXPECT_NEAR(pub(1, 1), 0.0, 1e-18);
  // 여러 번 호출해도 누적되지 않는다 (상태를 바꾸지 않음)
  cov.publishedPoseCovariance(0.0, q, q);
  EXPECT_DOUBLE_EQ(cov.poseCovariance().norm(), 0.0);
}

TEST(OdometryCovariance, StepVarianceModel)
{
  amr_localization::WheelNoiseParams noise;
  noise.slip_noise_stddev = 0.01;
  noise.slip_reference_distance = 0.01;
  noise.slip_distance_coeff = 1e-6;
  // (σ_s² ℓ_ref + k)|Δs|
  EXPECT_NEAR(
    amr_localization::stepDisplacementVariance(noise, -0.02),
    (1e-4 * 0.01 + 1e-6) * 0.02, 1e-18);
}

TEST(TwistCovariance, MatchesMonteCarloOfWheelNoise)
{
  const double dt = 0.02;
  const double ds_r = 0.03;
  const double ds_l = 0.01;
  const double sigma = 0.01;
  amr_localization::TwistCovarianceInput in;
  in.var_ds_left = std::pow(sigma * ds_l, 2);
  in.var_ds_right = std::pow(sigma * ds_r, 2);
  in.dt = dt;
  in.separation = kB;
  const auto tc = amr_localization::computeTwistCovariance(in);

  std::mt19937_64 rng(7);
  std::normal_distribution<double> n01(0.0, 1.0);
  const int n = 200000;
  double sv = 0.0;
  double sw = 0.0;
  double svw = 0.0;
  double mv = 0.0;
  double mw = 0.0;
  std::vector<double> vs(n);
  std::vector<double> ws(n);
  for (int i = 0; i < n; ++i) {
    const double r = ds_r * (1.0 + sigma * n01(rng));
    const double l = ds_l * (1.0 + sigma * n01(rng));
    vs[i] = 0.5 * (r + l) / dt;
    ws[i] = (r - l) / (kB * dt);
    mv += vs[i];
    mw += ws[i];
  }
  mv /= n;
  mw /= n;
  for (int i = 0; i < n; ++i) {
    sv += (vs[i] - mv) * (vs[i] - mv);
    sw += (ws[i] - mw) * (ws[i] - mw);
    svw += (vs[i] - mv) * (ws[i] - mw);
  }
  EXPECT_NEAR(sv / (n - 1) / tc.var_v, 1.0, 0.02);
  EXPECT_NEAR(sw / (n - 1) / tc.var_w, 1.0, 0.02);
  EXPECT_NEAR(svw / (n - 1) / tc.cov_vw, 1.0, 0.03);
  EXPECT_NEAR(tc.var_vy, 0.02 * 0.02, 1e-15);
}

TEST(TwistCovariance, ParameterAndLateralTerms)
{
  amr_localization::TwistCovarianceInput in;
  in.dt = 0.02;
  in.separation = kB;
  in.phidot_left = 10.0;
  in.phidot_right = 10.0;
  in.sigma_psi = Eigen::Matrix2d::Identity() * 1e-8;
  in.lateral_skid_coeff = 0.1;
  in.v = 1.0;
  in.w = 0.5;
  const auto tc = amr_localization::computeTwistCovariance(in);
  // h_v = (b/2)[φ̇_R, φ̇_L] → h_vᵀΣh_v = (b/2)² · 200 · 1e-8
  EXPECT_NEAR(tc.var_v, std::pow(0.5 * kB, 2) * 200.0 * 1e-8, 1e-15);
  EXPECT_NEAR(tc.var_w, 200.0 * 1e-8, 1e-15);
  // h_vᵀΣh_w = (b/2)(φ̇_R² − φ̇_L²)·1e-8 = 0 (직진)
  EXPECT_NEAR(tc.cov_vw, 0.0, 1e-18);
  EXPECT_NEAR(tc.var_vy, 0.02 * 0.02 + std::pow(0.1 * 0.5, 2), 1e-15);
}
