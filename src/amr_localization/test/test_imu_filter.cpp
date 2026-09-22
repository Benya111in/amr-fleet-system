// IMU 전처리 단위 테스트: 바이어스 추정(Welford, SE), 바이어스 제거, 기동 추정/움직임 감지,
//   calibrate 요청.

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <random>

#include "amr_localization/imu_bias_estimator.hpp"
#include "amr_localization/imu_filter.hpp"

using amr_localization::BiasEstimateOutcome;
using amr_localization::ImuBiasEstimator;
using amr_localization::ImuFilter;
using amr_localization::ImuFilterParams;

namespace
{
// config/sensors.yaml 의 잡음 모델: 바이어스 크기 gyro 0.01, accel 0.10, 부호 무작위
const Eigen::Vector3d kGyroBias(0.01, -0.01, 0.01);
const Eigen::Vector3d kAccelBias(-0.10, 0.10, 0.10);
constexpr double kGyroSigma = 0.0002;
constexpr double kAccelSigma = 0.017;
constexpr double kG = 9.8;

/// Gazebo IMU 모사: 참값 + 바이어스 + 백색 잡음.
struct FakeImu
{
  std::mt19937_64 rng{2024};
  std::normal_distribution<double> n01{0.0, 1.0};

  void sample(
    const Eigen::Vector3d & true_gyro, const Eigen::Vector3d & true_accel,
    Eigen::Vector3d & gyro, Eigen::Vector3d & accel)
  {
    for (int i = 0; i < 3; ++i) {
      gyro[i] = true_gyro[i] + kGyroBias[i] + kGyroSigma * n01(rng);
      accel[i] = true_accel[i] + kAccelBias[i] + kAccelSigma * n01(rng);
    }
  }
};

const Eigen::Vector3d kStillAccel(0.0, 0.0, kG);
}  // namespace

TEST(ImuBiasEstimator, WelfordMatchesDirectComputation)
{
  ImuBiasEstimator est(kG);
  FakeImu imu;
  const int n = 6000;
  Eigen::Vector3d sum_g = Eigen::Vector3d::Zero();
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d g;
    Eigen::Vector3d a;
    imu.sample(Eigen::Vector3d::Zero(), kStillAccel, g, a);
    est.addSample(g, a);
    sum_g += g;
  }
  const auto r = est.result();
  EXPECT_EQ(r.samples, static_cast<std::size_t>(n));
  EXPECT_LT((r.gyro_bias - sum_g / n).norm(), 1e-12);
  // 60 s(6000 샘플) 추정 SE: gyro 2.6e-6, accel 2.2e-4 (sensor_calibration.md §2.4)
  EXPECT_NEAR(r.gyro_se.x(), kGyroSigma / std::sqrt(n), 3e-7);
  EXPECT_NEAR(r.accel_se.y(), kAccelSigma / std::sqrt(n), 2e-5);
  // 바이어스 복원 오차가 SE 의 4 배 이내
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(r.gyro_bias[i], kGyroBias[i], 4.0 * r.gyro_se[i]);
    EXPECT_NEAR(r.accel_bias[i], kAccelBias[i], 4.0 * r.accel_se[i]);
  }
  EXPECT_TRUE(est.isStationary(0.01, 0.2));
  est.reset();
  EXPECT_EQ(est.count(), 0u);
  EXPECT_FALSE(est.isStationary(0.01, 0.2));
  EXPECT_EQ(est.result().samples, 0u);
  EXPECT_DOUBLE_EQ(est.gravity(), kG);
}

TEST(ImuBiasEstimator, DetectsMotion)
{
  ImuBiasEstimator est(kG);
  FakeImu imu;
  for (int i = 0; i < 500; ++i) {
    Eigen::Vector3d g;
    Eigen::Vector3d a;
    // 0.5 rad/s 로 회전하다 멈춤 → 자이로 z 분산이 커진다
    imu.sample(Eigen::Vector3d(0.0, 0.0, i < 250 ? 0.5 : 0.0), kStillAccel, g, a);
    est.addSample(g, a);
  }
  EXPECT_FALSE(est.isStationary(0.01, 0.2));
  // 기울어진(가속도 크기 ≠ g) 경우도 정지로 보지 않는다
  ImuBiasEstimator tilted(kG);
  for (int i = 0; i < 100; ++i) {
    tilted.addSample(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 8.0));
  }
  EXPECT_FALSE(tilted.isStationary(0.01, 0.2));
}

TEST(ImuFilter, StartupEstimationRemovesBias)
{
  ImuFilterParams p;
  p.startup_time = 10.0;
  ImuFilter filter(p);
  FakeImu imu;
  Eigen::Vector3d g;
  Eigen::Vector3d a;
  Eigen::Vector3d go;
  Eigen::Vector3d ao;
  int published = 0;
  double t = 0.0;
  for (int i = 0; i < 1500; ++i, t += 0.01) {
    imu.sample(Eigen::Vector3d::Zero(), kStillAccel, g, a);
    if (filter.process(t, g, a, go, ao)) {
      ++published;
    }
  }
  // 첫 min_startup_samples(20) 는 발행하지 않는다
  EXPECT_EQ(published, 1500 - 19);
  EXPECT_FALSE(filter.estimatingStartup());
  BiasEstimateOutcome o;
  ASSERT_TRUE(filter.takeOutcome(o));
  EXPECT_TRUE(o.success);
  EXPECT_TRUE(o.startup);
  EXPECT_NEAR(o.duration, 10.0, 0.02);
  EXPECT_FALSE(filter.takeOutcome(o));
  // 10 s(1000 샘플) 추정: gyro SE 6.3e-6 → 바이어스 잔차 ≪ 1e-4
  EXPECT_LT((filter.gyroBias() - kGyroBias).norm(), 5e-5);
  EXPECT_LT((filter.accelBias() - kAccelBias).norm(), 5e-3);
  EXPECT_GT(filter.gyroBiasVariance().x(), 0.0);
  // 보정 후 정지 출력의 평균 ≈ 0 (자이로), 가속도 z ≈ g (중력은 제거하지 않음)
  Eigen::Vector3d mean_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_a = Eigen::Vector3d::Zero();
  const int n = 1000;
  for (int i = 0; i < n; ++i, t += 0.01) {
    imu.sample(Eigen::Vector3d::Zero(), kStillAccel, g, a);
    ASSERT_TRUE(filter.process(t, g, a, go, ao));
    mean_g += go;
    mean_a += ao;
  }
  mean_g /= n;
  mean_a /= n;
  EXPECT_LT(mean_g.norm(), 5e-5);
  EXPECT_NEAR(mean_a.z(), kG, 5e-3);
  EXPECT_NEAR(mean_a.x(), 0.0, 5e-3);
}

TEST(ImuFilter, LowPassReducesNoise)
{
  ImuFilterParams p;
  p.startup_time = 0.0;
  p.gyro_bias = kGyroBias;
  p.accel_bias = kAccelBias;
  p.lpf_cutoff_hz = 10.0;
  ImuFilter filter(p);
  FakeImu imu;
  Eigen::Vector3d g;
  Eigen::Vector3d a;
  Eigen::Vector3d go;
  Eigen::Vector3d ao;
  double sum2 = 0.0;
  const int n = 20000;
  for (int i = 0; i < n; ++i) {
    imu.sample(Eigen::Vector3d::Zero(), kStillAccel, g, a);
    ASSERT_TRUE(filter.process(0.01 * i, g, a, go, ao));
    sum2 += (ao.x()) * (ao.x());
  }
  // 1차 IIR 백색 잡음 분산비 α/(2 − α), α = 0.456 (fc 10 Hz) → σ × 0.54
  const double alpha = 0.456;
  EXPECT_NEAR(std::sqrt(sum2 / n) / kAccelSigma, std::sqrt(alpha / (2.0 - alpha)), 0.03);
}

TEST(ImuFilter, MotionDuringStartupFallsBack)
{
  ImuFilterParams p;
  p.startup_time = 10.0;
  p.gyro_bias = Eigen::Vector3d(0.001, 0.002, 0.003);
  ImuFilter filter(p);
  FakeImu imu;
  Eigen::Vector3d g;
  Eigen::Vector3d a;
  Eigen::Vector3d go;
  Eigen::Vector3d ao;
  double t = 0.0;
  // 5 s 정지 후 회전 시작 → 움직임 감지, 정지 구간 추정값 사용
  for (int i = 0; i < 700; ++i, t += 0.01) {
    imu.sample(Eigen::Vector3d(0.0, 0.0, i < 500 ? 0.0 : 0.8), kStillAccel, g, a);
    filter.process(t, g, a, go, ao);
  }
  EXPECT_FALSE(filter.estimatingStartup());
  BiasEstimateOutcome o;
  ASSERT_TRUE(filter.takeOutcome(o));
  EXPECT_TRUE(o.success);
  EXPECT_NE(o.message.find("motion"), std::string::npos);
  EXPECT_LT((filter.gyroBias() - kGyroBias).norm(), 2e-3);

  // 처음부터 움직이면 파라미터 바이어스로 돌아간다
  ImuFilter moving(p);
  t = 0.0;
  for (int i = 0; i < 100; ++i, t += 0.01) {
    imu.sample(Eigen::Vector3d(0.0, 0.0, std::sin(0.3 * i)), kStillAccel, g, a);
    moving.process(t, g, a, go, ao);
  }
  ASSERT_TRUE(moving.takeOutcome(o));
  EXPECT_FALSE(o.success);
  EXPECT_LT((moving.gyroBias() - p.gyro_bias).norm(), 1e-15);
  EXPECT_DOUBLE_EQ(moving.gyroBiasVariance().norm(), 0.0);
}

TEST(ImuFilter, CalibrationRequest)
{
  ImuFilterParams p;
  p.startup_time = 0.0;  // 파라미터 바이어스(0)로 시작
  ImuFilter filter(p);
  FakeImu imu;
  Eigen::Vector3d g;
  Eigen::Vector3d a;
  Eigen::Vector3d go;
  Eigen::Vector3d ao;
  EXPECT_FALSE(filter.startCalibration(1));  // 2 샘플 미만은 거부
  ASSERT_TRUE(filter.startCalibration(500));
  EXPECT_TRUE(filter.calibrating());
  EXPECT_FALSE(filter.startCalibration(500));  // 진행 중
  double t = 0.0;
  BiasEstimateOutcome o;
  for (int i = 0; i < 499; ++i, t += 0.01) {
    imu.sample(Eigen::Vector3d::Zero(), kStillAccel, g, a);
    filter.process(t, g, a, go, ao);
  }
  EXPECT_FALSE(filter.takeOutcome(o));
  imu.sample(Eigen::Vector3d::Zero(), kStillAccel, g, a);
  filter.process(t, g, a, go, ao);
  ASSERT_TRUE(filter.takeOutcome(o));
  EXPECT_TRUE(o.success);
  EXPECT_FALSE(o.startup);
  EXPECT_EQ(o.result.samples, 500u);
  EXPECT_LT((filter.gyroBias() - kGyroBias).norm(), 5e-5);
  EXPECT_FALSE(filter.calibrating());

  // 움직이는 중 요청 → 실패, 바이어스 유지
  const Eigen::Vector3d before = filter.gyroBias();
  ASSERT_TRUE(filter.startCalibration(200));
  for (int i = 0; i < 200; ++i, t += 0.01) {
    imu.sample(Eigen::Vector3d(0.0, 0.0, 0.5 * std::sin(0.1 * i)), kStillAccel, g, a);
    filter.process(t, g, a, go, ao);
  }
  ASSERT_TRUE(filter.takeOutcome(o));
  EXPECT_FALSE(o.success);
  EXPECT_LT((filter.gyroBias() - before).norm(), 1e-15);
}

TEST(ImuFilter, GapResetsLowPass)
{
  ImuFilterParams p;
  p.startup_time = 0.0;
  p.lpf_cutoff_hz = 1.0;
  ImuFilter filter(p);
  Eigen::Vector3d go;
  Eigen::Vector3d ao;
  filter.process(0.0, Eigen::Vector3d::Zero(), kStillAccel, go, ao);
  filter.process(0.01, Eigen::Vector3d(1.0, 1.0, 1.0), kStillAccel, go, ao);
  EXPECT_LT(go.x(), 0.2);  // 평활
  // 1 s 공백(> max_gap 0.5 s) → LPF 재시작, 새 값이 그대로 나온다
  filter.process(1.01, Eigen::Vector3d(2.0, 2.0, 2.0), kStillAccel, go, ao);
  EXPECT_DOUBLE_EQ(go.x(), 2.0);
  EXPECT_DOUBLE_EQ(filter.params().max_gap, 0.5);
}
