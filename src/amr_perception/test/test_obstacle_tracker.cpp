// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 추적기 단위 테스트: 확장 행렬 GNN, LS 속도 검정, 이동/정지/교차/가림 시나리오, 생명주기.
//
// 시나리오는 scan_sim 으로 360° 스캔(σ 0.03 m, 10 Hz)을 만들어 extractClusters → ObstacleTracker
// 전체 파이프라인을 돌린다 (ROS 없이 노드와 같은 순서).

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <vector>

#include "amr_perception/kalman_filter.hpp"
#include "amr_perception/obstacle_tracker.hpp"
#include "amr_perception/scan_clustering.hpp"
#include "amr_perception/ttc.hpp"
#include "scan_sim.hpp"

using amr_perception::AssociationResult;
using amr_perception::ClusterModelParams;
using amr_perception::EgoState;
using amr_perception::Innovation;
using amr_perception::Mat2;
using amr_perception::ObstacleTracker;
using amr_perception::Pose2D;
using amr_perception::SegmentationParams;
using amr_perception::TrackerParams;
using amr_perception::TrackOutput;
using amr_perception::Vec2;

namespace
{

struct MovingCircle
{
  Vec2 p0;
  Vec2 v;
  double radius{0.2};
  Vec2 at(double t) const {return p0 + t * v;}
};

struct Step
{
  double t{0.0};
  Pose2D sensor;
  std::vector<TrackOutput> tracks;
};

/// 센서 등속 직선 운동(sensor_v, 추적 프레임) + 이동 원 + 정적 원 시나리오를 dt 간격으로 돌린다
std::vector<Step> runScenario(
  const std::vector<MovingCircle> & movers, const std::vector<scan_sim::Circle> & statics,
  double duration, const TrackerParams & tp, unsigned seed, const Vec2 & sensor_v = Vec2::Zero(),
  double dt = 0.1)
{
  std::mt19937 rng(seed);
  ObstacleTracker tracker(tp);
  std::vector<Step> out;
  const int steps = static_cast<int>(std::round(duration / dt));
  for (int i = 0; i <= steps; ++i) {
    const double t = i * dt;
    Step s;
    s.t = t;
    s.sensor = Pose2D{sensor_v.x() * t, sensor_v.y() * t, 0.0};
    std::vector<scan_sim::Circle> circles = statics;
    for (const auto & m : movers) {
      circles.push_back({m.at(t), m.radius});
    }
    const auto scan = scan_sim::makeScan(s.sensor, circles, {}, 0.03, &rng);
    const auto clusters = amr_perception::extractClusters(
      scan, s.sensor, nullptr, Pose2D{}, SegmentationParams{}, ClusterModelParams{});
    EgoState ego;
    ego.sensor_position = s.sensor.translation();
    ego.sensor_velocity = sensor_v;
    tracker.update(t, clusters, ego);
    s.tracks = tracker.outputs(true);
    out.push_back(s);
  }
  return out;
}

/// 참 위치에 가장 가까운 트랙 (없으면 nullptr)
const TrackOutput * nearest(const std::vector<TrackOutput> & tracks, const Vec2 & p, double gate)
{
  const TrackOutput * best = nullptr;
  double bd = gate;
  for (const auto & t : tracks) {
    const double d = (t.position - p).norm();
    if (d < bd) {
      bd = d;
      best = &t;
    }
  }
  return best;
}

double angleError(double a, double b)
{
  return std::abs(amr_perception::normalizeAngle(a - b));
}

Innovation makeInnovation(const Vec2 & nu, double var)
{
  Innovation in;
  in.nu = nu;
  in.S = Mat2::Identity() * var;
  return in;
}

}  // namespace

// ------------------------------------------------------------------ 연관 / 검정 단위

TEST(ObstacleTracker, AugmentedCostMatrixStructure)
{
  // 트랙 1, 클러스터 2 → 3×3. 금지 쌍은 big, 미탐/탄생 비용은 P_D, λ_B 로부터
  std::vector<std::vector<Innovation>> inn = {
    {makeInnovation(Vec2(0.1, 0.0), 0.01), makeInnovation(Vec2(1.0, 0.0), 0.01)}};
  std::vector<std::vector<char>> feas = {{1, 0}};
  const auto c = amr_perception::buildAugmentedCostMatrix(inn, feas, {0.9}, 0.01, 1e6);
  ASSERT_EQ(c.rows(), 3);
  ASSERT_EQ(c.cols(), 3);
  const double d2 = 0.01 / 0.01;
  const double expected = d2 + std::log((2.0 * M_PI * Mat2::Identity() * 0.01).determinant()) -
    2.0 * std::log(0.9);
  EXPECT_NEAR(c(0, 0), expected, 1e-9);
  EXPECT_DOUBLE_EQ(c(0, 1), 1e6);                         // 게이트 밖
  EXPECT_NEAR(c(0, 2), -2.0 * std::log(0.1), 1e-12);      // 미탐
  EXPECT_NEAR(c(1, 0), -2.0 * std::log(0.01), 1e-12);     // 클러스터 0 탄생
  EXPECT_NEAR(c(2, 1), -2.0 * std::log(0.01), 1e-12);     // 클러스터 1 탄생
  EXPECT_DOUBLE_EQ(c(1, 1), 1e6);                         // 탄생은 자기 열만
  EXPECT_DOUBLE_EQ(c(1, 2), 0.0);                         // 더미 × 더미
  EXPECT_DOUBLE_EQ(c(2, 2), 0.0);
}

TEST(ObstacleTracker, GnnAssignsNearestAndBirthsTheRest)
{
  // 트랙 2, 클러스터 3. 트랙 0 ↔ 클러스터 1, 트랙 1 ↔ 클러스터 0 이 가깝다. 클러스터 2 는 탄생.
  std::vector<std::vector<Innovation>> inn(2, std::vector<Innovation>(3));
  inn[0] = {makeInnovation(Vec2(0.5, 0.0), 0.04), makeInnovation(Vec2(0.02, 0.0), 0.04),
    makeInnovation(Vec2(0.4, 0.4), 0.04)};
  inn[1] = {makeInnovation(Vec2(0.01, 0.01), 0.04), makeInnovation(Vec2(0.5, 0.1), 0.04),
    makeInnovation(Vec2(0.5, 0.5), 0.04)};
  std::vector<std::vector<char>> feas = {{1, 1, 0}, {1, 1, 0}};
  const AssociationResult r = amr_perception::solveGnnAssociation(
    inn, feas, {0.9, 0.9}, 0.01, 1e6);
  EXPECT_EQ(r.track_to_cluster[0], 1);
  EXPECT_EQ(r.track_to_cluster[1], 0);
  EXPECT_EQ(r.cluster_to_track[2], -1);
  // 모든 쌍이 금지면 전부 미탐 + 탄생
  std::vector<std::vector<char>> none = {{0, 0, 0}, {0, 0, 0}};
  const AssociationResult r2 = amr_perception::solveGnnAssociation(
    inn, none, {0.9, 0.9}, 0.01, 1e6);
  EXPECT_EQ(r2.track_to_cluster[0], -1);
  EXPECT_EQ(r2.track_to_cluster[1], -1);
  // 빈 입력
  const AssociationResult r3 = amr_perception::solveGnnAssociation({}, {}, {}, 0.01, 1e6);
  EXPECT_TRUE(r3.track_to_cluster.empty());
}

TEST(ObstacleTracker, GnnPrefersMissWhenInnovationIsImplausible)
{
  // 게이트 안이지만 d² 가 큰 쌍: 할당 비용 > 미탐 + 탄생 이면 미탐+탄생이 선택된다
  std::vector<std::vector<Innovation>> inn = {{makeInnovation(Vec2(0.29, 0.0), 0.01)}};
  std::vector<std::vector<char>> feas = {{1}};
  // 할당 = d² 8.41 + ln det(2πS) (= 2 ln 0.0628 = -5.53) - 2 ln 0.9 (0.21) = 3.09
  // 미탐 + 탄생 = -2 ln 0.1 (4.61) - 2 ln λ_B. λ_B = 10 /m² (혼잡) 이면 0 < 3.09 → 미탐 + 탄생
  const auto cheap_birth = amr_perception::solveGnnAssociation(inn, feas, {0.9}, 10.0, 1e6);
  EXPECT_EQ(cheap_birth.track_to_cluster[0], -1);
  EXPECT_EQ(cheap_birth.cluster_to_track[0], -1);
  // 기본 λ_B = 0.01 이면 미탐 + 탄생 = 13.8 > 3.09 → 할당
  const auto normal = amr_perception::solveGnnAssociation(inn, feas, {0.9}, 0.01, 1e6);
  EXPECT_EQ(normal.track_to_cluster[0], 0);
}

TEST(ObstacleTracker, LeastSquaresVelocityTest)
{
  const Mat2 r = Mat2::Identity() * 0.03 * 0.03;
  std::vector<double> t;
  std::vector<Vec2> z;
  for (int i = 0; i < 10; ++i) {
    t.push_back(0.1 * i);
    z.push_back(Vec2(1.0 + 0.8 * 0.1 * i, 2.0 - 0.6 * 0.1 * i));
  }
  const auto res = amr_perception::leastSquaresVelocityTest(t, z, r, 4);
  ASSERT_TRUE(res.valid);
  EXPECT_NEAR(res.velocity.x(), 0.8, 1e-9);
  EXPECT_NEAR(res.velocity.y(), -0.6, 1e-9);
  // S_tt = Σ(t - 0.45)² = 0.825, T = S_tt |v|² / σ² = 0.825 · 1.0 / 9e-4
  EXPECT_NEAR(res.chi2, 0.825 / 9e-4, 1e-6);
  // 표본 부족 / 같은 시각
  EXPECT_FALSE(amr_perception::leastSquaresVelocityTest({0.0, 0.1}, {z[0], z[1]}, r, 4).valid);
  EXPECT_FALSE(
    amr_perception::leastSquaresVelocityTest(
      {1.0, 1.0, 1.0, 1.0}, {z[0], z[1], z[2], z[3]}, r, 4).valid);
}

// ------------------------------------------------------------------ 시나리오

TEST(ObstacleTracker, CrossingBlobVelocityHeadingAndDynamic)
{
  // 1.0 m/s 로 로봇 전방 4 m 를 가로지르는 원 (명세 4.7 의 1.0 m/s 동적 장애물)
  const MovingCircle mover{Vec2(4.0, -3.0), Vec2(0.0, 1.0), 0.2};
  const auto steps = runScenario({mover}, {}, 6.0, TrackerParams{}, 11);
  double first_dynamic = -1.0;
  int evaluated = 0;
  double max_speed_err = 0.0;
  double max_heading_err = 0.0;
  std::set<uint32_t> ids;
  for (const auto & s : steps) {
    const TrackOutput * tr = nearest(s.tracks, mover.at(s.t), 0.5);
    if (tr == nullptr) {
      continue;
    }
    ids.insert(tr->id);
    if (tr->is_dynamic && first_dynamic < 0.0) {
      first_dynamic = s.t;
    }
    if (s.t >= 1.5) {
      ++evaluated;
      max_speed_err = std::max(max_speed_err, std::abs(tr->speed - 1.0));
      max_heading_err = std::max(max_heading_err, angleError(tr->heading, M_PI / 2.0));
      EXPECT_TRUE(tr->is_dynamic) << "t=" << s.t;
      EXPECT_GT(tr->confidence, 0.5);
      EXPECT_LT(tr->heading_std, 0.3);
    }
  }
  EXPECT_GE(evaluated, 40);
  EXPECT_LT(max_speed_err, 0.1);
  EXPECT_LT(max_heading_err, 10.0 * M_PI / 180.0);
  ASSERT_GE(first_dynamic, 0.0);
  EXPECT_LE(first_dynamic, 1.0);  // 1.0 m/s: 확정 + 동적 판정 ≤ 1 s
  EXPECT_EQ(ids.size(), 1U);      // 한 번도 ID 가 바뀌지 않는다
}

TEST(ObstacleTracker, StaticObjectIsNotDynamicEvenWithEgoMotion)
{
  // 정지 원 + 센서가 1.0 m/s 로 옆을 지나간다 → 시선 회전과 형상 변화에도 동적 오탐 없음
  const std::vector<scan_sim::Circle> statics = {{Vec2(3.0, 1.2), 0.25}};
  const auto steps = runScenario({}, statics, 4.0, TrackerParams{}, 5, Vec2(1.0, 0.0));
  int seen = 0;
  for (const auto & s : steps) {
    for (const auto & tr : s.tracks) {
      ++seen;
      EXPECT_FALSE(tr.is_dynamic) << "t=" << s.t;
      if (s.t > 1.5) {
        EXPECT_LT(tr.speed, 0.25) << "t=" << s.t;
      }
    }
  }
  EXPECT_GT(seen, 20);
}

TEST(ObstacleTracker, CrossingTracksKeepIdentity)
{
  // X 자 교차 경로. B 는 A 보다 0.9 s 늦게 교차점을 지난다 (교차 시 간격 ≈ 0.9 m)
  const Vec2 cross(4.0, 0.0);
  const Vec2 va = Vec2(1.0, 1.0).normalized();
  const Vec2 vb = Vec2(1.0, -1.0).normalized();
  const MovingCircle a{cross - 2.0 * va, va, 0.2};
  const MovingCircle b{cross - 2.9 * vb, vb, 0.2};
  const auto steps = runScenario({a, b}, {}, 5.0, TrackerParams{}, 21);
  std::map<char, std::set<uint32_t>> ids;
  for (const auto & s : steps) {
    if (s.t < 0.6) {
      continue;  // 확정 전
    }
    const TrackOutput * ta = nearest(s.tracks, a.at(s.t), 0.4);
    const TrackOutput * tb = nearest(s.tracks, b.at(s.t), 0.4);
    if (ta != nullptr) {
      ids['a'].insert(ta->id);
      if (s.t > 1.5) {
        EXPECT_LT(angleError(ta->heading, M_PI / 4.0), 15.0 * M_PI / 180.0) << "t=" << s.t;
      }
    }
    if (tb != nullptr) {
      ids['b'].insert(tb->id);
      if (s.t > 1.5) {
        EXPECT_LT(angleError(tb->heading, -M_PI / 4.0), 15.0 * M_PI / 180.0) << "t=" << s.t;
      }
    }
  }
  EXPECT_EQ(ids['a'].size(), 1U);
  EXPECT_EQ(ids['b'].size(), 1U);
  EXPECT_NE(*ids['a'].begin(), *ids['b'].begin());
}

TEST(ObstacleTracker, TrackSurvivesShortOcclusion)
{
  // 센서와 이동체 사이 기둥(2 m) 뒤를 0.5 m/s 로 지나간다 → 가림 동안 P_D 를 낮춰 트랙 유지
  const std::vector<scan_sim::Circle> statics = {{Vec2(2.0, 0.0), 0.15}};
  const MovingCircle mover{Vec2(5.0, -2.0), Vec2(0.0, 0.8), 0.2};
  const auto steps = runScenario({mover}, statics, 5.0, TrackerParams{}, 8);
  std::set<uint32_t> ids;
  int occluded_steps = 0;
  for (const auto & s : steps) {
    const Vec2 p = mover.at(s.t);
    if (std::abs(std::atan2(p.y(), p.x())) < std::atan2(0.15, 2.0) + std::atan2(0.2, 5.0)) {
      ++occluded_steps;
    }
    if (s.t < 1.0) {
      continue;
    }
    const TrackOutput * tr = nearest(s.tracks, p, 0.6);
    if (tr != nullptr) {
      ids.insert(tr->id);
    }
  }
  EXPECT_GE(occluded_steps, 2);
  EXPECT_EQ(ids.size(), 1U);
}

TEST(ObstacleTracker, LifecycleTentativeAndConfirmedDeletion)
{
  TrackerParams tp;
  ObstacleTracker tracker(tp);
  std::mt19937 rng(4);
  EgoState ego;
  auto clustersFor = [&rng](const std::vector<scan_sim::Circle> & circles) {
      const auto scan = scan_sim::makeScan(Pose2D{}, circles, {}, 0.01, &rng);
      return amr_perception::extractClusters(
        scan, Pose2D{}, nullptr, Pose2D{}, SegmentationParams{}, ClusterModelParams{});
    };
  // 1 프레임 잡음 → 가(tentative) 트랙, 확정 안 됨, 3 연속 미탐 후 삭제
  tracker.update(0.0, clustersFor({{Vec2(2.0, 0.0), 0.2}}), ego);
  EXPECT_EQ(tracker.size(), 1U);
  EXPECT_TRUE(tracker.outputs(true).empty());
  EXPECT_EQ(tracker.outputs(false).size(), 1U);
  for (int i = 1; i <= 3; ++i) {
    tracker.update(0.1 * i, {}, ego);
  }
  EXPECT_EQ(tracker.size(), 0U);

  // 확정: 3 히트 + 최소 나이 0.2 s
  double t = 1.0;
  for (int i = 0; i < 3; ++i, t += 0.1) {
    tracker.update(t, clustersFor({{Vec2(3.0, 1.0), 0.2}}), ego);
  }
  ASSERT_EQ(tracker.outputs(true).size(), 1U);
  const auto confirmed = tracker.outputs(true).front();
  EXPECT_EQ(confirmed.hits, 3);
  EXPECT_NEAR(confirmed.age, 0.2, 1e-9);
  // max_misses(5) 까지는 유지, 6 번째 연속 미탐에서 삭제
  for (int i = 0; i < 5; ++i, t += 0.1) {
    tracker.update(t, {}, ego);
  }
  ASSERT_EQ(tracker.outputs(true).size(), 1U);
  EXPECT_EQ(tracker.outputs(true).front().consecutive_misses, 5);
  EXPECT_LT(tracker.outputs(true).front().confidence, confirmed.confidence);
  tracker.update(t, {}, ego);
  EXPECT_EQ(tracker.size(), 0U);
}

TEST(ObstacleTracker, ResetsOnTimeReversalAndLongGap)
{
  ObstacleTracker tracker{TrackerParams{}};
  std::mt19937 rng(9);
  const auto scan = scan_sim::makeScan(Pose2D{}, {{Vec2(2.0, 0.5), 0.2}}, {}, 0.01, &rng);
  const auto clusters = amr_perception::extractClusters(
    scan, Pose2D{}, nullptr, Pose2D{}, SegmentationParams{}, ClusterModelParams{});
  tracker.update(10.0, clusters, EgoState{});
  tracker.update(10.1, clusters, EgoState{});
  const uint32_t id0 = tracker.outputs(false).front().id;
  // 시간 역행 (시뮬레이션 리셋) → 기존 트랙 버리고 새로 탄생
  tracker.update(1.0, clusters, EgoState{});
  ASSERT_EQ(tracker.size(), 1U);
  EXPECT_NE(tracker.outputs(false).front().id, id0);
  EXPECT_DOUBLE_EQ(tracker.lastStamp(), 1.0);
  // max_dt(2 s) 초과 공백도 리셋
  const uint32_t id1 = tracker.outputs(false).front().id;
  tracker.update(5.0, clusters, EgoState{});
  EXPECT_NE(tracker.outputs(false).front().id, id1);
  tracker.reset();
  EXPECT_EQ(tracker.size(), 0U);
}

TEST(ObstacleTracker, CustomFilterFactoryIsUsed)
{
  int created = 0;
  ObstacleTracker tracker(
    TrackerParams{}, [&created](const Vec2 & z, const Mat2 & r) {
      ++created;
      return std::make_unique<amr_perception::ConstantVelocityKalmanFilter>(0.5, z, r, 1.0);
    });
  std::mt19937 rng(2);
  const auto scan = scan_sim::makeScan(
    Pose2D{}, {{Vec2(2.0, 0.5), 0.2}, {Vec2(2.0, -1.5), 0.2}}, {}, 0.01, &rng);
  tracker.update(
    0.0, amr_perception::extractClusters(
      scan, Pose2D{}, nullptr, Pose2D{}, SegmentationParams{}, ClusterModelParams{}),
    EgoState{});
  EXPECT_EQ(created, 2);
  EXPECT_EQ(tracker.params().confirm_hits, 3);
}

TEST(ObstacleTracker, EndToEndTtcMatchesAnalyticOnCollisionCourse)
{
  // 로봇(센서)이 +x 로 1.0 m/s, 장애물이 x = 4 m 를 +y 로 1.0 m/s 횡단 → 해석 TTC 와 비교
  const Vec2 robot_v(1.0, 0.0);
  const MovingCircle mover{Vec2(4.0, -4.0), Vec2(0.0, 1.0), 0.2};  // t=4 s 에 정면 충돌
  const auto steps = runScenario({mover}, {}, 2.0, TrackerParams{}, 17, robot_v);
  amr_perception::TtcParams tp;
  tp.k_sigma = 0.0;  // 해석해와 같은 기하 (불확실성 팽창 없음)
  const auto & last = steps.back();
  const TrackOutput * tr = nearest(last.tracks, mover.at(last.t), 0.5);
  ASSERT_NE(tr, nullptr);
  // 추정 TTC
  amr_perception::ObstacleMotion om;
  om.position = tr->position;
  om.velocity = tr->velocity;
  om.radius = 0.2;
  const auto robot = amr_perception::RobotMotionModel::straightLine(
    last.sensor.translation(), 0.0, 1.0);
  const double est = amr_perception::computeTimeToCollision(om, robot, tp).ttc;
  // 해석해: 참 상태
  amr_perception::ObstacleMotion truth;
  truth.position = mover.at(last.t);
  truth.velocity = mover.v;
  truth.radius = 0.2;
  const double ref = amr_perception::computeTimeToCollision(truth, robot, tp).ttc;
  ASSERT_TRUE(std::isfinite(ref));
  EXPECT_NEAR(est, ref, 0.3);
}
