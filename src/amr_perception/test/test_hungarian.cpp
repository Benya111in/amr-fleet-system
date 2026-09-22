// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// Hungarian 할당 단위 테스트: 알려진 해, 직사각 행렬, 전수 탐색 대조.

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <algorithm>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

#include "amr_perception/hungarian.hpp"

namespace
{
// 행 수 <= 열 수 인 행렬의 전수 탐색 최소 비용
double bruteForce(const Eigen::MatrixXd & c)
{
  const int n = static_cast<int>(c.rows());
  const int m = static_cast<int>(c.cols());
  std::vector<int> cols(m);
  std::iota(cols.begin(), cols.end(), 0);
  double best = std::numeric_limits<double>::infinity();
  // 열 순열의 앞 n 개를 행에 배정 (중복 순열 포함, 작은 m 에서만 사용)
  std::sort(cols.begin(), cols.end());
  do {
    double s = 0.0;
    for (int i = 0; i < n; ++i) {
      s += c(i, cols[i]);
    }
    best = std::min(best, s);
  } while (std::next_permutation(cols.begin(), cols.end()));
  return best;
}
}  // namespace

TEST(Hungarian, KnownSquare)
{
  Eigen::MatrixXd c(3, 3);
  c << 4, 1, 3,
    2, 0, 5,
    3, 2, 2;
  const auto a = amr_perception::solveAssignment(c);
  ASSERT_EQ(a.size(), 3U);
  EXPECT_DOUBLE_EQ(amr_perception::assignmentCost(c, a), 5.0);  // (0,1)+(1,0)+(2,2)
  EXPECT_EQ(a[0], 1);
  EXPECT_EQ(a[1], 0);
  EXPECT_EQ(a[2], 2);
}

TEST(Hungarian, Rectangular)
{
  Eigen::MatrixXd c(2, 4);
  c << 9, 2, 7, 8,
    6, 4, 3, 7;
  const auto a = amr_perception::solveAssignment(c);
  EXPECT_DOUBLE_EQ(amr_perception::assignmentCost(c, a), 5.0);
  EXPECT_NE(a[0], a[1]);
}

TEST(Hungarian, MatchesBruteForceOnRandomProblems)
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(-5.0, 20.0);
  std::uniform_int_distribution<int> dim(1, 6);
  for (int trial = 0; trial < 300; ++trial) {
    const int n = dim(rng);
    const int m = n + dim(rng) % 3;
    Eigen::MatrixXd c(n, m);
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < m; ++j) {
        c(i, j) = u(rng);
        if (u(rng) > 15.0) {
          c(i, j) = 1e6;  // 금지 쌍
        }
      }
    }
    const auto a = amr_perception::solveAssignment(c);
    // 할당이 서로 다른 열인지
    std::vector<int> sorted = a;
    std::sort(sorted.begin(), sorted.end());
    EXPECT_TRUE(std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end());
    EXPECT_NEAR(amr_perception::assignmentCost(c, a), bruteForce(c), 1e-6) << "trial " << trial;
  }
}

TEST(Hungarian, EdgeCases)
{
  EXPECT_TRUE(amr_perception::solveAssignment(Eigen::MatrixXd(0, 3)).empty());
  EXPECT_THROW(amr_perception::solveAssignment(Eigen::MatrixXd::Zero(3, 2)), std::invalid_argument);
  Eigen::MatrixXd bad = Eigen::MatrixXd::Zero(2, 2);
  bad(0, 1) = std::numeric_limits<double>::infinity();
  EXPECT_THROW(amr_perception::solveAssignment(bad), std::invalid_argument);
  const std::vector<int> partial{-1, 0};
  Eigen::MatrixXd c = Eigen::MatrixXd::Ones(2, 2);
  EXPECT_DOUBLE_EQ(amr_perception::assignmentCost(c, partial), 1.0);
}
