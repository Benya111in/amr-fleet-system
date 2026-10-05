// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// Hungarian 알고리즘 구현 (쌍대 포텐셜 u, v 를 유지하는 O(n^2 m) 변형).
//
// 행을 하나씩 추가하면서 감소 비용 c(i,j) - u(i) - v(j) >= 0 을 유지하고, 새 행에서 출발하는
// 최단 증가 경로(Dijkstra 와 같은 구조)를 찾아 매칭을 한 칸씩 늘린다. 경로를 찾는 동안
// 방문한 행/열의 포텐셜을 delta 만큼 옮겨 감소 비용 0 인 간선(타이트 간선)을 새로 만든다.

#include "amr_perception/hungarian.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace amr_perception
{

std::vector<int> solveAssignment(const Eigen::MatrixXd & cost)
{
  const int n = static_cast<int>(cost.rows());
  const int m = static_cast<int>(cost.cols());
  if (n == 0) {
    return {};
  }
  if (n > m) {
    throw std::invalid_argument("solveAssignment: rows > cols (전치해서 호출할 것)");
  }
  if (!cost.allFinite()) {
    throw std::invalid_argument("solveAssignment: 비유한 비용 — 금지 쌍은 큰 유한값으로 표현");
  }

  const double inf = std::numeric_limits<double>::infinity();
  // 1-기반 인덱스: 열 0 은 가상 열(새 행의 출발점)
  std::vector<double> u(n + 1, 0.0);
  std::vector<double> v(m + 1, 0.0);
  std::vector<int> row_of_col(m + 1, 0);  // p[j]: 열 j 에 매칭된 행 (0 = 없음)
  std::vector<int> way(m + 1, 0);         // 증가 경로 역추적

  for (int i = 1; i <= n; ++i) {
    row_of_col[0] = i;
    int j0 = 0;
    std::vector<double> minv(m + 1, inf);
    std::vector<char> used(m + 1, 0);
    do {
      used[j0] = 1;
      const int i0 = row_of_col[j0];
      double delta = inf;
      int j1 = 0;
      for (int j = 1; j <= m; ++j) {
        if (used[j]) {
          continue;
        }
        const double reduced = cost(i0 - 1, j - 1) - u[i0] - v[j];
        if (reduced < minv[j]) {
          minv[j] = reduced;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= m; ++j) {
        if (used[j]) {
          u[row_of_col[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (row_of_col[j0] != 0);
    // 증가 경로를 따라 매칭 뒤집기
    do {
      const int j1 = way[j0];
      row_of_col[j0] = row_of_col[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assignment(n, -1);
  for (int j = 1; j <= m; ++j) {
    if (row_of_col[j] != 0) {
      assignment[row_of_col[j] - 1] = j - 1;
    }
  }
  return assignment;
}

double assignmentCost(const Eigen::MatrixXd & cost, const std::vector<int> & assignment)
{
  double total = 0.0;
  for (std::size_t i = 0; i < assignment.size(); ++i) {
    if (assignment[i] >= 0) {
      total += cost(static_cast<Eigen::Index>(i), assignment[i]);
    }
  }
  return total;
}

}  // namespace amr_perception
