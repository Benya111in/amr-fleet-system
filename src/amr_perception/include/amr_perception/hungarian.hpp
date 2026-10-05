// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// Hungarian(Kuhn–Munkres) 최소 비용 할당 — 직접 구현 (ROS 비의존).
// 추적기의 확장 행렬 GNN 데이터 연관이 사용한다 (docs/algorithms/tracking.md §3).

#ifndef AMR_PERCEPTION__HUNGARIAN_HPP_
#define AMR_PERCEPTION__HUNGARIAN_HPP_

#include <Eigen/Core>

#include <vector>

namespace amr_perception
{

/// 비용 행렬 C (rows <= cols, 모든 원소 유한)의 최소 합 할당을 푼다.
/// 반환: 행 i 에 할당된 열 번호 (크기 rows). O(rows^2 · cols), 포텐셜(쌍대 변수) 기반.
/// 무한대 대신 큰 유한값(예: 1e6)으로 금지 쌍을 표현한다. rows > cols 또는 비유한 값이면
/// std::invalid_argument 를 던진다.
std::vector<int> solveAssignment(const Eigen::MatrixXd & cost);

/// 할당의 총비용 (검증/테스트용)
double assignmentCost(const Eigen::MatrixXd & cost, const std::vector<int> & assignment);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__HUNGARIAN_HPP_
