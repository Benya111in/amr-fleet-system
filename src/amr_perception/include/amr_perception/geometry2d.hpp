// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 2D 기하 도우미 (ROS 비의존): 강체 변환, 각도 정규화, 풋프린트/다각형 거리.
// 추적기(odom 평면), 안전 노드(base_link 평면), TTC 가 공통으로 쓴다.

#ifndef AMR_PERCEPTION__GEOMETRY2D_HPP_
#define AMR_PERCEPTION__GEOMETRY2D_HPP_

#include <Eigen/Core>

#include <vector>

namespace amr_perception
{

using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;

/// 평면 강체 변환 T = (x, y, yaw). 점 변환 p' = R(yaw) p + t.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  /// 점 변환 (회전 + 평행이동)
  Vec2 apply(const Vec2 & p) const;
  /// 벡터 회전만 (속도, 방향)
  Vec2 rotate(const Vec2 & v) const;
  /// 회전 행렬 R(yaw)
  Mat2 rotation() const;
  /// this ∘ other : other 프레임의 점을 this 의 부모 프레임으로
  Pose2D compose(const Pose2D & other) const;
  /// 역변환
  Pose2D inverse() const;
  Vec2 translation() const {return Vec2(x, y);}
};

/// 각도를 (-pi, pi] 로 정규화
double normalizeAngle(double angle);

/// 원점 중심 축 정렬 사각형 [-L/2, L/2] x [-W/2, W/2] 의 변까지 최단 거리. 내부 점은 0.
double distanceToRectangle(const Vec2 & p, double length, double width);

/// 점 - 선분 최단 거리
double distanceToSegment(const Vec2 & p, const Vec2 & a, const Vec2 & b);

/// 점이 다각형(꼭짓점 순서 무관, 단순 다각형) 내부인가 (ray casting)
bool pointInPolygon(const Vec2 & p, const std::vector<Vec2> & polygon);

/// 점 - 다각형 변 최단 거리. 내부 점은 0.
double distanceToPolygon(const Vec2 & p, const std::vector<Vec2> & polygon);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__GEOMETRY2D_HPP_
