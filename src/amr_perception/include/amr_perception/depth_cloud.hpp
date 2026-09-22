// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 깊이 이미지 → 점군 역투영 + 거리 제곱 노이즈 + 거리 컷 + voxel 다운샘플 (ROS 비의존).
//
// 광학 프레임(x 우, y 하, z 전방) Pinhole 역투영: X = (u - cx)·Z/fx, Y = (v - cy)·Z/fy, Z = depth.
// 노이즈 (config/sensors.yaml depth_camera): 시뮬레이터가 σ_base 를 이미 넣었고, 이 단계에서
// 픽셀별 N(0, (k·Z²)²) 를 가산한다 → 합성 σ(Z) = √(σ_base² + (k·Z²)²).
// voxel 다운샘플: leaf 크기 격자의 칸마다 점 평균(무게중심) 하나. leaf <= 0 이면 다운샘플 없음.
// 설계 결정: Fortress 가 내는 camera/depth/points 는 좌표 규약이 optical 이 아니어서 쓰지 않고
// 깊이 이미지 + camera_info 로 직접 만든다 (docs/algorithms/perception.md §4).

#ifndef AMR_PERCEPTION__DEPTH_CLOUD_HPP_
#define AMR_PERCEPTION__DEPTH_CLOUD_HPP_

#include <cstdint>
#include <random>
#include <vector>

namespace amr_perception
{

struct PinholeIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  bool valid() const {return fx > 0.0 && fy > 0.0;}
};

struct DepthCloudParams
{
  double range_min{0.20};              ///< [m] sensors.yaml depth_camera.range_min
  double max_range{5.0};               ///< [m] 거리 컷 (지역 costmap voxel layer 범위)
  double leaf_size{0.05};              ///< [m] voxel 한 변, <= 0 이면 다운샘플 없음
  double noise_quadratic_coeff{0.002};  ///< k [1/m], σ = k·Z²
  bool add_noise{true};
  int pixel_step{1};                   ///< 픽셀 부표본 간격 (1 = 전체)
  int min_points_per_voxel{1};         ///< voxel 에 이 수 미만이면 버림 (고립 잡음 제거)
};

struct Point3
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

/// 깊이 인코딩
enum class DepthEncoding
{
  kFloat32Meters,   ///< 32FC1 [m]
  kUint16Millimeters,  ///< 16UC1 [mm]
};

/// 깊이 이미지 한 장을 광학 프레임 점군으로. data: 행 시작 바이트 포인터, step: 행 바이트 수.
std::vector<Point3> depthImageToPoints(
  const uint8_t * data, int width, int height, int step, DepthEncoding encoding,
  const PinholeIntrinsics & intrinsics, const DepthCloudParams & params, std::mt19937 * rng);

/// voxel 격자 무게중심 다운샘플
std::vector<Point3> voxelDownsample(
  const std::vector<Point3> & points, double leaf_size, int min_points_per_voxel);

/// 픽셀 (u, v) + 깊이 Z → 광학 프레임 점
Point3 backProject(double u, double v, double z, const PinholeIntrinsics & intrinsics);

}  // namespace amr_perception

#endif  // AMR_PERCEPTION__DEPTH_CLOUD_HPP_
