// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 깊이 이미지 점군 변환 구현.

#include "amr_perception/depth_cloud.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

namespace amr_perception
{

Point3 backProject(double u, double v, double z, const PinholeIntrinsics & k)
{
  Point3 p;
  p.x = static_cast<float>((u - k.cx) * z / k.fx);
  p.y = static_cast<float>((v - k.cy) * z / k.fy);
  p.z = static_cast<float>(z);
  return p;
}

std::vector<Point3> depthImageToPoints(
  const uint8_t * data, int width, int height, int step, DepthEncoding encoding,
  const PinholeIntrinsics & intrinsics, const DepthCloudParams & params, std::mt19937 * rng)
{
  std::vector<Point3> out;
  if (data == nullptr || width <= 0 || height <= 0 || !intrinsics.valid()) {
    return out;
  }
  const int stride = params.pixel_step > 0 ? params.pixel_step : 1;
  out.reserve(static_cast<std::size_t>(width / stride + 1) * (height / stride + 1));
  std::normal_distribution<double> unit(0.0, 1.0);
  for (int v = 0; v < height; v += stride) {
    const uint8_t * row = data + static_cast<std::size_t>(v) * step;
    for (int u = 0; u < width; u += stride) {
      double z = 0.0;
      if (encoding == DepthEncoding::kFloat32Meters) {
        float f;
        std::memcpy(&f, row + static_cast<std::size_t>(u) * sizeof(float), sizeof(float));
        z = static_cast<double>(f);
      } else {
        uint16_t mm;
        std::memcpy(&mm, row + static_cast<std::size_t>(u) * sizeof(uint16_t), sizeof(uint16_t));
        z = mm == 0 ? std::nan("") : static_cast<double>(mm) * 1e-3;
      }
      if (!std::isfinite(z) || z <= 0.0) {
        continue;
      }
      if (params.add_noise && rng != nullptr && params.noise_quadratic_coeff > 0.0) {
        z += params.noise_quadratic_coeff * z * z * unit(*rng);
      }
      if (z < params.range_min || z > params.max_range) {
        continue;
      }
      out.push_back(backProject(u, v, z, intrinsics));
    }
  }
  return out;
}

std::vector<Point3> voxelDownsample(
  const std::vector<Point3> & points, double leaf_size, int min_points_per_voxel)
{
  if (leaf_size <= 0.0) {
    return points;
  }
  struct Acc
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    int n{0};
    std::size_t order{0};
  };
  // 21 비트씩 부호 있는 격자 인덱스를 64 비트 키로 묶는다 (±1e6 칸 = 0.05 m 에서 ±52 km)
  auto key = [leaf_size](const Point3 & p) {
      const int64_t ix = static_cast<int64_t>(std::floor(p.x / leaf_size)) & 0x1FFFFF;
      const int64_t iy = static_cast<int64_t>(std::floor(p.y / leaf_size)) & 0x1FFFFF;
      const int64_t iz = static_cast<int64_t>(std::floor(p.z / leaf_size)) & 0x1FFFFF;
      return static_cast<uint64_t>((ix << 42) | (iy << 21) | iz);
    };
  std::unordered_map<uint64_t, Acc> grid;
  grid.reserve(points.size() / 4 + 1);
  for (const auto & p : points) {
    Acc & a = grid[key(p)];
    if (a.n == 0) {
      a.order = grid.size();
    }
    a.x += p.x;
    a.y += p.y;
    a.z += p.z;
    ++a.n;
  }
  std::vector<std::pair<std::size_t, Point3>> ordered;
  ordered.reserve(grid.size());
  for (const auto & kv : grid) {
    const Acc & a = kv.second;
    if (a.n < min_points_per_voxel) {
      continue;
    }
    Point3 c;
    c.x = static_cast<float>(a.x / a.n);
    c.y = static_cast<float>(a.y / a.n);
    c.z = static_cast<float>(a.z / a.n);
    ordered.emplace_back(a.order, c);
  }
  // 첫 등장 순서로 정렬 → 결과가 해시 순서와 무관하게 결정적
  std::sort(
    ordered.begin(), ordered.end(),
    [](const auto & a, const auto & b) {return a.first < b.first;});
  std::vector<Point3> out;
  out.reserve(ordered.size());
  for (const auto & o : ordered) {
    out.push_back(o.second);
  }
  return out;
}

}  // namespace amr_perception
