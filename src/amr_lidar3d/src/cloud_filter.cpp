// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 3D LiDAR 점군 전처리 순수 로직 — include/amr_lidar3d/cloud_filter.hpp 참고.

#include "amr_lidar3d/cloud_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "sensor_msgs/msg/point_field.hpp"

namespace amr_lidar3d
{

namespace
{

using sensor_msgs::msg::PointField;

const PointField * findField(const sensor_msgs::msg::PointCloud2 & cloud, const char * name)
{
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

std::size_t typeSize(uint8_t datatype)
{
  return datatype == PointField::FLOAT32 ? 4U : datatype == PointField::FLOAT64 ? 8U : 0U;
}

bool finiteAt(const uint8_t * p, uint8_t datatype)
{
  if (datatype == PointField::FLOAT32) {
    float v;
    std::memcpy(&v, p, sizeof(v));   // 정렬 가정 없이 읽는다
    return std::isfinite(v);
  }
  double v;
  std::memcpy(&v, p, sizeof(v));
  return std::isfinite(v);
}

}  // namespace

const char * toString(FilterError error)
{
  switch (error) {
    case FilterError::kOk: return "ok";
    case FilterError::kMissingXyz: return "x/y/z 필드 없음";
    case FilterError::kUnsupportedType: return "x/y/z 형 미지원 (FLOAT32/FLOAT64, count 1)";
    case FilterError::kBadLayout: return "필드·row_step·data 크기 불일치";
    case FilterError::kBigEndian: return "빅엔디언 미지원";
  }
  return "?";
}

FilterError filterNonFinite(
  const sensor_msgs::msg::PointCloud2 & in, const std::string & frame_id_override,
  sensor_msgs::msg::PointCloud2 & out, FilterStats & stats)
{
  const PointField * fx = findField(in, "x");
  const PointField * fy = findField(in, "y");
  const PointField * fz = findField(in, "z");
  if (fx == nullptr || fy == nullptr || fz == nullptr) {
    return FilterError::kMissingXyz;
  }
  const uint8_t type = fx->datatype;
  const std::size_t size = typeSize(type);
  if (size == 0U || fy->datatype != type || fz->datatype != type ||
    fx->count != 1U || fy->count != 1U || fz->count != 1U)
  {
    return FilterError::kUnsupportedType;
  }
  if (in.is_bigendian) {
    return FilterError::kBigEndian;
  }
  const std::size_t step = in.point_step;
  for (const auto & f : in.fields) {
    const std::size_t fsize = f.datatype == PointField::INT8 || f.datatype == PointField::UINT8 ?
      1U : f.datatype == PointField::INT16 || f.datatype == PointField::UINT16 ? 2U :
      f.datatype == PointField::FLOAT64 ? 8U : 4U;
    if (static_cast<std::size_t>(f.offset) + fsize * std::max<std::size_t>(f.count, 1U) > step) {
      return FilterError::kBadLayout;
    }
  }
  const std::size_t width = in.width;
  const std::size_t height = in.height;
  const std::size_t row_step = in.row_step;
  if (step == 0U || row_step < width * step || in.data.size() < row_step * height) {
    return FilterError::kBadLayout;
  }

  out.header = in.header;
  if (!frame_id_override.empty()) {
    out.header.frame_id = frame_id_override;
  }
  out.fields = in.fields;
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.height = 1U;
  out.data.resize(width * height * step);

  std::size_t kept = 0;
  const uint8_t * base = in.data.data();
  uint8_t * dst = out.data.data();
  for (std::size_t r = 0; r < height; ++r) {
    const uint8_t * row = base + r * row_step;
    for (std::size_t c = 0; c < width; ++c) {
      const uint8_t * p = row + c * step;
      if (finiteAt(p + fx->offset, type) && finiteAt(p + fy->offset, type) &&
        finiteAt(p + fz->offset, type))
      {
        std::memcpy(dst + kept * step, p, step);
        ++kept;
      }
    }
  }
  out.data.resize(kept * step);
  out.width = static_cast<uint32_t>(kept);
  out.row_step = static_cast<uint32_t>(kept * step);
  out.is_dense = true;

  stats.input_points = width * height;
  stats.kept_points = kept;
  stats.dropped_points = width * height - kept;
  return FilterError::kOk;
}

}  // namespace amr_lidar3d
