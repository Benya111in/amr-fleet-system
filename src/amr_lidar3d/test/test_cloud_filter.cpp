// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// filterNonFinite 단위 시험 — Fortress gpu_lidar 배치
// (x·y·z·intensity FLOAT32, ring UINT16, point_step 32)를
// 흉내 낸 조직 점군으로 비유한 점 제거, 순서·ring 보존, is_dense, frame_id, 배치 검사를 본다.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "amr_lidar3d/cloud_filter.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

using amr_lidar3d::FilterError;
using amr_lidar3d::FilterStats;
using amr_lidar3d::filterNonFinite;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

struct Pt
{
  float x, y, z, intensity;
  uint16_t ring;
};

PointField field(const std::string & name, uint32_t offset, uint8_t type)
{
  PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  return f;
}

// Gazebo 와 같은 배치: x 0, y 4, z 8, intensity 16, ring 24, point_step 32 (보고서·sensor.md)
PointCloud2 gazeboCloud(
  const std::vector<Pt> & pts, uint32_t width, uint32_t height,
  uint32_t row_pad = 0)
{
  PointCloud2 c;
  c.header.frame_id = "amr_01/lidar3d_link";
  c.header.stamp.sec = 12;
  c.header.stamp.nanosec = 345;
  c.fields = {field("x", 0, PointField::FLOAT32), field("y", 4, PointField::FLOAT32),
    field("z", 8, PointField::FLOAT32), field("intensity", 16, PointField::FLOAT32),
    field("ring", 24, PointField::UINT16)};
  c.point_step = 32;
  c.width = width;
  c.height = height;
  c.row_step = width * c.point_step + row_pad;
  c.is_dense = false;
  c.data.assign(static_cast<std::size_t>(c.row_step) * height, 0xAB);   // 패딩 바이트 표식
  for (uint32_t r = 0; r < height; ++r) {
    for (uint32_t col = 0; col < width; ++col) {
      const Pt & p = pts[r * width + col];
      uint8_t * d = c.data.data() + r * c.row_step + col * c.point_step;
      std::memcpy(d + 0, &p.x, 4);
      std::memcpy(d + 4, &p.y, 4);
      std::memcpy(d + 8, &p.z, 4);
      std::memcpy(d + 16, &p.intensity, 4);
      std::memcpy(d + 24, &p.ring, 2);
    }
  }
  return c;
}

Pt read(const PointCloud2 & c, std::size_t i)
{
  Pt p{};
  const uint8_t * d = c.data.data() + i * c.point_step;
  std::memcpy(&p.x, d + 0, 4);
  std::memcpy(&p.y, d + 4, 4);
  std::memcpy(&p.z, d + 8, 4);
  std::memcpy(&p.intensity, d + 16, 4);
  std::memcpy(&p.ring, d + 24, 2);
  return p;
}

// 2 행(ring 0, 1) × 3 열, 4 점이 비유한
std::vector<Pt> mixedPoints()
{
  return {
    {1.0F, 0.0F, -0.1F, 0.5F, 0}, {kInf, kInf, kInf, 0.0F, 0}, {2.0F, 1.0F, -0.2F, 0.7F, 0},
    {-kInf, 1.0F, 1.0F, 0.0F, 1}, {3.0F, kNaN, 0.3F, 0.0F, 1}, {4.0F, -1.0F, kInf, 0.0F, 1},
  };
}

}  // namespace

TEST(CloudFilter, DropsNonFiniteKeepsOrderAndRing)
{
  const PointCloud2 in = gazeboCloud(mixedPoints(), 3, 2);
  PointCloud2 out;
  FilterStats st;
  ASSERT_EQ(filterNonFinite(in, "", out, st), FilterError::kOk);
  EXPECT_EQ(st.input_points, 6U);
  EXPECT_EQ(st.kept_points, 2U);
  EXPECT_EQ(st.dropped_points, 4U);
  EXPECT_EQ(out.height, 1U);
  EXPECT_EQ(out.width, 2U);
  EXPECT_EQ(out.point_step, 32U);
  EXPECT_EQ(out.row_step, 64U);
  EXPECT_EQ(out.data.size(), 64U);
  EXPECT_TRUE(out.is_dense);
  EXPECT_EQ(out.fields, in.fields);           // ring 포함 필드 배치 그대로
  EXPECT_EQ(out.header, in.header);           // 스탬프·frame 유지 (override 없음)
  const Pt a = read(out, 0);
  const Pt b = read(out, 1);
  EXPECT_FLOAT_EQ(a.x, 1.0F);
  EXPECT_FLOAT_EQ(a.intensity, 0.5F);
  EXPECT_EQ(a.ring, 0U);
  EXPECT_FLOAT_EQ(b.x, 2.0F);
  EXPECT_FLOAT_EQ(b.z, -0.2F);
  EXPECT_EQ(b.ring, 0U);
}

TEST(CloudFilter, AllFiniteIsCopiedAndMarkedDense)
{
  std::vector<Pt> pts;
  for (uint16_t ring = 0; ring < 16; ++ring) {
    for (int col = 0; col < 4; ++col) {
      pts.push_back({static_cast<float>(col), 1.0F, 0.1F * ring, 1.0F, ring});
    }
  }
  const PointCloud2 in = gazeboCloud(pts, 4, 16);
  PointCloud2 out;
  FilterStats st;
  ASSERT_EQ(filterNonFinite(in, "", out, st), FilterError::kOk);
  EXPECT_EQ(st.kept_points, 64U);
  EXPECT_EQ(out.data, in.data);              // 패딩 없으면 바이트 그대로
  EXPECT_TRUE(out.is_dense);
  EXPECT_EQ(read(out, 63).ring, 15U);
}

TEST(CloudFilter, AllInfiniteGivesEmptyDenseCloud)
{
  const std::vector<Pt> pts(8, Pt{kInf, kInf, kInf, 0.0F, 3});
  PointCloud2 out;
  FilterStats st;
  ASSERT_EQ(filterNonFinite(gazeboCloud(pts, 8, 1), "", out, st), FilterError::kOk);
  EXPECT_EQ(out.width, 0U);
  EXPECT_EQ(out.height, 1U);
  EXPECT_TRUE(out.data.empty());
  EXPECT_TRUE(out.is_dense);
  EXPECT_EQ(st.dropped_points, 8U);
}

TEST(CloudFilter, RowPaddingIsSkipped)
{
  const PointCloud2 in = gazeboCloud(mixedPoints(), 3, 2, 16);   // 행 끝 16 B 패딩
  PointCloud2 out;
  FilterStats st;
  ASSERT_EQ(filterNonFinite(in, "", out, st), FilterError::kOk);
  ASSERT_EQ(out.width, 2U);
  EXPECT_FLOAT_EQ(read(out, 1).x, 2.0F);
  EXPECT_EQ(out.row_step, out.width * out.point_step);
}

TEST(CloudFilter, FrameIdOverride)
{
  PointCloud2 out;
  FilterStats st;
  ASSERT_EQ(
    filterNonFinite(gazeboCloud(mixedPoints(), 3, 2), "amr_01/lidar3d_link_fix", out, st),
    FilterError::kOk);
  EXPECT_EQ(out.header.frame_id, "amr_01/lidar3d_link_fix");
  EXPECT_EQ(out.header.stamp.sec, 12);
}

TEST(CloudFilter, Float64Xyz)
{
  PointCloud2 c;
  c.fields = {field("x", 0, PointField::FLOAT64), field("y", 8, PointField::FLOAT64),
    field("z", 16, PointField::FLOAT64)};
  c.point_step = 24;
  c.width = 2;
  c.height = 1;
  c.row_step = 48;
  const double v[6] = {1.0, 2.0, 3.0, std::numeric_limits<double>::infinity(), 0.0, 0.0};
  c.data.resize(48);
  std::memcpy(c.data.data(), v, sizeof(v));
  PointCloud2 out;
  FilterStats st;
  ASSERT_EQ(filterNonFinite(c, "", out, st), FilterError::kOk);
  EXPECT_EQ(out.width, 1U);
}

TEST(CloudFilter, RejectsBadInput)
{
  FilterStats st;
  PointCloud2 out;
  out.header.frame_id = "untouched";

  PointCloud2 no_z = gazeboCloud(mixedPoints(), 3, 2);
  no_z.fields.erase(no_z.fields.begin() + 2);
  EXPECT_EQ(filterNonFinite(no_z, "", out, st), FilterError::kMissingXyz);

  PointCloud2 int_xyz = gazeboCloud(mixedPoints(), 3, 2);
  int_xyz.fields[1].datatype = PointField::INT32;
  EXPECT_EQ(filterNonFinite(int_xyz, "", out, st), FilterError::kUnsupportedType);

  PointCloud2 mixed = gazeboCloud(mixedPoints(), 3, 2);
  mixed.fields[2].datatype = PointField::FLOAT64;
  EXPECT_EQ(filterNonFinite(mixed, "", out, st), FilterError::kUnsupportedType);

  PointCloud2 vec = gazeboCloud(mixedPoints(), 3, 2);
  vec.fields[0].count = 3;
  EXPECT_EQ(filterNonFinite(vec, "", out, st), FilterError::kUnsupportedType);

  PointCloud2 short_data = gazeboCloud(mixedPoints(), 3, 2);
  short_data.data.resize(short_data.data.size() - 1);
  EXPECT_EQ(filterNonFinite(short_data, "", out, st), FilterError::kBadLayout);

  PointCloud2 bad_row = gazeboCloud(mixedPoints(), 3, 2);
  bad_row.row_step = 3 * 32 - 1;
  EXPECT_EQ(filterNonFinite(bad_row, "", out, st), FilterError::kBadLayout);

  PointCloud2 field_out = gazeboCloud(mixedPoints(), 3, 2);
  field_out.fields[4].offset = 31;     // UINT16 이 point_step 32 를 넘는다
  EXPECT_EQ(filterNonFinite(field_out, "", out, st), FilterError::kBadLayout);

  PointCloud2 zero_step = gazeboCloud(mixedPoints(), 3, 2);
  zero_step.point_step = 0;
  EXPECT_EQ(filterNonFinite(zero_step, "", out, st), FilterError::kBadLayout);

  PointCloud2 big = gazeboCloud(mixedPoints(), 3, 2);
  big.is_bigendian = true;
  EXPECT_EQ(filterNonFinite(big, "", out, st), FilterError::kBigEndian);

  EXPECT_EQ(out.header.frame_id, "untouched");   // 오류면 출력을 손대지 않는다
  EXPECT_STREQ(amr_lidar3d::toString(FilterError::kOk), "ok");
}
