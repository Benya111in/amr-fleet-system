// A* 격자 경로 후처리 — ROS 비의존 (명세 4.4 "경로 평활화").
//
// 1) 비용 인지 숏컷: 기준점 i 에서 j 를 늘려 가며 현(chord)
//   p_i→p_j 가 (a) supercover 셀이 모두 통행 가능
//    (≤ max_allowed_cost) 하고, (b) 현이 지나는 셀의 최대 비용
//    ≤ 원래 격자 경로 구간 [i, j] 의 최대 셀 비용
//    + clearance_cost_margin (여유거리 천장), (c) 현을 따라
//    적분한 A* 비용 ∫m(c)ds 가 원래 격자 경로 비용
//    G_j − G_i 의 (1+δ) 배 이하인 동안 전진, 처음 실패하면 직전 j 를 채택한다. (b) 로 모서리를
//    가로질러 장애물에 붙는 현이 막히고, (c) 로 inflation 띠를 길게 가로지르는 현이 막힌다.
//    8-연결 격자의 지그재그(최대 8.2 % 초과 길이)를 없앤다.
// 2) 등간격 재표본화 (output_spacing).
// 3) Gauss–Seidel 경사 하강 평활화 (양 끝 고정):
//      y_i ← y_i + w_d (x_i − y_i) + w_s (y_{i−1} + y_{i+1} − 2 y_i) + w_c · r · s_i · n̂_i
//    x = 숏컷 경로, n̂ = −∇c/|∇c| (코스트맵 비용이 줄어드는
//    방향), s_i = max(0, c − c_thr)/(252 − c_thr).
//    선형 부분은 SOR 로 w_d > 0, 0 < w_d + 2 w_s < 2 에서 수렴한다(docs/algorithms/astar.md).
//    점별 사영(projection): 갱신 후 점 i 의 셀 비용과 이웃
//    두 선분의 최대 셀 비용이 입력 경로의 같은 자리
//    천장 ceil_i(점 i 에 닿는 입력 선분 두 개의 최대 비용)
//    를 넘으면 그 점은 움직이지 않는다 → 평활화가
//    여유거리를 줄이지 않는다. 매 스윕 뒤 전체 선분 통행 가능성도
//    검사해 실패하면 직전 스윕으로 되돌린다.
// 4) 방향: 내부 점은 중심차분 접선, 마지막 점은 목표 방향(옵션).
// 보장(단위 테스트): 평활 경로의 최대 셀 비용 ≤ 원래 격자 경로의 최대 셀 비용 +
//   clearance_cost_margin.
#ifndef AMR_NAVIGATION__CORE__PATH_SMOOTHER_HPP_
#define AMR_NAVIGATION__CORE__PATH_SMOOTHER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"

namespace amr_navigation
{
namespace core
{

struct SmootherConfig
{
  bool enable_shortcut{true};
  double shortcut_max_length{10.0};     // [m] 현 길이 상한
  double shortcut_cost_ratio{0.05};     // δ
  double output_spacing{0.05};          // [m] 재표본 간격 (≤ 0 이면 격자 해상도)
  bool enable_smoothing{true};
  double w_data{0.2};
  double w_smooth{0.3};
  double w_clearance{0.3};
  uint8_t clearance_cost_threshold{100};  // 이 비용보다 높은 점만 밀어낸다
  /// 숏컷 현의 여유거리 천장 여유 [비용 단위]. 10 ≈ c≈100,
  ///   cost_scaling 2.0 에서 여유거리 1 셀(0.05 m).
  int clearance_cost_margin{10};
  int max_iterations{100};
  double tolerance{1e-4};               // [m] 한 스윕의 총 이동량이 이보다 작으면 수렴
  // 롤백 가드/숏컷 통행 기준 (미지 255 는 allow_unknown 에 따름)
  uint8_t max_allowed_cost{kMaxNonObstacle};
  bool allow_unknown{true};
};

struct SmootherStats
{
  double raw_length{0.0};
  double shortcut_length{0.0};
  double smoothed_length{0.0};
  std::size_t raw_points{0};
  std::size_t shortcut_points{0};
  std::size_t output_points{0};
  int iterations{0};
  bool rolled_back{false};
};

class PathSmoother
{
public:
  explicit PathSmoother(const SmootherConfig & config = SmootherConfig());

  void setConfig(const SmootherConfig & config) {config_ = config;}
  const SmootherConfig & config() const {return config_;}

  /// 격자 경로 → 평활 자세 목록. multiplier 는 A* 의 통행 배율 표(숏컷 비용 비교용).
  /// start/goal 이 주어지면 첫/끝 점을 그 정확한 좌표로 바꾼다. goal_yaw 는 마지막 자세 방향.
  std::vector<Pose2D> process(
    const CostGrid & grid, const std::vector<Cell> & cells,
    const std::array<float, 256> & multiplier, const Point2D * start, const Point2D * goal,
    const double * goal_yaw, SmootherStats * stats = nullptr) const;

  // --- 단계별 공개 함수 (단위 테스트용) ---
  static std::vector<Point2D> cellsToWorld(const CostGrid & grid, const std::vector<Cell> & cells);

  /// 비용 인지 숏컷. 채택된 점의 인덱스(첫·끝 포함)를 돌려준다.
  std::vector<std::size_t> shortcut(
    const CostGrid & grid, const std::vector<Point2D> & pts,
    const std::array<float, 256> & multiplier) const;

  static std::vector<Point2D> resample(const std::vector<Point2D> & pts, double spacing);

  /// Gauss–Seidel 평활화(제자리). 반환: 수행 스윕 수. rolled_back 에 롤백 여부.
  int smooth(const CostGrid & grid, std::vector<Point2D> & pts, bool & rolled_back) const;

  /// 선분의 supercover 셀이 모두 통행 가능(≤ max_allowed_cost)한가.
  bool segmentFree(const CostGrid & grid, const Point2D & a, const Point2D & b) const;

  /// 선분의 supercover 셀 최대 비용 (미지 셀: allow_unknown 이면 0, 아니면 255; 범위 밖 254).
  int segmentMaxCost(const CostGrid & grid, const Point2D & a, const Point2D & b) const;

  /// 선분을 따라 ∫ m(c) ds [셀 단위]. 통행 불가 셀을 만나면 +inf.
  static double segmentCost(
    const CostGrid & grid, const Point2D & a, const Point2D & b,
    const std::array<float, 256> & multiplier);

  static std::vector<Pose2D> assignOrientations(
    const std::vector<Point2D> & pts, const double * goal_yaw);

private:
  bool cellAllowed(uint8_t c) const;
  int cellValue(uint8_t c) const;

  SmootherConfig config_;
};

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__PATH_SMOOTHER_HPP_
