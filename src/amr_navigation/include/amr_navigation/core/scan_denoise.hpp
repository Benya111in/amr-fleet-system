// 코스트맵용 LiDAR 거리 잡음 억제 — 같은 표면 이웃 빔의 게이트 중앙값 (ROS 비의존).
//
// 문제 (docs/algorithms/costmap.md §3): LiDAR σ = 0.03 m 이면 벽 끝점이 빔마다 ±3σ = ±0.09 m
// 흩어지고, 코스트맵은 끝점 하나하나를 치명 셀로 찍는다. 0.60 m 통로(측면 여유 0.10 m)에서는
// 매 스캔 수 개의 끝점이 0.05 m 이상 안쪽으로 들어와 로봇 외곽선 셀이 치명이 된다
// (리뷰 실측: 통로 안 치명 셀 26–51 개, 침범 0.075 m).
// 빔 i 의 거리를 ±k 빔 창에서 |r_j − r_i| ≤ gate 인 빔(= 같은 표면)들의 중앙값으로 바꾼다:
//   - 수직 입사 벽: 이웃 빔 간격이 수 mm 라 창 전체가 같은 표면
//     → σ_med ≈ 1.25·σ/√(2k+1) (k 5 → 0.011 m)
//   - 스치는 입사: 이웃 거리 차가 커 게이트에 걸리지만, 이때 벽 수직 방향 잡음은
//     σ·sin(입사각) 로 이미 작다
//   - 얇은 물체(빔 1–2 개)·거리 불연속: 게이트가 배경 빔을 빼므로 물체 빔끼리만 중앙값
//     → 지워지지 않는다.
//     지지 빔이 min_support 보다 적으면 원래 값 그대로 (단발 반사도 장애물로 남긴다 — 안전 쪽)
// 비유한 값(NaN, ±inf)은 이웃에서 빼고 그 자리는 그대로 둔다.
#ifndef AMR_NAVIGATION__CORE__SCAN_DENOISE_HPP_
#define AMR_NAVIGATION__CORE__SCAN_DENOISE_HPP_

#include <cstddef>
#include <vector>

#include "amr_navigation/core/geometry.hpp"

namespace amr_navigation
{
namespace core
{

struct ScanDenoiseConfig
{
  int half_window{5};         // k: ±k 빔 (0.5° 간격이면 ±2.5°)
  double range_gate{0.15};    // [m] 같은 표면 판정 |r_j − r_i| (σ 0.03 의 두 빔 차 σ√2 의 3.5 배)
  int min_support{3};         // 자기 포함 지지 빔 수가 이보다 적으면 원래 값
};

/// 게이트 중앙값 필터. wrap = true 면 첫·끝 빔이 이웃 (360° 스캔).
std::vector<float> denoiseRanges(
  const std::vector<float> & ranges, const ScanDenoiseConfig & config, bool wrap);

/// 동적 트랙 제외 (전역 코스트맵 입력): 끝점이 원판(센서 프레임 중심 centers, 반경 radius)
/// 안인 빔을 +inf 로 바꾼다. 지나가는 사람·차량은 지역 계획기(DWA VO/TTC + 지역 코스트맵)가
/// 맡고, 전역 경로가 그 순간 위치를 돌아가며 매 재계획마다 뒤집히는 것을 막는다
/// (costmap.md §5.2). 반환: 바꾼 빔 수.
std::size_t excludeDiscsFromScan(
  std::vector<float> & ranges, double angle_min, double angle_increment,
  const std::vector<Point2D> & centers, double radius);

}  // namespace core
}  // namespace amr_navigation

#endif  // AMR_NAVIGATION__CORE__SCAN_DENOISE_HPP_
