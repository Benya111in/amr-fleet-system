// R2.5 — 비용 함수 진단기. Gazebo 없이 DwaPlanner::compute 를 직접 불러
// **어느 비용항이 argmin 을 결정하는지** 본다.
//
// 왜 필요한가: 통합 시험 한 라운드가 55 분이다. 비용 함수 설계는 수십 번 돌려봐야 하므로
// 그 루프를 오프라인으로 옮긴다. 장면은 실측에서 접촉의 86.5 % 를 차지하는 기하를 쓴다 —
// 직선 경로 위를 가는 로봇과 1.0 m/s 로 직교 횡단하는 보행자(worker_crossing).
//
// 사용: cost_probe [--sustained] [--vmeas V] [--dy D]
//   --sustained  R1+R2(지속 목표 명령 + 램프 롤아웃)를 켠다
//   --vmeas      로봇 현재 속도 [m/s] (실측 접촉의 52 % 가 < 0.05)
//   --dy         보행자가 경로를 가로지르기까지 남은 가로 거리 [m]
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/dwa.hpp"
#include "amr_navigation/core/footprint.hpp"

using amr_navigation::core::DwaConfig;
using amr_navigation::core::DwaInput;
using amr_navigation::core::DwaPlanner;
using amr_navigation::core::DwaResult;
using amr_navigation::core::DynamicObstacle;
using amr_navigation::core::FootprintChecker;
using amr_navigation::core::OwnedGrid;
using amr_navigation::core::Pose2D;
using amr_navigation::core::inflate;
using amr_navigation::core::inflationCost;

namespace
{
const char * kTermNames[8] = {
  "heading", "clearance", "velocity", "path", "oscillation", "dynamic", "off_path", "escape"};

std::vector<Pose2D> straightPath(double len)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= len + 1e-9; s += 0.05) {
    p.push_back({s, 0.0, 0.0});
  }
  return p;
}

void report(const char * label, const DwaResult & r)
{
  std::printf("\n== %s ==\n", label);
  std::printf(
    "  선택: v = %.4f  w = %.4f   (found=%d, vo_saturated=%d)\n",
    r.v, r.w, static_cast<int>(r.found), static_cast<int>(r.vo_saturated));
  std::printf(
    "  후보: %zu 개 중 선택가능 %zu · 충돌없음 %zu · VO기각 %zu\n",
    r.n_samples, r.diag.n_selectable, r.diag.n_collision_free, r.n_vo_rejected);
  std::printf(
    "  표현력: disp_span %.4f m · v [%.3f, %.3f] · w [%.3f, %.3f]\n",
    r.diag.disp_span, r.diag.v_lo, r.diag.v_hi, r.diag.w_lo, r.diag.w_hi);
  std::printf("  비용 폭(전체): %.4f\n", r.diag.cost_max - r.diag.cost_min);
  std::printf("  항별 **가중** 기여 폭 (큰 것이 argmin 을 결정한다):\n");
  std::array<std::size_t, 8> idx{{0, 1, 2, 3, 4, 5, 6, 7}};
  std::sort(
    idx.begin(), idx.end(),
    [&r](std::size_t a, std::size_t b) {return r.diag.term_span[a] > r.diag.term_span[b];});
  double tot = 0.0;
  for (double x : r.diag.term_span) {
    tot += x;
  }
  for (std::size_t i : idx) {
    const double v = r.diag.term_span[i];
    std::printf(
      "    %-12s %8.4f  %5.1f %%%s\n", kTermNames[i], v,
      tot > 1e-12 ? 100.0 * v / tot : 0.0, v > 0.3 * tot ? "   <- 지배" : "");
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  bool sustained = false;
  double v_meas = 0.0, dy = 2.0, aisle = 0.0;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--sustained") == 0) {
      sustained = true;
    } else if (std::strcmp(argv[i], "--vmeas") == 0 && i + 1 < argc) {
      v_meas = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--dy") == 0 && i + 1 < argc) {
      dy = std::stod(argv[++i]);
    } else if (std::strcmp(argv[i], "--aisle") == 0 && i + 1 < argc) {
      aisle = std::stod(argv[++i]);
    }
  }

  OwnedGrid grid{300, 300, 0.05, 0, -7.5, -7.5};
  if (aisle > 0.0) {
    // 좁은 통로: 경로 양옆에 벽을 세운다 (순폭 aisle). 소크가 도는 랙 사이 통로 모사.
    auto v = grid.view();
    for (int gx = 0; gx < 300; ++gx) {
      for (int gy = 0; gy < 300; ++gy) {
        const double wx = -7.5 + (gx + 0.5) * 0.05, wy = -7.5 + (gy + 0.5) * 0.05;
        if (wx > -1.0 && wx < 9.0 && std::abs(wy) > aisle / 2 && std::abs(wy) < aisle / 2 + 0.3) {
          grid.at(gx, gy) = 254;
        }
      }
    }
  }
  inflate(grid, 0.2, 0.8, 3.0);
  const auto fp = FootprintChecker::rectangle(0.60, 0.40);
  const FootprintChecker chk(
    grid.view(), fp, inflationCost(FootprintChecker::circumscribedRadius(fp), 0.2, 3.0));
  const auto view = grid.view();
  const auto path = straightPath(8.0);

  DwaConfig cfg;
  cfg.sustained_sampling = sustained;

  DwaInput in;
  in.pose = {0.0, 0.0, 0.0};
  in.v_meas = v_meas;
  in.w_meas = 0.0;
  in.has_last = true;
  in.v_last = v_meas;
  in.w_last = 0.0;
  in.path = &path;
  // 직교 횡단 보행자: 로봇 앞 2.0 m, 가로로 dy 떨어진 곳에서 경로를 향해 1.0 m/s
  DynamicObstacle ped;
  ped.x = 2.0;
  ped.y = dy;
  ped.vx = 0.0;
  ped.vy = -1.0;
  ped.radius = 0.25;
  ped.id = 1;
  in.obstacles.push_back(ped);

  std::printf(
    "장면: 직선 경로, 보행자 (2.0, %.2f) 에서 1.0 m/s 로 직교 횡단. v_meas = %.2f\n"
    "표본 방식: %s\n", dy, v_meas, sustained ? "지속 목표 명령 + 램프" : "1주기 창 + 등속");

  const DwaPlanner dwa(cfg);
  const DwaResult r = dwa.compute(
    in, [&chk](const Pose2D & p) {return chk.cost(p);},
    [view](double x, double y) {return static_cast<double>(view.costAtWorld(x, y));});
  report(sustained ? "지속 목표 명령" : "1주기 창", r);
  return 0;
}
