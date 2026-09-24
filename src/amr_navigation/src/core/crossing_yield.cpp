#include "amr_navigation/core/crossing_yield.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "amr_navigation/core/speed_profile.hpp"

namespace amr_navigation
{
namespace core
{
namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

// 통로 좌표계: 장애물 진행 방향 û 와 좌수직 n̂ 기준의 (α, β)
struct Corridor
{
  double ox{0.0};
  double oy{0.0};
  double ux{1.0};
  double uy{0.0};
  double nx{0.0};
  double ny{1.0};
  double R{1.0};      // 반폭 [m]
  double L{0.0};      // 지평 동안 쓸고 가는 길이 [m]

  double alpha(double x, double y) const {return (x - ox) * ux + (y - oy) * uy;}
  double beta(double x, double y) const {return (x - ox) * nx + (y - oy) * ny;}
  bool inside(double x, double y) const
  {
    const double a = alpha(x, y);
    return std::abs(beta(x, y)) <= R && a >= -R && a <= L + R;
  }
};
}  // namespace

double travelTime(double d, double v0, double a, double v_max)
{
  if (d <= 0.0) {
    return 0.0;
  }
  const double v = std::max(0.0, v0);
  const double vmax = std::max(1e-6, v_max);
  if (a <= 0.0) {
    return d / std::max(v, 1e-9);   // 가속이 없으면 지금 속도 그대로 (정지면 영영 못 간다)
  }
  if (v >= vmax) {
    return d / v;
  }
  const double d_acc = (vmax * vmax - v * v) / (2.0 * a);
  if (d <= d_acc) {
    return (-v + std::sqrt(v * v + 2.0 * a * d)) / a;
  }
  return (vmax - v) / a + (d - d_acc) / vmax;
}

YieldResult evaluateYield(
  const std::vector<Pose2D> & path, const std::vector<double> & cum_s, const Pose2D & robot,
  double s_robot, double v_robot, const std::vector<DynamicObstacle> & obstacles,
  const YieldConfig & cfg)
{
  YieldResult out;
  if (!cfg.enable || path.size() < 2 || cum_s.size() != path.size() || obstacles.empty()) {
    return out;
  }
  const double s_end = std::min(cum_s.back(), s_robot + cfg.lookahead);

  for (std::size_t idx = 0; idx < obstacles.size(); ++idx) {
    const DynamicObstacle & o = obstacles[idx];
    // 통로 축·길이는 평활 속도로 세운다 (velocity_obstacle.hpp 참고): 추적기 진행각이
    // 주기간 최대 144.9° 흔들려 그대로 쓰면 통로가 그만큼 돌고 교차 구간이 미터 단위로
    // 이동한다. VO·TTC 는 이 함수 밖에서 원시 속도를 그대로 쓴다.
    const double pvx = o.predVx();
    const double pvy = o.predVy();
    const double su = std::hypot(pvx, pvy);
    if (su < cfg.min_speed) {
      continue;   // 정지 물체는 코스트맵이 처리한다
    }
    // 장애물까지의 거리로는 거르지 않는다: 교차 구간이 경로 창(lookahead) 안에 있는지와 통로
    // 길이(|u|·horizon)가 거름망이고, 멀리 있는 장애물은 "먼저 빠져나간다" 판정에서 풀린다.
    Corridor c;
    c.ox = o.x;
    c.oy = o.y;
    c.ux = pvx / su;
    c.uy = pvy / su;
    c.nx = -c.uy;
    c.ny = c.ux;
    c.R = cfg.robot_radius + o.radius + cfg.corridor_margin;
    c.L = su * cfg.horizon;

    // --- 경로 위 교차 구간 [s_in, s_out] (로봇 앞 첫 연속 구간) ---
    bool found = false;
    bool open = false;          // 훑기가 통로 안에서 끝났다 (구간이 창 밖으로 이어진다)
    double s_in = 0.0;
    double s_out = 0.0;
    double a_min = kInf;
    double a_max = -kInf;
    std::size_t last_i = 0;
    for (std::size_t i = 0; i < path.size(); ++i) {
      if (cum_s[i] < s_robot) {
        continue;
      }
      if (cum_s[i] > s_end) {
        break;
      }
      const double a = c.alpha(path[i].x, path[i].y);
      if (c.inside(path[i].x, path[i].y)) {
        if (!found) {
          found = true;
          open = true;
          // 입구는 바로 앞 정점으로 잡는다 (0.05 m 경로 간격에서 보수적으로 일찍 선다)
          s_in = i > 0 ? std::max(s_robot, cum_s[i - 1]) : cum_s[i];
        }
        s_out = cum_s[i];
        a_min = std::min(a_min, a);
        a_max = std::max(a_max, a);
        last_i = i;
      } else if (found) {
        open = false;
        s_out = cum_s[i];
        a_min = std::min(a_min, a);
        a_max = std::max(a_max, a);
        break;
      }
    }
    if (!found) {
      continue;
    }
    if (open) {
      // 창 끝까지 통로 안이다 — 경로 접선으로 출구를 외삽한다
      const Pose2D & p0 = path[last_i > 0 ? last_i - 1 : last_i];
      const Pose2D & p1 = path[last_i + 1 < path.size() ? last_i + 1 : last_i];
      const double seg = std::hypot(p1.x - p0.x, p1.y - p0.y);
      if (seg < 1e-9) {
        continue;
      }
      const double tx = (p1.x - p0.x) / seg;
      const double ty = (p1.y - p0.y) / seg;
      const double db = tx * c.nx + ty * c.ny;       // dβ/ds
      if (std::abs(db) < 1e-3) {
        continue;   // 통로와 나란한 경로 — 정지선으로 풀 문제가 아니다 (VO/TTC 가 맡는다)
      }
      const double b_last = c.beta(path[last_i].x, path[last_i].y);
      const double ds = ((db > 0.0 ? c.R : -c.R) - b_last) / db;
      if (!(ds > 0.0) || (s_out + ds) - s_in > cfg.max_zone) {
        continue;   // 교차 구간이 너무 길다 = 사실상 나란한 주행
      }
      const double da = tx * c.ux + ty * c.uy;       // dα/ds
      const double a_exit = c.alpha(path[last_i].x, path[last_i].y) + da * ds;
      a_min = std::min(a_min, a_exit);
      a_max = std::max(a_max, a_exit);
      s_out += ds;
    }
    if (s_out - s_in > cfg.max_zone) {
      continue;
    }

    const double d_in = s_in - s_robot;
    const double d_out = s_out - s_robot;
    if (d_out <= 0.0) {
      continue;   // 교차 구간이 로봇 뒤에 있다
    }
    // 장애물이 교차 구간을 점유하는 시각 [t_in, t_out]
    const double t_in = (a_min - c.R) / su;
    const double t_out = (a_max + c.R) / su;
    if (t_out <= 0.0) {
      continue;   // 장애물이 이미 지나갔다 (뒤로 지나간 경우)
    }
    const double t_r_in = travelTime(std::max(0.0, d_in), v_robot, cfg.accel, cfg.v_max);
    const double t_r_out = travelTime(std::max(0.0, d_out), v_robot, cfg.accel, cfg.v_max);
    if (t_r_out + cfg.clear_margin < t_in) {
      continue;   // 로봇이 먼저 빠져나간다
    }
    if (t_out + cfg.clear_margin < t_r_in) {
      continue;   // 장애물이 먼저 빠져나간다
    }

    // --- 충돌 예상: 통로 밖 정지선 또는 이미 들어선 상태 ---
    // 이미 어느 통로 안이면(kCommitted) 다른 장애물의 정지선보다 그쪽이 우선한다:
    // 정지선을 지키려고 지금 서면 서 있는 자리가 남의 차선이다 (시나리오 08 접촉이 그 모양).
    // 먼저 빠져나가고, 그 사이 다른 장애물은 VO/TTC 가 막는다.
    // 우선순위를 못 박아 장애물 순서에 결과가 달라지지 않게 한다.
    const bool committed = d_in <= 0.0 || c.inside(robot.x, robot.y);
    if (committed) {
      if (out.state != YieldState::kCommitted || d_out > out.zone_exit) {
        out.state = YieldState::kCommitted;      // 빠져나가는 데 가장 오래 걸리는 통로를 남긴다
        out.speed_limit = kInf;
        out.stop_distance = kInf;
        out.zone_entry = d_in;
        out.zone_exit = d_out;
        out.obstacle_in = t_in;
        out.obstacle_out = t_out;
        out.obstacle = static_cast<std::ptrdiff_t>(idx);
      }
      continue;
    }
    if (out.state == YieldState::kCommitted) {
      continue;                                  // 빠져나가는 중에는 정지선을 걸지 않는다
    }
    const double stop_distance = std::max(0.0, d_in - cfg.stop_margin);
    const double limit =
      SpeedProfile::maxSpeedForStop(stop_distance, cfg.decel, cfg.jerk, cfg.latency);
    if (limit < out.speed_limit) {
      out.state = YieldState::kYield;
      out.speed_limit = limit;
      out.stop_distance = stop_distance;
      out.zone_entry = d_in;
      out.zone_exit = d_out;
      out.obstacle_in = t_in;
      out.obstacle_out = t_out;
      out.obstacle = static_cast<std::ptrdiff_t>(idx);
    }
  }
  return out;
}

}  // namespace core
}  // namespace amr_navigation
