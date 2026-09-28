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
  bool inside(double x, double y) const {return insideRadius(x, y, R);}
  /// 부풀린 반폭 R 은 "언제 양보를 시작할지" 를 정하는 값이다. "지금 실제로 길을 막고 있는지" 는
  /// 몸(반경 합)으로 봐야 한다 — 두 값의 차이(corridor_margin)만큼 통로가 흔들리면 판정이 뒤집힌다.
  bool insideRadius(double x, double y, double r) const
  {
    const double a = alpha(x, y);
    return std::abs(beta(x, y)) <= r && a >= -r && a <= L + r;
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

namespace
{
int severity(YieldDecision d)
{
  switch (d) {
    case YieldDecision::kRetreat: return 3;
    case YieldDecision::kHold: return 2;
    case YieldDecision::kGo: return 1;
    default: return 0;
  }
}
}  // namespace

YieldResult evaluateYield(
  const std::vector<Pose2D> & path, const std::vector<double> & cum_s, const Pose2D & robot,
  double s_robot, double v_robot, const std::vector<DynamicObstacle> & obstacles,
  const YieldConfig & cfg, YieldDecision prev_decision)
{
  YieldResult out;
  if (!cfg.enable || path.size() < 2 || cum_s.size() != path.size() || obstacles.empty()) {
    return out;
  }
  out.decided.assign(obstacles.size(), false);
  const double s_end = std::min(cum_s.back(), s_robot + cfg.lookahead);
  double hold_limit = kInf;      // kHold 인 장애물들의 속도 상한 최소
  double hold_entry = kInf;      // 그 입구까지 거리 최소

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
    // 몸 원통(위험 구간): 접촉이 실제로 나는 반폭. R_c 는 "언제 양보를 시작할지" 의 여유다.
    const double r_body = std::max(0.0, c.R - cfg.corridor_margin);
    const double r_danger = r_body + cfg.body_margin;

    // --- 경로 위 교차 구간 [s_in, s_out] (로봇 앞 첫 연속 구간) ---
    bool found = false;
    bool open = false;          // 훑기가 통로 안에서 끝났다 (구간이 창 밖으로 이어진다)
    double s_in = 0.0;
    double s_out = 0.0;
    double a_min = kInf;
    double a_max = -kInf;
    std::size_t last_i = 0;
    // 위험 구간의 경로 정점 (s_k, α_k) — kCommitted 결정(go/hold/retreat)의 시간 여유 계산용
    std::vector<std::pair<double, double>> body_pts;
    bool body_open = false;     // 창 끝까지 몸 원통 안이다
    std::size_t body_last_i = 0;
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
        if (c.insideRadius(path[i].x, path[i].y, r_danger)) {
          body_pts.emplace_back(cum_s[i], a);
          body_open = true;
          body_last_i = i;
        } else if (!body_pts.empty()) {
          body_open = false;
        }
      } else if (found) {
        open = false;
        body_open = false;
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
      if (body_open) {
        // 몸 원통도 창 끝까지 이어진다 — 같은 접선으로 그 출구를 외삽한다
        const double bb_last = c.beta(path[body_last_i].x, path[body_last_i].y);
        const double ds_b = ((db > 0.0 ? r_danger : -r_danger) - bb_last) / db;
        if (ds_b > 0.0) {
          body_pts.emplace_back(
            cum_s[body_last_i] + ds_b,
            c.alpha(path[body_last_i].x, path[body_last_i].y) + da * ds_b);
        }
      }
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
    // 이미 통로 안(kCommitted)일 때 해제는 **몸이 지나간 시점**으로 잰다. t_out 은 통로 반폭
    // R_c 로 재는데 거기에는 접근 판단용 여유(corridor_margin)가 들어 있어, 그걸로 해제까지
    // 재면 장애물이 지나간 뒤 corridor_margin/|u| 만큼 더 붙잡힌다. 08 실측 f8a 1번 시행:
    // 위협이 사라지고도(ttc inf, VO 기각 0) 0.70 s 동안 v ≈ 0.01 로 서 있었고, 그 값이
    // corridor_margin 0.7 m ÷ 작업자 1.0 m/s 와 정확히 같다. 그 0.70 s 가 복귀 5.02 s 를
    // 만들어 명세 4.7 상한 5.0 s 를 넘겼다. 접근에는 여유를 두고 해제는 몸 기준으로 재는
    // 비대칭이 맞다 — 여유는 "들어가도 되는가" 를 위한 것이지 "나가도 되는가" 를 위한 것이 아니다.
    // 기준은 **장애물의 몸이 로봇을 지났는가** 다. 교차 구간의 끝(a_max)으로 재면 안 된다 —
    // 구간 양 끝은 통로가 조금만 돌아도 크게 움직이기 때문이다 (08 실측 g8b 4번 시행: 구간
    // 입구가 0.19 s 만에 2.07 m → 0 으로 튀었다). 로봇 위치를 장애물 진행 방향에 투영한 값은
    // 실제 상대 위치라 진행각이 몇 도 흔들려도 몇 % 밖에 안 변한다.
    const bool committed = d_in <= 0.0 || c.inside(robot.x, robot.y);
    const double alpha_r = c.alpha(robot.x, robot.y);
    const double beta_r = c.beta(robot.x, robot.y);
    if (committed && alpha_r + r_body <= 0.0) {
      continue;   // 장애물이 몸으로 로봇을 지나갔다
    }
    if (committed) {
      // --- 진입 시점의 결정: go / hold / retreat (몸 원통 기준 시간 여유) ---
      // 통합 08 접촉 4건(j8c t6·t21, reg1 t12·t13)은 넷 다 "정지선 없이 통로 안에 들어선 채(v
      // 0.76~1.0) 점진 감속 + 선회" 였고, 진입 3.4~5.1 s 뒤 접촉했다. VO 포화는 1.2~2.3 s 전에야
      // 왔고 그때 v 는 0.11~0.54 — 차동 구동의 가로 권한은 v 에 비례하므로(후보 간 β 스프레드 ≈
      // 2·(v/ω)(1 − cos ωT): v 0.4 에서 0.87 m, v 0.1 에서 0.22 m) 그 뒤로는 어떤 비용항도 로봇을
      // 옆으로 못 옮긴다. 진입 순간에 옳은 행동은 따로 있었다:
      //   go      j8c t6  — 로봇 x −0.5·0.8 m/s, 작업자 x 5.41: 위험 구간 출구(x 1.32~1.6)까지
      //                     2.0~2.4 s vs 작업자 앞 3.1~3.4 s. 감속 없이 직진하면 스칠 때 몸 사이
      //                     0.35~0.5 m.
      //   hold    j8c t21 — β 1.05·1.0 m/s, 작업자 6 m: 0.5 m 안에 서면 β ≥ 0.83. reg1 t13 — β 1.3.
      //   retreat reg1 t12 — β 0.95, 여유 0: 축에서 멀어지는 쪽(북, 자유 공간)으로만.
      // R_c 통로(반폭 1.36)로 재면 "지나가도 되는" 경우까지 충돌로 보고, 그 감속이 접촉을 만든다.
      YieldDecision dec = YieldDecision::kNone;
      double margin = -kInf;
      double clearance = -kInf;
      double body_entry = kInf;
      double limit = kInf;
      const bool robot_in_body = c.insideRadius(robot.x, robot.y, r_danger);
      // 교차각: 로봇 자리의 경로 접선과 통로 축의 sin. 얕은 교차(정면·같은 차선)에서는 몸 원통이
      // 경로를 따라 길게 이어져 이 결정이 뜻이 없다 — 예전 그대로 VO/TTC·원통 안 탈출이 맡는다
      // (단위 시험 ClosedLoopHeadOnYields 0°, VoSaturatedEscapeStaysInsideTheDeviationBudget 15°).
      double cross_sin = 1.0;
      double along = 0.0;    // 경로 접선 · û (+ 같은 방향, − 마주 옴)
      {
        const Projection pr = projectOntoPath(path, {robot.x, robot.y}, 0, 0, &cum_s);
        const double tx = std::cos(pr.heading), ty = std::sin(pr.heading);
        cross_sin = std::abs(tx * c.nx + ty * c.ny);
        along = tx * c.ux + ty * c.uy;
      }
      if (cfg.decision_enable && alpha_r >= 0.0 && !body_pts.empty() &&
        cross_sin >= cfg.decision_min_sin)
      {
        // 로봇이 먼저: 위험 구간의 모든 정점에서 작업자 앞(중심 − r_body)이 로봇보다 뒤여야 한다.
        // 작업자가 먼저: 로봇이 구간에 닿기 전에 작업자 몸(중심 + r_body)이 지나가야 한다.
        // "로봇이 먼저" 는 작업자 속도를 go_speed_floor 아래로 보지 않는다 — 반환점 직후의 평활
        // 속도(0.3 → 1.0 으로 오르는 중)로 재면 여유가 실제보다 최대 2 배 크다.
        const double su_go = std::max(su, cfg.go_speed_floor);
        double m_robot_first = kInf;
        double m_walker_first = kInf;
        for (const auto & [s_k, a_k] : body_pts) {
          const double t_k =
            travelTime(std::max(0.0, s_k - s_robot), v_robot, cfg.accel, cfg.v_max);
          m_robot_first = std::min(m_robot_first, (a_k - r_body) / su_go - t_k);
          m_walker_first = std::min(m_walker_first, t_k - (a_k + r_body) / su);
        }
        if (robot_in_body) {
          m_walker_first = -kInf;   // 이미 안에 있다 — 작업자가 먼저 지나갈 수는 없다
        }
        margin = std::max(m_robot_first, m_walker_first);
        body_entry = robot_in_body ? 0.0 : std::max(0.0, body_pts.front().first - s_robot);
        // 스칠 때 몸 사이 예측 여유. 로봇이 먼저: 출구(|β| = r_danger)를 나선 뒤 로봇은 경로를 따라
        // v_max 로 축에서 멀어지고(가로 성분 v·sin θ), 작업자 중심은 (여유·|u| + r_body)/(닫힘
        // 속도) 뒤에 로봇의 α 에 온다 — 마주 오면 닫힘 = |u| + v·|cos θ|, 같은 방향이면
        // |u| − v·cos θ (0 이하면 못 따라잡는다). 작업자가 먼저: 몸이 지난 뒤 여유·|u| 만큼 더
        // 멀어져 있다.
        // 같은 시간 여유라도 교차각이 가파를수록 여유가 크다 — 26.6° 에서 0.6 s ≈ 0.45 m, 90°
        // 에서 0.2 s ≈ 1.0 m. 시간 여유 하한(go_margin)은 "앞서긴 해야 한다" 만 본다.
        if (m_robot_first >= cfg.go_margin) {
          const double closing = su_go - cfg.v_max * along;
          clearance = closing <= 1e-3 ? kInf :
            cfg.body_margin + cfg.v_max * cross_sin * (m_robot_first * su_go + r_body) / closing;
        }
        if (m_walker_first >= cfg.go_margin) {
          clearance = std::max(clearance, cfg.body_margin + m_walker_first * su);
        }
        // go 는 이력을 둔다: 들어갈 때 go_clearance, 유지는 go_hold_clearance (추적기 속도 추정이
        // 한 주기 튀어도 go ↔ retreat 로 뒤집히지 않게).
        const double need =
          prev_decision == YieldDecision::kGo ? cfg.go_hold_clearance : cfg.go_clearance;
        if (clearance >= need) {
          dec = YieldDecision::kGo;
        } else if (robot_in_body) {
          dec = YieldDecision::kRetreat;
        } else if (prev_decision == YieldDecision::kHold) {
          // 이미 제동 중이면 입구 밖에 있는 한 계속 선다 — 실속도가 명령을 늦게 따라와 d_stop 이
          // 남은 거리보다 커 보여도(재현: 1.15 s 뒤 retreat 로 뒤집혀 축을 가로질렀다) 브레이크를
          // 놓지 않는다. 상한이 입구에서 0 이 되므로 못 서도 최대 감속으로 선다.
          dec = YieldDecision::kHold;
          limit = SpeedProfile::maxSpeedForStop(body_entry, cfg.decel, cfg.jerk, cfg.latency);
        } else {
          // 몸 원통 밖: 입구 앞에 설 수 있으면 선다. 제동 중 바깥으로 조향하면 안쪽 표류가 줄므로
          // hold_slack 만큼 낙관한다. 못 서면 브레이크로 원통 안에 들어가는 것이 최악이므로
          // retreat.
          const double d_stop =
            SpeedProfile::stoppingDistance(v_robot, cfg.decel, cfg.jerk, cfg.latency);
          if (d_stop <= body_entry + cfg.hold_slack) {
            dec = YieldDecision::kHold;
            limit = SpeedProfile::maxSpeedForStop(body_entry, cfg.decel, cfg.jerk, cfg.latency);
          } else {
            dec = YieldDecision::kRetreat;
          }
        }
      }
      out.decided[idx] = dec != YieldDecision::kNone;
      // 기록: 축이 가장 가까운(|β| 최소) 통로를 남긴다 — 근거리에서 사람 하나가 두 트랙으로
      // 갈라지면
      // (08 접촉 4건 모두 0.4~1.0 s 전, 0.35~0.39 m 비낌) 예전 규칙("출구가 가장 먼 통로")은
      // 체계적으로 비낀 쪽을 골라 β 를 과대평가했다. 결정은 가장 보수적인 것(retreat > hold > go).
      const bool first = out.state != YieldState::kCommitted;
      if (first) {
        out.state = YieldState::kCommitted;
        out.speed_limit = kInf;
        out.stop_distance = kInf;
      }
      if (first || std::abs(beta_r) < std::abs(out.body_beta)) {
        out.zone_entry = d_in;
        out.zone_exit = d_out;
        out.obstacle_in = t_in;
        out.obstacle_out = t_out;
        out.obstacle = static_cast<std::ptrdiff_t>(idx);
        out.body_beta = beta_r;
      }
      if (first || severity(dec) > severity(out.decision)) {
        out.decision = dec;
        out.margin = margin;
        out.clearance = clearance;
        out.body_entry = body_entry;
      } else if (dec == out.decision) {
        out.margin = std::min(out.margin, margin);
        out.clearance = std::min(out.clearance, clearance);
        out.body_entry = std::min(out.body_entry, body_entry);
      }
      if (dec == YieldDecision::kHold) {
        hold_limit = std::min(hold_limit, limit);
        hold_entry = std::min(hold_entry, body_entry);
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
  if (out.state == YieldState::kCommitted && out.decision == YieldDecision::kHold) {
    // hold: 몸 원통 입구까지의 속도 상한 (kYield 의 정지선과 같은 자리, 같은 역함수)
    out.speed_limit = hold_limit;
    out.stop_distance = hold_entry;
  }
  return out;
}

}  // namespace core
}  // namespace amr_navigation
