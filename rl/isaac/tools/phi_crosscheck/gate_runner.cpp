// Copyright 2026 enhyunss. Apache-2.0 (package.xml 참고).
//
// 정본 safety φ 교차 시험 실행기 — C++ 원본 amr_perception::SafetyGate 를 ROS 없이 직접 돌린다.
// crosscheck.py 가 이진 입력을 쓰고, 이 실행기가 원본으로 계산한 값을 이진 출력으로 돌려준다.
//
//   gate_runner fp  <in> <out>               접근 거리·꼭짓점 속력·여유 상한·존 (점별)
//   gate_runner win <in> <out>               physicalWindowStatistic·kthSmallest (배열별)
//   gate_runner seq <in> <out> [이름=값 ...] SafetyGate 순차 evaluate (시나리오 × 틱)
//
// 이진 형식은 crosscheck.py 머리말에 있다 (모두 little-endian int64 / float64).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "amr_perception/safety_gate.hpp"

namespace per = amr_perception;

namespace
{

class Reader
{
public:
  explicit Reader(const char * path)
  : f_(std::fopen(path, "rb"))
  {
    if (f_ == nullptr) {
      std::fprintf(stderr, "열 수 없다: %s\n", path);
      std::exit(2);
    }
  }
  ~Reader() {std::fclose(f_);}
  int64_t i64()
  {
    int64_t v = 0;
    must(std::fread(&v, sizeof(v), 1, f_) == 1);
    return v;
  }
  double f64()
  {
    double v = 0.0;
    must(std::fread(&v, sizeof(v), 1, f_) == 1);
    return v;
  }
  void f64s(double * dst, std::size_t n)
  {
    must(n == 0 || std::fread(dst, sizeof(double), n, f_) == n);
  }

private:
  static void must(bool ok)
  {
    if (!ok) {
      std::fprintf(stderr, "입력이 짧다\n");
      std::exit(2);
    }
  }
  std::FILE * f_;
};

class Writer
{
public:
  explicit Writer(const char * path)
  : f_(std::fopen(path, "wb"))
  {
    if (f_ == nullptr) {
      std::fprintf(stderr, "쓸 수 없다: %s\n", path);
      std::exit(2);
    }
  }
  ~Writer() {std::fclose(f_);}
  void f64(double v) {std::fwrite(&v, sizeof(v), 1, f_);}

private:
  std::FILE * f_;
};

// fp: int64 M, K → M × (v, w, d) → M × K × (x, y)
// 출력: M × (maxPointSpeed(0.3, 0.2), clearance(d, 0.30), clearance(d, 0.10), zone(d)),
//       그 뒤 M × K 접근 거리
int runFootprint(const char * in, const char * out)
{
  Reader r(in);
  Writer w(out);
  const int64_t M = r.i64();
  const int64_t K = r.i64();
  std::vector<double> mot(static_cast<std::size_t>(M * 3));
  r.f64s(mot.data(), mot.size());
  std::vector<double> pts(static_cast<std::size_t>(M * K * 2));
  r.f64s(pts.data(), pts.size());
  const per::SafetyParams sp;
  const per::SafetyGate gate(sp, {}, 0.0);
  for (int64_t m = 0; m < M; ++m) {
    const double v = mot[m * 3], om = mot[m * 3 + 1], d = mot[m * 3 + 2];
    w.f64(per::SweptFootprint::maxPointSpeed(0.3, 0.2, per::Motion{v, om}));
    w.f64(per::SafetyGate::clearanceSpeedLimit(d, 0.30, sp));
    w.f64(per::SafetyGate::clearanceSpeedLimit(d, 0.10, sp));
    w.f64(static_cast<double>(per::SafetyGate::classifyZone(d, sp)));
  }
  for (int64_t m = 0; m < M; ++m) {
    const per::SweptFootprint reg = gate.sweptRegion(per::Motion{mot[m * 3], mot[m * 3 + 1]});
    for (int64_t k = 0; k < K; ++k) {
      const std::size_t i = static_cast<std::size_t>((m * K + k) * 2);
      w.f64(reg.approach(per::Vec2(pts[i], pts[i + 1])));
    }
  }
  return 0;
}

// win: int64 A → A × [int64 n, f64 inc, f64 width, int64 min_window, f64 support, int64 circular,
//        n 값, n 거리] → int64 Q → Q × [int64 n, int64 support, n 값]
// 출력: A 개 창 통계, Q 개 k 번째 값
int runWindow(const char * in, const char * out)
{
  Reader r(in);
  Writer w(out);
  const int64_t A = r.i64();
  for (int64_t a = 0; a < A; ++a) {
    const int64_t n = r.i64();
    const double inc = r.f64();
    const double width = r.f64();
    const int64_t min_window = r.i64();
    const double support = r.f64();
    const bool circular = r.i64() != 0;
    std::vector<double> vals(static_cast<std::size_t>(n)), rng(static_cast<std::size_t>(n));
    r.f64s(vals.data(), vals.size());
    r.f64s(rng.data(), rng.size());
    w.f64(
      per::physicalWindowStatistic(
        vals, rng, inc, width, static_cast<int>(min_window), support, circular));
  }
  const int64_t Q = r.i64();
  for (int64_t q = 0; q < Q; ++q) {
    const int64_t n = r.i64();
    const int64_t support = r.i64();
    std::vector<double> vals(static_cast<std::size_t>(n));
    r.f64s(vals.data(), vals.size());
    std::vector<double> finite;
    for (const double x : vals) {
      if (std::isfinite(x)) {
        finite.push_back(x);
      }
    }
    w.f64(per::kthSmallest(finite, static_cast<int>(support)));
  }
  return 0;
}

// 매개변수 이름 → 필드 (seq 의 이름=값 덮어쓰기)
bool setParam(per::SafetyParams & p, const std::string & key, double val)
{
  std::map<std::string, double *> d{
    {"footprint_length", &p.footprint_length}, {"footprint_width", &p.footprint_width},
    {"emergency_stop_distance", &p.emergency_stop_distance},
    {"critical_zone_distance", &p.critical_zone_distance},
    {"warning_zone_distance", &p.warning_zone_distance},
    {"warning_zone_max_speed", &p.warning_zone_max_speed},
    {"critical_zone_max_speed", &p.critical_zone_max_speed},
    {"stop_release_distance", &p.stop_release_distance},
    {"zone_hysteresis", &p.zone_hysteresis}, {"reaction_latency", &p.reaction_latency},
    {"max_deceleration", &p.max_deceleration}, {"max_linear_velocity", &p.max_linear_velocity},
    {"min_linear_velocity", &p.min_linear_velocity},
    {"max_angular_velocity", &p.max_angular_velocity}, {"ttc_critical", &p.ttc_critical},
    {"ttc_max_age", &p.ttc_max_age}, {"degraded_mode_max_speed", &p.degraded_mode_max_speed},
    {"command_timeout", &p.command_timeout}, {"swept_margin", &p.swept_margin},
    {"region_margin", &p.region_margin}, {"motion_epsilon_v", &p.motion_epsilon_v},
    {"motion_epsilon_w", &p.motion_epsilon_w},
    {"measured_velocity_timeout", &p.measured_velocity_timeout},
    {"measured_velocity_time_constant", &p.measured_velocity_time_constant},
    {"self_filter_margin", &p.self_filter_margin}, {"beam_window_width", &p.beam_window_width},
    {"beam_support", &p.beam_support}, {"guard_window_width", &p.guard_window_width},
    {"contact_guard_distance", &p.contact_guard_distance},
    {"immediate_stop_margin", &p.immediate_stop_margin}, {"cloud_timeout", &p.cloud_timeout},
    {"cloud_max_age", &p.cloud_max_age}, {"max_compensation_age", &p.max_compensation_age},
    {"escape_horizon", &p.escape_horizon}, {"escape_max_speed", &p.escape_max_speed},
    {"retreat_max_speed", &p.retreat_max_speed}, {"yield_escape_speed", &p.yield_escape_speed},
    {"dynamic_obstacle_radius", &p.dynamic_obstacle_radius},
    {"yield_escape_overlap_margin", &p.yield_escape_overlap_margin},
    {"yield_escape_min_gain", &p.yield_escape_min_gain},
    {"yield_escape_min_obstacle_speed", &p.yield_escape_min_obstacle_speed},
    {"exclusion_stop_distance", &p.exclusion_stop_distance},
    {"exclusion_timeout", &p.exclusion_timeout}};
  std::map<std::string, bool *> b{
    {"clearance_speed_limit_enabled", &p.clearance_speed_limit_enabled},
    {"ttc_limit_enabled", &p.ttc_limit_enabled},
    {"estop_release_requires_reset", &p.estop_release_requires_reset},
    {"hold_motion_intent", &p.hold_motion_intent}, {"cloud_required", &p.cloud_required},
    {"latency_compensation", &p.latency_compensation}, {"allow_escape", &p.allow_escape},
    {"ttc_only_dynamic", &p.ttc_only_dynamic},
    {"yield_escape_enabled", &p.yield_escape_enabled}};
  std::map<std::string, int *> i{
    {"beam_window", &p.beam_window}, {"guard_window", &p.guard_window},
    {"stop_confirm_frames", &p.stop_confirm_frames}, {"cloud_min_points", &p.cloud_min_points}};
  if (d.count(key)) {
    *d[key] = val;
  } else if (b.count(key)) {
    *b[key] = val != 0.0;
  } else if (i.count(key)) {
    *i[key] = static_cast<int>(val);
  } else {
    return false;
  }
  return true;
}

bool hasReason(const per::SafetyStatus & st, const char * r)
{
  for (const auto & s : st.reasons) {
    if (s == r) {
      return true;
    }
  }
  return false;
}

double causeBits(const per::SafetyStatus & st)
{
  int bits = 0;
  for (const auto & c : st.stop_causes) {
    if (c == "lidar") {
      bits |= 1;
    } else if (c == "depth") {
      bits |= 2;
    } else if (c == "contact") {
      bits |= 4;
    } else if (c == "dock_exclusion") {
      bits |= 8;
    } else {
      bits |= 16;
    }
  }
  return static_cast<double>(bits);
}

// seq 입력:
//   int64 S, T, B, circular, NS(시나리오당 스캔 수), Cmax, Tmax, Pmax; f64 inc, origin_x, origin_y
//   시나리오마다: NS × B × 2 스캔 빔, 그 뒤 T × 틱 기록 (아래 순서)
// 틱 기록 (f64): now | meas_flag v w stamp | cmd_flag v w stamp | scan_flag scan_index stamp |
//   cloud_flag capture received n [Cmax × 2] |
//   trk_flag(0 없음/1 setTracks/2 setMinTtc) capture received n min_ttc
//   [Tmax × 6: ttc dyn px py vx vy] | excl_flag stamp n [Pmax × 2]
// 출력 (틱마다 f64 × 20): v w zone prox causes escaping retreat yield ttc_limit cloud_stale
//   command_stale
//   speed_limit angular_limit min_distance lidar cloud contact exclusion min_ttc estop_active
int runSequence(const char * in, const char * out, int argc, char ** argv)
{
  per::SafetyParams p;
  for (int a = 0; a < argc; ++a) {
    const std::string kv = argv[a];
    const auto eq = kv.find('=');
    if (eq == std::string::npos || !setParam(p, kv.substr(0, eq), std::atof(kv.c_str() + eq + 1))) {
      std::fprintf(stderr, "모르는 매개변수: %s\n", kv.c_str());
      return 2;
    }
  }
  Reader r(in);
  Writer w(out);
  const int64_t S = r.i64(), T = r.i64(), B = r.i64();
  const bool circular = r.i64() != 0;
  const int64_t NS = r.i64(), Cmax = r.i64(), Tmax = r.i64(), Pmax = r.i64();
  const double inc = r.f64();
  const double ox = r.f64(), oy = r.f64();
  const per::Vec2 origin(ox, oy);
  const std::size_t rec = static_cast<std::size_t>(
    1 + 4 + 4 + 3 + 4 + 2 * Cmax + 5 + 6 * Tmax + 3 + 2 * Pmax);
  std::vector<double> scans(static_cast<std::size_t>(NS * B * 2));
  std::vector<double> x(rec);
  for (int64_t s = 0; s < S; ++s) {
    r.f64s(scans.data(), scans.size());
    per::SafetyGate gate(p, {}, 0.0);
    for (int64_t t = 0; t < T; ++t) {
      r.f64s(x.data(), rec);
      std::size_t k = 0;
      const double now = x[k++];
      const double mf = x[k++], mv = x[k++], mw = x[k++], ms = x[k++];
      const double cf = x[k++], cv = x[k++], cw = x[k++], cs = x[k++];
      const double sf = x[k++], si = x[k++], ss = x[k++];
      const double clf = x[k++], clc = x[k++], clr = x[k++], cln = x[k++];
      const std::size_t cl0 = k;
      k += static_cast<std::size_t>(2 * Cmax);
      const double tf = x[k++], tc = x[k++], tr = x[k++], tn = x[k++], tm = x[k++];
      const std::size_t t0 = k;
      k += static_cast<std::size_t>(6 * Tmax);
      const double ef = x[k++], es = x[k++], en = x[k++];
      const std::size_t e0 = k;
      if (mf != 0.0) {
        gate.setMeasuredVelocity(mv, mw, ms);
      }
      if (sf != 0.0) {
        const std::size_t base = static_cast<std::size_t>(si) * static_cast<std::size_t>(B * 2);
        std::vector<per::Vec2> beams;
        beams.reserve(static_cast<std::size_t>(B));
        for (int64_t b = 0; b < B; ++b) {
          beams.emplace_back(scans[base + 2 * b], scans[base + 2 * b + 1]);
        }
        gate.setScan(beams, origin, inc, circular, ss);
      }
      if (clf != 0.0) {
        std::vector<per::Vec2> pts;
        for (int64_t c = 0; c < static_cast<int64_t>(cln); ++c) {
          pts.emplace_back(x[cl0 + 2 * c], x[cl0 + 2 * c + 1]);
        }
        gate.setCloud(pts, clc, clr);
      }
      if (tf == 1.0) {
        std::vector<per::TrackTtc> tracks;
        for (int64_t j = 0; j < static_cast<int64_t>(tn); ++j) {
          const double * q = &x[t0 + 6 * j];
          per::TrackTtc tt;
          tt.ttc = q[0];
          tt.is_dynamic = q[1] != 0.0;
          tt.position = per::Vec2(q[2], q[3]);
          tt.velocity = per::Vec2(q[4], q[5]);
          tracks.push_back(tt);
        }
        gate.setTracks(tracks, tc, tr);
      } else if (tf == 2.0) {
        gate.setMinTtc(tm, tr);
      }
      if (ef != 0.0) {
        std::vector<per::Vec2> poly;
        for (int64_t j = 0; j < static_cast<int64_t>(en); ++j) {
          poly.emplace_back(x[e0 + 2 * j], x[e0 + 2 * j + 1]);
        }
        gate.setExclusionPolygon(poly, es);
      }
      if (cf != 0.0) {
        gate.setCommand(cv, cw, cs);
      }
      const per::SafetyStatus st = gate.evaluate(now);
      const double o[20] = {
        st.command.linear, st.command.angular, static_cast<double>(st.zone),
        st.proximity_stop ? 1.0 : 0.0, causeBits(st), st.escaping ? 1.0 : 0.0,
        hasReason(st, "ttc_retreat") ? 1.0 : 0.0, st.yield_escaping ? 1.0 : 0.0,
        hasReason(st, "ttc_limit") ? 1.0 : 0.0, st.cloud_stale ? 1.0 : 0.0,
        st.command_stale ? 1.0 : 0.0, st.speed_limit, st.angular_limit, st.min_distance,
        st.lidar_distance, st.cloud_distance, st.contact_distance, st.exclusion_distance,
        st.min_ttc, st.estop_active ? 1.0 : 0.0};
      for (const double v : o) {
        w.f64(v);
      }
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::fprintf(stderr, "사용: gate_runner fp|win|seq <in> <out> [이름=값 ...]\n");
    return 2;
  }
  const std::string mode = argv[1];
  if (mode == "fp") {
    return runFootprint(argv[2], argv[3]);
  }
  if (mode == "win") {
    return runWindow(argv[2], argv[3]);
  }
  if (mode == "seq") {
    return runSequence(argv[2], argv[3], argc - 4, argv + 4);
  }
  std::fprintf(stderr, "모르는 모드: %s\n", mode.c_str());
  return 2;
}
