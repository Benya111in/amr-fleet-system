// A* 코어 벤치마크 (ROS 비의존) — 명세 4.4 "경로 계획 성능" 코어 측정.
//
//   ros2 run amr_navigation astar_benchmark <map.yaml> <pairs.csv> [out.csv]
//     [--inflation 1.2] [--scaling 2.0] [--inscribed 0.2] [--cost-weight 2.0] [--repeat 3]
//
// 지도(map_server 형식 P5 PGM)를 Nav2 InflationLayer 와 같은 식으로 팽창한 뒤 쌍마다
// core::AStar + core::PathSmoother 를 실행해 성공, 탐색/평활 시간, 확장 수, 경로 길이(격자/평활),
// 최소 여유거리(정확 EDT, 로봇 중심 기준)를 CSV 로 쓰고
//   요약(성공률, 평균/p95/최대 시간)을 출력한다.
// Nav2 planner_server 안에서의 NavFn/Smac 비교는 scripts/bench_planners.py (같은 지도·같은 쌍).
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "amr_navigation/core/astar.hpp"
#include "amr_navigation/core/geometry.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/path_smoother.hpp"

using amr_navigation::core::AStar;
using amr_navigation::core::AStarConfig;
using amr_navigation::core::AStarResult;
using amr_navigation::core::Cell;
using amr_navigation::core::OwnedGrid;
using amr_navigation::core::PathSmoother;
using amr_navigation::core::Point2D;
using amr_navigation::core::Pose2D;
using amr_navigation::core::SmootherStats;

namespace
{
struct MapMeta
{
  std::string image;
  double resolution{0.05};
  double ox{0.0};
  double oy{0.0};
  double occupied_thresh{0.65};
};

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r");
  const auto e = s.find_last_not_of(" \t\r");
  return b == std::string::npos ? "" : s.substr(b, e - b + 1);
}

bool readYaml(const std::string & path, MapMeta & m)
{
  std::ifstream f(path);
  if (!f) {
    return false;
  }
  std::string line;
  while (std::getline(f, line)) {
    const auto c = line.find(':');
    if (c == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, c));
    std::string val = trim(line.substr(c + 1));
    if (key == "image") {
      m.image = val;
    } else if (key == "resolution") {
      m.resolution = std::stod(val);
    } else if (key == "occupied_thresh") {
      m.occupied_thresh = std::stod(val);
    } else if (key == "origin") {
      std::replace(val.begin(), val.end(), '[', ' ');
      std::replace(val.begin(), val.end(), ']', ' ');
      std::replace(val.begin(), val.end(), ',', ' ');
      std::istringstream is(val);
      is >> m.ox >> m.oy;
    }
  }
  if (!m.image.empty() && m.image[0] != '/') {
    const auto slash = path.find_last_of('/');
    m.image = (slash == std::string::npos ? "" : path.substr(0, slash + 1)) + m.image;
  }
  return !m.image.empty();
}

bool readPgm(const MapMeta & m, OwnedGrid & g)
{
  std::ifstream f(m.image, std::ios::binary);
  if (!f) {
    return false;
  }
  std::string magic;
  int w = 0;
  int h = 0;
  int maxval = 0;
  f >> magic >> w >> h >> maxval;
  f.get();
  if (magic != "P5" || w <= 0 || h <= 0) {
    return false;
  }
  std::vector<unsigned char> img(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  f.read(reinterpret_cast<char *>(img.data()), static_cast<std::streamsize>(img.size()));
  g = OwnedGrid(w, h, m.resolution, 0, m.ox, m.oy);
  const double thr = (1.0 - m.occupied_thresh) * maxval;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      // PGM 첫 행 = y 최대
      const unsigned char p = img[static_cast<std::size_t>(h - 1 - y) * w + x];
      g.at(x, y) = p < thr ? amr_navigation::core::kLethalObstacle : 0;
    }
  }
  return true;
}

// Felzenszwalb–Huttenlocher 정확 제곱 거리 변환 (1D)
void edt1d(const std::vector<double> & f, std::vector<double> & d, int n)
{
  std::vector<int> v(n);
  std::vector<double> z(n + 1);
  int k = 0;
  v[0] = 0;
  z[0] = -1e20;
  z[1] = 1e20;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = 1e20;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

std::vector<double> distanceField(const OwnedGrid & g)
{
  const int w = g.width;
  const int h = g.height;
  std::vector<double> sq(static_cast<std::size_t>(w) * h);
  std::vector<double> f(std::max(w, h));
  std::vector<double> d(std::max(w, h));
  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      f[y] = g.at(x, y) == amr_navigation::core::kLethalObstacle ? 0.0 : 1e20;
    }
    edt1d(f, d, h);
    for (int y = 0; y < h; ++y) {
      sq[static_cast<std::size_t>(y) * w + x] = d[y];
    }
  }
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      f[x] = sq[static_cast<std::size_t>(y) * w + x];
    }
    edt1d(f, d, w);
    for (int x = 0; x < w; ++x) {
      sq[static_cast<std::size_t>(y) * w + x] = std::sqrt(d[x]) * g.resolution;
    }
  }
  return sq;
}

double minClearance(
  const OwnedGrid & g, const std::vector<double> & dist, const std::vector<Pose2D> & path)
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double len = std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    const int n = std::max(1, static_cast<int>(std::ceil(len / 0.025)));
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      int mx = 0;
      int my = 0;
      if (g.view().worldToMap(
          path[i].x + t * (path[i + 1].x - path[i].x), path[i].y + t * (path[i + 1].y - path[i].y),
          mx, my))
      {
        best = std::min(best, dist[static_cast<std::size_t>(my) * g.width + mx]);
      }
    }
  }
  return best;
}

double percentile(std::vector<double> v, double p)
{
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const double idx = p * static_cast<double>(v.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(idx));
  const auto hi = std::min(v.size() - 1, lo + 1);
  return v[lo] + (idx - lo) * (v[hi] - v[lo]);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(
      stderr, "usage: %s <map.yaml> <pairs.csv> [out.csv] [--inflation R] [--scaling s] "
      "[--inscribed r] [--cost-weight k] [--repeat n]\n", argv[0]);
    return 2;
  }
  std::string out_csv;
  double inflation = 1.2;
  double scaling = 2.0;
  double inscribed = 0.2;
  double cost_weight = 2.0;
  int repeat = 3;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](double & v) {
        if (i + 1 < argc) {
          v = std::stod(argv[++i]);
        }
      };
    if (a == "--inflation") {
      next(inflation);
    } else if (a == "--scaling") {
      next(scaling);
    } else if (a == "--inscribed") {
      next(inscribed);
    } else if (a == "--cost-weight") {
      next(cost_weight);
    } else if (a == "--repeat") {
      double r = repeat;
      next(r);
      repeat = std::max(1, static_cast<int>(r));
    } else {
      out_csv = a;
    }
  }
  MapMeta meta;
  OwnedGrid grid;
  if (!readYaml(argv[1], meta) || !readPgm(meta, grid)) {
    std::fprintf(stderr, "cannot read map %s\n", argv[1]);
    return 1;
  }
  const auto dist = distanceField(grid);
  amr_navigation::core::inflate(grid, inscribed, inflation, scaling);

  std::ifstream pf(argv[2]);
  std::string line;
  std::getline(pf, line);   // 헤더
  std::vector<std::array<double, 6>> pairs;
  while (std::getline(pf, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream is(line);
    std::array<double, 6> p{};
    if (is >> p[0] >> p[1] >> p[2] >> p[3] >> p[4] >> p[5]) {
      pairs.push_back(p);
    }
  }
  AStarConfig cfg;
  cfg.cost_weight = cost_weight;
  AStar astar(cfg);
  PathSmoother smoother;
  FILE * csv = out_csv.empty() ? nullptr : std::fopen(out_csv.c_str(), "w");
  if (csv != nullptr) {
    std::fprintf(
      csv, "idx,success,status,search_ms,smooth_ms,total_ms,expansions,raw_length,length,"
      "min_clearance\n");
  }
  int ok = 0;
  std::vector<double> t_search;
  std::vector<double> t_total;
  std::vector<double> lengths;
  std::vector<double> clearances;
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto & p = pairs[i];
    Cell s;
    Cell g;
    grid.view().worldToMap(p[0], p[1], s.x, s.y);
    grid.view().worldToMap(p[3], p[4], g.x, g.y);
    AStarResult r;
    std::vector<Pose2D> path;
    SmootherStats st;
    double best_search = 1e9;
    double best_total = 1e9;
    for (int k = 0; k < repeat; ++k) {   // 최솟값 = 외부 부하 잡음 제거 (부하 하 측정 보정)
      const auto t0 = std::chrono::steady_clock::now();
      r = astar.plan(grid.view(), s, g, 5);
      const auto t1 = std::chrono::steady_clock::now();
      if (r.ok()) {
        const Point2D sp{p[0], p[1]};
        const Point2D gp{p[3], p[4]};
        path = smoother.process(
          grid.view(), r.path, astar.traversalTable(), &sp, &gp, &p[5], &st);
      }
      const auto t2 = std::chrono::steady_clock::now();
      best_search = std::min(
        best_search, std::chrono::duration<double, std::milli>(t1 - t0).count());
      best_total = std::min(
        best_total, std::chrono::duration<double, std::milli>(t2 - t0).count());
    }
    const double clear = r.ok() ? minClearance(grid, dist, path) : 0.0;
    if (r.ok()) {
      ++ok;
      t_search.push_back(best_search);
      t_total.push_back(best_total);
      lengths.push_back(st.smoothed_length);
      clearances.push_back(clear);
    }
    if (csv != nullptr) {
      std::fprintf(
        csv, "%zu,%d,%s,%.3f,%.3f,%.3f,%zu,%.3f,%.3f,%.3f\n", i, r.ok() ? 1 : 0,
        amr_navigation::core::toString(r.status).c_str(), best_search, best_total - best_search,
        best_total, r.expansions, st.raw_length, st.smoothed_length, clear);
    }
  }
  if (csv != nullptr) {
    std::fclose(csv);
  }
  auto mean = [](const std::vector<double> & v) {
      double s = 0.0;
      for (double x : v) {
        s += x;
      }
      return v.empty() ? 0.0 : s / static_cast<double>(v.size());
    };
  std::printf(
    "pairs %zu, success %d (%.1f %%)\n"
    "search ms: mean %.2f p95 %.2f max %.2f\n"
    "total  ms: mean %.2f p95 %.2f max %.2f (A* + smoothing)\n"
    "length m : mean %.2f, min clearance m: mean %.3f min %.3f\n",
    pairs.size(), ok, pairs.empty() ? 0.0 : 100.0 * ok / static_cast<double>(pairs.size()),
    mean(t_search), percentile(t_search, 0.95),
    t_search.empty() ? 0.0 : *std::max_element(t_search.begin(), t_search.end()), mean(t_total),
    percentile(t_total, 0.95),
    t_total.empty() ? 0.0 : *std::max_element(t_total.begin(), t_total.end()), mean(lengths),
    mean(clearances),
    clearances.empty() ? 0.0 : *std::min_element(clearances.begin(), clearances.end()));
  return ok == static_cast<int>(pairs.size()) ? 0 : 3;
}
