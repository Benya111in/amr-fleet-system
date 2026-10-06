// golden JSON → ContractState (시험 공용, nlohmann::json).
#ifndef GOLDEN_IO_HPP_
#define GOLDEN_IO_HPP_

#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "amr_rl_controller/core/state.hpp"

namespace golden
{

inline double num(const nlohmann::json & v)
{
  return v.is_null() ? std::numeric_limits<double>::infinity() : v.get<double>();
}

inline nlohmann::json load(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("열 수 없다: " + path);
  }
  nlohmann::json j;
  f >> j;
  return j;
}

inline amr_rl_controller::ContractState toState(const nlohmann::json & d)
{
  amr_rl_controller::ContractState s;
  s.pose = {d["pose"][0].get<double>(), d["pose"][1].get<double>(), d["pose"][2].get<double>()};
  for (const auto & p : d["path"]) {
    s.path.push_back({p[0].get<double>(), p[1].get<double>()});
  }
  for (const auto & r : d["scan_ranges"]) {
    s.scan_ranges.push_back(
      r.is_null() ? std::numeric_limits<double>::quiet_NaN() :
      r.get<double>());
  }
  s.v_meas = d.value("v_meas", 0.0);
  s.w_meas = d.value("w_meas", 0.0);
  s.a_v_meas = d.value("a_v_meas", 0.0);
  s.a_w_meas = d.value("a_w_meas", 0.0);
  s.v_cmd = d.value("v_cmd", 0.0);
  s.w_cmd = d.value("w_cmd", 0.0);
  if (d.contains("prev_actions")) {
    const auto & pa = d["prev_actions"];
    for (std::size_t k = 0; k < std::min<std::size_t>(pa.size(), s.prev_actions.size()); ++k) {
      s.prev_actions[k] = {pa[k][0].get<double>(), pa[k][1].get<double>()};
    }
  }
  s.v_cap = d.value("v_cap", 1.0);
  s.scan_age = d.value("scan_age", 0.0);
  s.track_age = d.value("track_age", 0.0);
  if (d.contains("tracks")) {
    for (const auto & t : d["tracks"]) {
      amr_rl_controller::Track tr;
      tr.id = t["id"].get<int64_t>();
      tr.x = t["x"].get<double>();
      tr.y = t["y"].get<double>();
      tr.vx = t["vx"].get<double>();
      tr.vy = t["vy"].get<double>();
      tr.is_dynamic = t["is_dynamic"].get<bool>();
      s.tracks.push_back(tr);
    }
  }
  if (d.contains("safety")) {
    const auto & sf = d["safety"];
    s.safety.zone = sf.value("zone", 0);
    s.safety.speed_limit = sf.contains("speed_limit") ? num(sf["speed_limit"]) :
      std::numeric_limits<double>::infinity();
    s.safety.proximity_stop = sf.value("proximity_stop", false);
    s.safety.min_ttc = sf.contains("min_ttc") ? num(sf["min_ttc"]) :
      std::numeric_limits<double>::infinity();
  }
  if (d.contains("dev")) {
    const auto & dv = d["dev"];
    s.dev.in_dev = dv.value("in_dev", false);
    s.dev.peak = dv.value("peak", 0.0);
    s.dev.t_since_peak = dv.value("t_since_peak", 0.0);
  }
  return s;
}

}  // namespace golden

#endif  // GOLDEN_IO_HPP_
