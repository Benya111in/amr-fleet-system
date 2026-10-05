// amr_navigation ROS 비의존 코어(src/amr_navigation/src/core/*.cpp)를 파이썬에서 부르는 pybind11 모듈.
//
// 원본 소스는 읽기 전용으로 그대로 컴파일한다 (build.sh). 여기에는 변환 코드만 있다.
//   - inflate:  grid.cpp inflate() — 전역 코스트맵 정적 팽창과 같은 함수
//   - plan:     astar.cpp AStar::plan + path_smoother.cpp PathSmoother::process
//               (src/amr_navigation/src/ros/astar_planner.cpp createPlan 과 같은 순서)
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cmath>
#include <cstring>
#include <string>
#include <tuple>
#include <vector>

#include "amr_navigation/core/astar.hpp"
#include "amr_navigation/core/grid.hpp"
#include "amr_navigation/core/path_smoother.hpp"

namespace py = pybind11;
namespace core = amr_navigation::core;

namespace
{
using U8Array = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>;

core::OwnedGrid toGrid(const U8Array & cells, double res, double ox, double oy)
{
  if (cells.ndim() != 2) {
    throw std::runtime_error("cells 는 [H, W] uint8 이어야 한다");
  }
  const int h = static_cast<int>(cells.shape(0));
  const int w = static_cast<int>(cells.shape(1));
  core::OwnedGrid g(w, h, res, core::kFreeSpace, ox, oy);
  std::memcpy(g.cells.data(), cells.data(), static_cast<std::size_t>(w) * h);
  return g;
}

U8Array inflatePy(
  const U8Array & cells, double res, double inscribed, double radius, double scaling)
{
  core::OwnedGrid g = toGrid(cells, res, 0.0, 0.0);
  core::inflate(g, inscribed, radius, scaling);
  U8Array out({g.height, g.width});
  std::memcpy(out.mutable_data(), g.cells.data(), g.cells.size());
  return out;
}

// 반환: (상태 문자열, 자세 [N,3] (x, y, yaw), 격자 경로 비용, 확장 수)
std::tuple<std::string, py::array_t<double>, double, std::size_t> planPy(
  const U8Array & cells, double res, double ox, double oy,
  double sx, double sy, double gx, double gy, double goal_yaw,
  double cost_weight, double tolerance, bool allow_unknown, int unknown_cost,
  bool smoother)
{
  core::OwnedGrid g = toGrid(cells, res, ox, oy);
  const core::CostGrid grid = g.view();
  core::AStarConfig ac;
  ac.cost_weight = cost_weight;
  ac.allow_unknown = allow_unknown;
  ac.unknown_cost = static_cast<uint8_t>(unknown_cost);
  ac.allow_corner_cutting = false;
  ac.allow_start_in_inscribed = true;
  core::AStar astar(ac);
  core::Cell s;
  core::Cell e;
  py::array_t<double> empty(std::vector<py::ssize_t>{0, 3});
  if (!grid.worldToMap(sx, sy, s.x, s.y)) {
    return {"start_out_of_bounds", empty, 0.0, 0};
  }
  if (!grid.worldToMap(gx, gy, e.x, e.y)) {
    return {"goal_out_of_bounds", empty, 0.0, 0};
  }
  const int tol_cells = static_cast<int>(std::ceil(tolerance / grid.resolution));
  core::AStarResult r;
  {
    py::gil_scoped_release release;
    r = astar.plan(grid, s, e, tol_cells);
  }
  if (!r.ok()) {
    return {core::toString(r.status), empty, 0.0, r.expansions};
  }
  core::Point2D sp{sx, sy};
  core::Point2D end{gx, gy};
  if (!(r.reached_goal == e)) {
    grid.mapToWorld(r.reached_goal.x, r.reached_goal.y, end.x, end.y);
  }
  core::SmootherConfig sc;   // nav2_params.yaml AStar.smoother 값과 기본값이 같다 (test_binding 이 확인)
  sc.enable_shortcut = smoother;
  sc.enable_smoothing = smoother;
  sc.allow_unknown = allow_unknown;
  core::PathSmoother sm(sc);
  std::vector<core::Pose2D> poses;
  {
    py::gil_scoped_release release;
    poses = sm.process(grid, r.path, astar.traversalTable(), &sp, &end, &goal_yaw, nullptr);
  }
  py::array_t<double> out({static_cast<py::ssize_t>(poses.size()), py::ssize_t{3}});
  auto m = out.mutable_unchecked<2>();
  for (std::size_t i = 0; i < poses.size(); ++i) {
    m(i, 0) = poses[i].x;
    m(i, 1) = poses[i].y;
    m(i, 2) = poses[i].theta;
  }
  return {"success", out, r.cost, r.expansions};
}

py::dict smootherDefaults()
{
  core::SmootherConfig sc;
  py::dict d;
  d["enable_shortcut"] = sc.enable_shortcut;
  d["shortcut_max_length"] = sc.shortcut_max_length;
  d["shortcut_cost_ratio"] = sc.shortcut_cost_ratio;
  d["output_spacing"] = sc.output_spacing;
  d["enable_smoothing"] = sc.enable_smoothing;
  d["w_data"] = sc.w_data;
  d["w_smooth"] = sc.w_smooth;
  d["w_clearance"] = sc.w_clearance;
  d["clearance_cost_threshold"] = static_cast<int>(sc.clearance_cost_threshold);
  d["clearance_cost_margin"] = sc.clearance_cost_margin;
  d["max_iterations"] = sc.max_iterations;
  d["tolerance"] = sc.tolerance;
  return d;
}
}  // namespace

PYBIND11_MODULE(amr_core_py, m)
{
  m.doc() = "amr_navigation core (A*, inflate, path smoother) 바인딩 — rl/sim2d 용";
  m.def("inflate", &inflatePy, py::arg("cells"), py::arg("resolution"),
    py::arg("inscribed_radius"), py::arg("inflation_radius"), py::arg("cost_scaling_factor"));
  m.def("plan", &planPy, py::arg("cells"), py::arg("resolution"), py::arg("origin_x"),
    py::arg("origin_y"), py::arg("sx"), py::arg("sy"), py::arg("gx"), py::arg("gy"),
    py::arg("goal_yaw") = 0.0, py::arg("cost_weight") = 2.0, py::arg("tolerance") = 0.25,
    py::arg("allow_unknown") = true, py::arg("unknown_cost") = 252, py::arg("smoother") = true);
  m.def("smoother_defaults", &smootherDefaults);
  m.attr("LETHAL") = static_cast<int>(core::kLethalObstacle);
  m.attr("UNKNOWN") = static_cast<int>(core::kNoInformation);
  m.attr("INSCRIBED") = static_cast<int>(core::kInscribedInflated);
}
