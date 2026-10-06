// amr_perception SafetyGate::sweptRegion(...).approach() 를 파이썬에서 부르는 대조용 바인딩.
// rl/sim2d/amr_sim2d/geometry2d.py swept_approach 가 C++ 원본과 같은 값을 내는지
// 시험한다 (tests/test_collision.py).
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <limits>
#include <vector>

#include "amr_perception/safety_gate.hpp"

namespace py = pybind11;
namespace ap = amr_perception;

namespace
{
py::array_t<double> sweptApproachPy(
  const py::array_t<double, py::array::c_style | py::array::forcecast> & pts, double v, double w)
{
  ap::SafetyParams params;   // safety_gate.hpp 기본값 = robot_params.yaml safety.* (test 가 대조)
  ap::SafetyGate gate(params, std::vector<ap::SensorWatch>{}, 0.0);
  const ap::SweptFootprint r = gate.sweptRegion(ap::Motion{v, w});
  const auto p = pts.unchecked<2>();
  py::array_t<double> out(p.shape(0));
  auto o = out.mutable_unchecked<1>();
  for (py::ssize_t i = 0; i < p.shape(0); ++i) {
    o(i) = r.empty() ? std::numeric_limits<double>::infinity() :
      r.approach(ap::Vec2(p(i, 0), p(i, 1)));
  }
  return out;
}
}  // namespace

void registerSafety(py::module_ & m)
{
  m.def("swept_approach", &sweptApproachPy, py::arg("points"), py::arg("v"), py::arg("w"));
}
