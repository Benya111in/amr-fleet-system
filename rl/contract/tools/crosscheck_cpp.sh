#!/usr/bin/env bash
# C++ 원본(ROS 비의존 코어)을 빌드해 계약 기준 구현과 대조한다. 저장소 루트를 첫 인자로 (기본: 이 파일 기준 ../../..)
set -euo pipefail
ROOT="${1:-$(cd "$(dirname "$0")/../../.." && pwd)}"
OUT="$(mktemp -d)"
N="$ROOT/src/amr_navigation"
PE="$ROOT/src/amr_perception"
g++ -std=c++17 -O1 -I"$N/include" -I"$PE/include" -I/usr/include/eigen3 \
  "$(dirname "$0")/crosscheck_cpp.cpp" \
  "$N/src/core/dwa.cpp" "$N/src/core/velocity_obstacle.cpp" "$N/src/core/geometry.cpp" \
  "$N/src/core/speed_profile.cpp" "$N/src/core/crossing_yield.cpp" "$N/src/core/crossing_gate.cpp" \
  "$N/src/core/jerk_limiter.cpp" "$N/src/core/velocity_profiler.cpp" "$N/src/core/pid.cpp" \
  "$PE/src/safety_gate.cpp" "$PE/src/geometry2d.cpp" -o "$OUT/crosscheck"
"$OUT/crosscheck" | python3 "$(dirname "$0")/crosscheck_cpp.py"
