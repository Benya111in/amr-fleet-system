#!/usr/bin/env bash
# 정본 safety φ 교차 시험: C++ 원본 SafetyGate(src/amr_perception)를 ROS 없이 직접 빌드하고
# crosscheck.py 로 torch 이식(rl/isaac/amr_isaac/safety_gate_torch.py)과 대조한다. amr-ai/rl:0.3 안에서:
#   rl/isaac/tools/phi_crosscheck/crosscheck.sh [crosscheck.py 인자 …]   예: --quick, --out <json>, --cuda_check
# 원본(safety_gate.cpp·geometry2d.cpp·safety_gate.hpp)이나 이식을 바꾸면 다시 돌린다.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../../.." && pwd)
PE=$ROOT/src/amr_perception
BIN=${XCHECK_BIN:-$(mktemp -d)/gate_runner}
g++ -std=c++17 -O1 -Wall -Wextra -I"$PE/include" -I/usr/include/eigen3 \
  "$HERE/gate_runner.cpp" "$PE/src/safety_gate.cpp" "$PE/src/geometry2d.cpp" -o "$BIN"
python3 "$HERE/crosscheck.py" --binary "$BIN" "$@"
