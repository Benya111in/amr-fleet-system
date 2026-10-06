#!/usr/bin/env bash
# env 안 교차 시험: C++ 원본 SafetyGate 를 ROS 없이 직접 빌드하고 replay_env.py 로 Isaac 결선 기록
# (scripts/record_phi_wiring.py 산출)을 불변식 검사·재생 대조한다. amr-ai/rl:0.3 안에서:
#   rl/isaac/tools/phi_crosscheck/replay_env.sh <기록 디렉터리> [--out <json>]
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../../.." && pwd)
PE=$ROOT/src/amr_perception
BIN=${XCHECK_BIN:-$(mktemp -d)/gate_runner}
g++ -std=c++17 -O1 -Wall -Wextra -I"$PE/include" -I/usr/include/eigen3 \
  "$HERE/gate_runner.cpp" "$PE/src/safety_gate.cpp" "$PE/src/geometry2d.cpp" -o "$BIN"
python3 "$HERE/replay_env.py" --binary "$BIN" "$@"
