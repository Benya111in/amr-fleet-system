#!/usr/bin/env bash
# amr_dwa2d_py 모듈과 골든 대조용 독립 실행 파일 빌드 (amr-ai/rl:0.3 컨테이너 안에서).
# 원본 코어 소스(src/amr_navigation/src/core)는 읽기 전용으로 컴파일만 한다. 기존 amr_core_py(build.sh)와 따로다.
#   bash rl/sim2d/dwa_binding/build_dwa2d.sh
#   → rl/sim2d/dwa_binding/build/amr_dwa2d_py*.so, rl/sim2d/dwa_binding/build/dwa_golden_main
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../../.." && pwd)"
CORE="${REPO}/src/amr_navigation/src/core"
OUT="${HERE}/build"
mkdir -p "${OUT}"
EXT="$(python3 -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
CORE_SRCS=("${CORE}/dwa.cpp" "${CORE}/velocity_obstacle.cpp" "${CORE}/crossing_gate.cpp"
           "${CORE}/crossing_yield.cpp" "${CORE}/speed_profile.cpp" "${CORE}/geometry.cpp"
           "${CORE}/footprint.cpp" "${CORE}/grid.cpp" "${CORE}/scan_denoise.cpp")
FLAGS=(-O2 -std=c++17 -Wall -Wextra -Werror -I "${REPO}/src/amr_navigation/include")
# shellcheck disable=SC2046
g++ "${FLAGS[@]}" -shared -fPIC -pthread $(python3 -m pybind11 --includes) \
  "${HERE}/dwa2d_py.cpp" "${CORE_SRCS[@]}" -o "${OUT}/amr_dwa2d_py${EXT}"
echo "built ${OUT}/amr_dwa2d_py${EXT}"
g++ "${FLAGS[@]}" "${HERE}/dwa_golden_main.cpp" "${CORE_SRCS[@]}" -o "${OUT}/dwa_golden_main"
echo "built ${OUT}/dwa_golden_main"
