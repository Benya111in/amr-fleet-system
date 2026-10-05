#!/usr/bin/env bash
# amr_core_py 모듈 빌드 (amr-ai/rl:0.3 컨테이너 안에서). 원본 코어 소스는 읽기 전용으로 컴파일만 한다.
#   bash rl/sim2d/dwa_binding/build.sh   → rl/sim2d/dwa_binding/build/amr_core_py*.so
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../../.." && pwd)"
CORE="${REPO}/src/amr_navigation/src/core"
OUT="${HERE}/build"
mkdir -p "${OUT}"
EXT="$(python3 -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
SRCS=("${HERE}/amr_core_py.cpp" "${CORE}/astar.cpp" "${CORE}/grid.cpp" "${CORE}/path_smoother.cpp"
      "${CORE}/geometry.cpp")
if [ -f "${HERE}/dwa_py.cpp" ]; then
  SRCS+=("${HERE}/dwa_py.cpp" "${CORE}/dwa.cpp" "${CORE}/velocity_obstacle.cpp" "${CORE}/footprint.cpp"
         "${CORE}/crossing_gate.cpp" "${CORE}/crossing_yield.cpp")
fi
# shellcheck disable=SC2046
g++ -O2 -std=c++17 -shared -fPIC -Wall -Wextra -Werror $(python3 -m pybind11 --includes) \
  -I "${REPO}/src/amr_navigation/include" "${SRCS[@]}" -o "${OUT}/amr_core_py${EXT}"
echo "built ${OUT}/amr_core_py${EXT}"
