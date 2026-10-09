#!/usr/bin/env bash
# amr_core_py (팀 A* · 팽창 · 평활) 모듈을 layopt/sim/native/build/ 에 빌드한다 (amr-ai/rl:0.3 컨테이너 안에서).
#   bash layopt/sim/native/build.sh
# 원본 코어 소스(src/amr_navigation/src/core)는 읽기 전용으로 컴파일만 한다. 바인딩은 rl/sim2d 의 사본(amr_core_py.cpp).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../../.." && pwd)"
CORE="${REPO}/src/amr_navigation/src/core"
OUT="${HERE}/build"
mkdir -p "${OUT}"
EXT="$(python3 -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
SRCS=("${HERE}/amr_core_py.cpp" "${CORE}/astar.cpp" "${CORE}/grid.cpp" "${CORE}/path_smoother.cpp"
      "${CORE}/geometry.cpp")
# shellcheck disable=SC2046
g++ -O2 -std=c++17 -shared -fPIC -Wall -Wextra -Werror $(python3 -m pybind11 --includes) \
  -I "${REPO}/src/amr_navigation/include" -I /usr/include/eigen3 "${SRCS[@]}" -o "${OUT}/amr_core_py${EXT}"
echo "built ${OUT}/amr_core_py${EXT}"
