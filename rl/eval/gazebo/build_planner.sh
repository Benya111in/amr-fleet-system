#!/usr/bin/env bash
# 팀 A* 코어 파이썬 모듈(amr_core_py)을 <출력 디렉터리>에 빌드한다 (amr-ai/rl:0.3 컨테이너 안).
#   bash rl/eval/gazebo/build_planner.sh /out/planner
# 소스·플래그는 rl/sim2d/dwa_binding/build.sh 와 같다 (그 디렉터리에 빌드 산출물을 쓰지 않으려고 따로 둔다).
set -euo pipefail
OUT=${1:?출력 디렉터리}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../../.." && pwd)"
B="${REPO}/rl/sim2d/dwa_binding"
CORE="${REPO}/src/amr_navigation/src/core"
PE="${REPO}/src/amr_perception"
mkdir -p "${OUT}"
EXT="$(python3 -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
SRCS=("${B}/amr_core_py.cpp" "${CORE}/astar.cpp" "${CORE}/grid.cpp" "${CORE}/path_smoother.cpp"
      "${CORE}/geometry.cpp" "${B}/safety_py.cpp" "${PE}/src/safety_gate.cpp" "${PE}/src/geometry2d.cpp")
if [ -f "${B}/dwa_py.cpp" ]; then
  SRCS+=("${B}/dwa_py.cpp" "${CORE}/dwa.cpp" "${CORE}/velocity_obstacle.cpp" "${CORE}/footprint.cpp"
         "${CORE}/crossing_gate.cpp" "${CORE}/crossing_yield.cpp")
fi
# shellcheck disable=SC2046
g++ -O2 -std=c++17 -shared -fPIC -Wall -Wextra -Werror $(python3 -m pybind11 --includes) \
  -I "${REPO}/src/amr_navigation/include" -I "${PE}/include" -I /usr/include/eigen3 "${SRCS[@]}" \
  -o "${OUT}/amr_core_py${EXT}"
echo "built ${OUT}/amr_core_py${EXT}"
