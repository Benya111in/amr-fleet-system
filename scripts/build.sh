#!/usr/bin/env bash
# 컨테이너 안에서 colcon 워크스페이스를 빌드한다.
# 명세 7장 제약: colcon build 시 에러/경고가 없어야 한다.
#   기본     -Wall -Wextra -Wpedantic 으로 경고를 모두 드러낸다 (빌드는 계속된다).
#   WERROR=1 ./scripts/build.sh  → -Werror 를 더해 경고를 에러로 승격한다 (CI/리뷰 검증용).
set -euo pipefail

cd "${ROS_WS}"

# ROS setup.bash 는 미정의 변수(AMENT_TRACE_SETUP_FILES)를 참조하므로 소싱 동안만 -u 를 끈다.
set +u
source /opt/ros/"${ROS_DISTRO}"/setup.bash
set -u

CXX_FLAGS="-Wall -Wextra -Wpedantic"
if [ "${WERROR:-0}" = "1" ]; then
    CXX_FLAGS="${CXX_FLAGS} -Werror"
    echo "WERROR=1: 경고를 에러로 승격한다 (${CXX_FLAGS})"
fi

colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="${CXX_FLAGS}" \
    --event-handlers console_direct+ \
    "$@"

echo
echo "빌드 완료. 다음 명령으로 환경을 소싱한다:"
echo "  source ${ROS_WS}/install/setup.bash"
