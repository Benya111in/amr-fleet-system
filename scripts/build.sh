#!/usr/bin/env bash
# 컨테이너 안에서 colcon 워크스페이스를 빌드한다.
# 명세 7장 제약: colcon build 시 에러/경고가 없어야 한다 → -Werror 로 경고를 승격.
set -euo pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash
cd "${ROS_WS}"

colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic" \
    --event-handlers console_direct+ \
    "$@"

echo
echo "빌드 완료. 다음 명령으로 환경을 소싱한다:"
echo "  source ${ROS_WS}/install/setup.bash"
