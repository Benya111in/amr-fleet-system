#!/usr/bin/env bash
# ROS2 환경 + 워크스페이스 오버레이를 소싱하는 컨테이너 엔트리포인트
set -e

source /opt/ros/"${ROS_DISTRO}"/setup.bash

# 빌드된 워크스페이스가 있으면 오버레이로 소싱 (최초 실행 시엔 없음)
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

# Gazebo 리소스 경로(IGN_GAZEBO_RESOURCE_PATH)는 Dockerfile ENV 에 있다 —
# `docker compose exec dev bash` 처럼 엔트리포인트를 거치지 않는 셸에서도 보여야 하므로.

exec "$@"
