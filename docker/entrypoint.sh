#!/usr/bin/env bash
# ROS2 환경 + 워크스페이스 오버레이를 소싱하는 컨테이너 엔트리포인트
set -e

source /opt/ros/"${ROS_DISTRO}"/setup.bash

# 빌드된 워크스페이스가 있으면 오버레이로 소싱 (최초 실행 시엔 없음)
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

# Gazebo Fortress 리소스 경로에 프로젝트 모델/월드 등록
export IGN_GAZEBO_RESOURCE_PATH="${ROS_WS}/src/amr_simulation/worlds:${ROS_WS}/src/amr_simulation/models:${IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH}"

exec "$@"
