#!/usr/bin/env bash
# ROS2 + 워크스페이스 오버레이 + Gazebo 리소스 경로 — 컨테이너의 모든 셸이 공통으로 소싱한다.
#   엔트리포인트(docker run / compose up / compose run)와 대화형 bash(/etc/bash.bashrc ← compose exec dev bash) 양쪽.
# 두 번 소싱해도 결과가 같다 (ROS setup.bash 는 자체 멱등, 리소스 경로는 아래서 중복 검사).
# source 대상이므로 set -e/-u 나 exit 를 쓰지 않는다 (ROS setup.bash 가 미정의 변수를 참조한다).

source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash

# 빌드된 워크스페이스가 있으면 오버레이로 소싱 (최초 실행 시엔 없음)
if [ -f "${ROS_WS:-/ros2_ws}/install/setup.bash" ]; then
    source "${ROS_WS:-/ros2_ws}/install/setup.bash"
fi

# Gazebo Fortress 리소스 경로 — 프로젝트 월드/모델을 기존 값 *앞에* 붙인다.
# Dockerfile ENV 가 기본값을 주지만 `-e IGN_GAZEBO_RESOURCE_PATH=...` / compose environment / .env 로
# 넘긴 값은 그 기본값을 통째로 대체하므로, 여기서 프로젝트 경로를 다시 앞에 둔다.
# 이미 들어 있으면 다시 넣지 않고, GZ_SIM_RESOURCE_PATH(Garden 이후 이름)는 같은 값으로 맞춘다.
_amr_gz_res="${ROS_WS:-/ros2_ws}/src/amr_simulation/worlds:${ROS_WS:-/ros2_ws}/src/amr_simulation/models"
case ":${IGN_GAZEBO_RESOURCE_PATH:-}:" in
    *":${_amr_gz_res}:"*) ;;
    *) export IGN_GAZEBO_RESOURCE_PATH="${_amr_gz_res}${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}" ;;
esac
export GZ_SIM_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH}"
unset _amr_gz_res
