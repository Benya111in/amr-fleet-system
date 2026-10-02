#!/usr/bin/env bash
# 컨테이너 엔트리포인트 — ROS2 + 워크스페이스 오버레이 + Gazebo 리소스 경로를 소싱하고 명령을 실행한다.
# 소싱 내용은 /etc/amr/ros_env.sh (docker/ros_env.sh) 에 있고, 대화형 bash(/etc/bash.bashrc)도 같은
# 파일을 쓴다. docker compose exec 는 엔트리포인트를 거치지 않는다 (대화형 bash 만 bashrc 로 소싱).
set -e

source /etc/amr/ros_env.sh

exec "$@"
