#!/usr/bin/env bash
# 도킹·종단 시험 공통 환경 (일회용 컨테이너 안, 워크스페이스 /ros2_ws). OUT = 결과 디렉터리
source /etc/amr/ros_env.sh
export ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export OUT="${OUT:-/ros2_ws/log/bhv_trials}"
mkdir -p "${OUT}"
cd /ros2_ws
