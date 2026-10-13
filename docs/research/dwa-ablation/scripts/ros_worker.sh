#!/usr/bin/env bash
# ROS 시험대 한 벌 (도메인 하나): kinematic_sim.launch.py 를 띄우고 명령을 돌린 뒤 내린다.
#   ros_worker.sh <domain> <ns> <params.yaml> <launch.log> [noise_seed odom_noise_v odom_noise_w] -- <명령...>
# 명령 안의 @NS@ 는 ns 로 바뀐다. 컨테이너 amr-ai-dwa-abl-dev 안에서 돈다.
set -u
DOM=$1; NS=$2; PARAMS=$3; LOG=$4; shift 4
NOISE=()
if [ "$1" != "--" ]; then
  NOISE=(noise_seed:=$1 odom_noise_v:=$2 odom_noise_w:=$3); shift 3
fi
shift   # --
set +u; source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
export ROS_DOMAIN_ID=$DOM ROS_LOCALHOST_ONLY=1 ROS_WS=/ros2_ws
setsid ros2 launch amr_navigation kinematic_sim.launch.py robot_name:=$NS \
  map:=/runs/paths/composite.yaml params_file:=$PARAMS use_ttc_bt:=false log_level:=warn \
  "${NOISE[@]}" > "$LOG" 2>&1 &
LPID=$!
CMD=("${@//@NS@/$NS}")
"${CMD[@]}"; rc=$?
kill -INT -- -$LPID 2>/dev/null
for i in $(seq 1 30); do kill -0 $LPID 2>/dev/null || break; sleep 1; done
kill -KILL -- -$LPID 2>/dev/null
wait $LPID 2>/dev/null
exit $rc
