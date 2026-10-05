#!/usr/bin/env bash
# Gazebo 없는 폐루프 스모크 (amr-ai/rl:0.3 컨테이너 안, 워크트리 = /ros2_ws, 출력 = /runs/smoke/<tag>):
#   rl_navigation.launch.py sim:=kinematic (amr_navigation kinematic_sim + navigation, RLController 덧씌우기)
#   → rl_smoke.py. 호스트에서 HEAD·diff 를 출력 디렉터리에 먼저 써 둔다 (컨테이너에는 .git 이 없다).
#   예: docker run --rm -e ROS_DOMAIN_ID=204 -e ROS_LOCALHOST_ONLY=1 -e LAUNCH_EXTRA=... \
#         -v <wt>:/ros2_ws -v <runs>:/runs -w /ros2_ws amr-ai/rl:0.3 \
#         bash src/amr_rl_controller/scripts/smoke_kinematic.sh <tag> <model> <B|C> <rl_smoke 인자…>
set +u
TAG=$1; MODEL=$2; MODE=$3; shift 3
OUT=/runs/smoke/$TAG; mkdir -p $OUT
source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash
export ROS_WS=/ros2_ws

echo "loadavg start $(cat /proc/loadavg)" > $OUT/load.txt
ros2 launch amr_rl_controller rl_navigation.launch.py sim:=kinematic model_path:=$MODEL mode:=$MODE \
  $LAUNCH_EXTRA > $OUT/launch.log 2>&1 &
LP=$!
sleep ${WAIT:-35}
ros2 run amr_rl_controller rl_smoke.py "$@" --out $OUT --ros-args -r __ns:=/amr_01 > $OUT/smoke.log 2>&1
echo "smoke rc $?" >> $OUT/smoke.log
echo "loadavg end $(cat /proc/loadavg)" >> $OUT/load.txt
kill -INT $LP; sleep 5; kill -9 $LP 2>/dev/null
grep -E "RLController '|DWA 폴백|RLController:" $OUT/launch.log | head -20 > $OUT/rl_lines.txt
