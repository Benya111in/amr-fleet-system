#!/usr/bin/env bash
# 단일 로봇 Gazebo 스모크 (gz_slot.sh 아래 포그라운드, amr-ai/rl:0.3, 워크트리 = /ros2_ws, 출력 = /runs/smoke/<tag>):
#   system.launch.py (navigation 끔) + rl_navigation.launch.py (replace_dwa: RLController 가 'DWA' 자리)
#   + amr_bringup lifecycle_watchdog → rl_smoke.py follow 5 m → navigate (GOAL, 기본 16,-8,π/2).
#   예: /home/CAPYI_AI/env/gz_slot.sh r3-controller docker run --rm --gpus all --shm-size=8g \
#         -e ROS_DOMAIN_ID=204 -e IGN_PARTITION=amr_ai_r3 -e ROS_LOCALHOST_ONLY=1 \
#         -v <wt>:/ros2_ws -v <runs>:/runs -w /ros2_ws amr-ai/rl:0.3 \
#         bash src/amr_rl_controller/scripts/smoke_gazebo.sh <tag> <model> <B|C>
set +u
TAG=$1; MODEL=$2; MODE=$3
OUT=/runs/smoke/$TAG; mkdir -p $OUT
source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash
export ROS_WS=/ros2_ws
echo "loadavg start $(cat /proc/loadavg)" > $OUT/load.txt
echo "env ROS_DOMAIN_ID=$ROS_DOMAIN_ID IGN_PARTITION=$IGN_PARTITION ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY" >> $OUT/load.txt
ros2 launch amr_bringup system.launch.py with_navigation:=false with_fleet:=false \
  with_dashboard:=false with_evaluation:=false with_behavior:=false headless:=true \
  > $OUT/system.log 2>&1 &
SP=$!
sleep ${WAIT_SYS:-60}
ros2 launch amr_rl_controller rl_navigation.launch.py model_path:=$MODEL mode:=$MODE \
  replace_dwa:=true robot_name:=amr_01 use_sim_time:=true > $OUT/nav.log 2>&1 &
NP=$!
# lifecycle 전이 응답 유실 안전망 (amr_bringup lifecycle_watchdog — bringup 이 navigation 을 켤 때 쓰는 것과 같음).
# 1 차 실행(gz_c_heur_a)에서 controller_server change_state 응답이 유실돼 매니저가 멈췄다.
ros2 run amr_bringup lifecycle_watchdog --ros-args -p use_sim_time:=false \
  -p nodes:="['/amr_01/planner_server','/amr_01/controller_server','/amr_01/behavior_server','/amr_01/bt_navigator']" \
  -p grace:=${GRACE:-40.0} > $OUT/watchdog.log 2>&1 &
WP=$!
for i in $(seq 1 60); do
  sleep 5
  ok=0
  for n in planner_server controller_server behavior_server bt_navigator; do
    timeout 10 ros2 lifecycle get /amr_01/$n 2>/dev/null | grep -q "^active" && ok=$((ok+1))
  done
  echo "t=$((i*5))s active=$ok/4" >> $OUT/lifecycle.txt
  [ $ok -eq 4 ] && break
done
ros2 topic list > $OUT/topics.txt 2>&1
timeout 30 ros2 topic echo --once /amr_01/scan_filtered --field header > $OUT/scan_once.txt 2>&1
ros2 run amr_rl_controller rl_smoke.py --mode follow --controller-id DWA --length 5 --timeout 120 \
  --out $OUT/follow --ros-args -r __ns:=/amr_01 -p use_sim_time:=true > $OUT/follow.log 2>&1
ros2 run amr_rl_controller rl_smoke.py --mode navigate --goal ${GOAL:-16.0,-8.0,1.5708} \
  --timeout 180 --out $OUT/navigate --ros-args -r __ns:=/amr_01 -p use_sim_time:=true \
  > $OUT/navigate.log 2>&1
echo "loadavg end $(cat /proc/loadavg)" >> $OUT/load.txt
kill -INT $WP $NP; sleep 5; kill -INT $SP; sleep 10; kill -9 $WP $NP $SP 2>/dev/null
grep -E "RLController '|DWA 폴백|RLController:|Failed|failed to|ERROR" $OUT/nav.log | head -40 > $OUT/rl_lines.txt
