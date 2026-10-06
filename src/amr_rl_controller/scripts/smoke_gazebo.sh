#!/usr/bin/env bash
# 단일 로봇 Gazebo 스모크 (gz_slot.sh 아래 포그라운드, amr-ai/rl:0.3, 워크트리 = /ros2_ws, 출력 = /runs/smoke/<tag>):
#   system.launch.py (navigation 끔) + rl_navigation.launch.py (replace_dwa: RLController 가 'DWA' 자리)
#   + amr_bringup lifecycle_watchdog → rl_smoke.py follow 5 m → navigate (GOAL, 기본 16,-8,π/2).
#   예: /home/CAPYI_AI/env/gz_slot.sh r3-controller docker run --rm --gpus all --shm-size=8g \
#         -e ROS_DOMAIN_ID=204 -e IGN_PARTITION=amr_ai_r3 -e ROS_LOCALHOST_ONLY=1 \
#         -v <wt>:/ros2_ws -v <runs>:/runs -w /ros2_ws amr-ai/rl:0.3 \
#         bash src/amr_rl_controller/scripts/smoke_gazebo.sh <tag> <model> <B|C>
# 선택 환경 변수 (A2): OUT_ROOT (기본 /runs/smoke), WORLD (system.launch.py world:= — 예: 움직이는 장애물을 뺀
#   사본, scripts/make_static_world.py), GOALS ("x,y,yaw;x,y,yaw;…" — 주면 navigate 를 차례로, 결과 goal_<k>/),
#   FOLLOW_LAST=1 (follow 를 navigate 뒤에), NAV_TIMEOUT (기본 180 s),
#   BASELINE_DWA=1 (대조: replace_dwa 없이 띄워 BT 기본 제어기 'DWA' = 원래 amr_navigation DWA 가 navigate·follow 를
#   몬다 — 같은 스택·같은 파라미터에서 제어기만 바뀐다. RL 은 'RL' 로 실리지만 쓰지 않는다).
set +u
TAG=$1; MODEL=$2; MODE=$3
OUT=${OUT_ROOT:-/runs/smoke}/$TAG; mkdir -p $OUT
WORLD_ARG=""; [ -n "$WORLD" ] && WORLD_ARG="world:=$WORLD"
source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash
export ROS_WS=/ros2_ws
echo "loadavg start $(cat /proc/loadavg)" > $OUT/load.txt
echo "env ROS_DOMAIN_ID=$ROS_DOMAIN_ID IGN_PARTITION=$IGN_PARTITION ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY" >> $OUT/load.txt
ros2 launch amr_bringup system.launch.py with_navigation:=false with_fleet:=false \
  with_dashboard:=false with_evaluation:=false with_behavior:=false headless:=true $WORLD_ARG \
  > $OUT/system.log 2>&1 &
SP=$!
sleep ${WAIT_SYS:-60}
REPLACE=true; [ "$BASELINE_DWA" = "1" ] && REPLACE=false
ros2 launch amr_rl_controller rl_navigation.launch.py model_path:=$MODEL mode:=$MODE \
  replace_dwa:=$REPLACE robot_name:=amr_01 use_sim_time:=true > $OUT/nav.log 2>&1 &
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
# 실제로 실린 판정 파라미터 (A2: 목표 yaw 오차가 0.16–0.25 rad 인데 성공으로 끝나 확인한다)
for prm in general_goal_checker.xy_goal_tolerance general_goal_checker.yaw_goal_tolerance \
    general_goal_checker.stateful current_goal_checker goal_checker_plugins controller_frequency \
    DWA.plugin DWA.rl.model_path DWA.rl.mode DWA.rl.max_inference_ms; do
  echo "controller_server $prm: $(timeout 15 ros2 param get /amr_01/controller_server $prm 2>&1)" >> $OUT/params.txt
done
for prm in default_nav_to_pose_bt_xml goal_reached_tol; do
  echo "bt_navigator $prm: $(timeout 15 ros2 param get /amr_01/bt_navigator $prm 2>&1)" >> $OUT/params.txt
done
timeout 30 ros2 topic echo --once /amr_01/scan_filtered --field header > $OUT/scan_once.txt 2>&1
follow() {
  ros2 run amr_rl_controller rl_smoke.py --mode follow --controller-id DWA --length 5 --timeout 120 \
    --out $OUT/follow --ros-args -r __ns:=/amr_01 -p use_sim_time:=true > $OUT/follow.log 2>&1
}
[ "$FOLLOW_LAST" = "1" ] || follow
if [ -n "$GOALS" ]; then
  k=0
  IFS=';' read -ra GL <<< "$GOALS"
  for g in "${GL[@]}"; do
    k=$((k+1))
    echo "goal $k $g start $(date -Iseconds) load $(cut -d' ' -f1-3 /proc/loadavg)" >> $OUT/goals.txt
    # --goal=… : 음수 좌표("-6.0,…")를 argparse 가 옵션으로 읽지 않게 (A2 1 회차 실행에서 목표 2 개를 잃었다)
    ros2 run amr_rl_controller rl_smoke.py --mode navigate --goal=$g --timeout ${NAV_TIMEOUT:-180} \
      --out $OUT/goal_$k --ros-args -r __ns:=/amr_01 -p use_sim_time:=true > $OUT/goal_$k.log 2>&1
    echo "goal $k end $(date -Iseconds)" >> $OUT/goals.txt
  done
else
  ros2 run amr_rl_controller rl_smoke.py --mode navigate --goal ${GOAL:-16.0,-8.0,1.5708} \
    --timeout 180 --out $OUT/navigate --ros-args -r __ns:=/amr_01 -p use_sim_time:=true \
    > $OUT/navigate.log 2>&1
fi
[ "$FOLLOW_LAST" = "1" ] && follow
echo "loadavg end $(cat /proc/loadavg)" >> $OUT/load.txt
kill -INT $WP $NP; sleep 5; kill -INT $SP; sleep 10; kill -9 $WP $NP $SP 2>/dev/null
grep -E "RLController '|DWA 폴백|RLController:|Failed|failed to|ERROR" $OUT/nav.log | head -200 > $OUT/rl_lines.txt
