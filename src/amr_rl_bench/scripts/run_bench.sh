#!/usr/bin/env bash
# Gazebo 다회 에피소드 평가 한 세션 (컨테이너 안, gz_slot.sh 아래 포그라운드, 워크트리 = /ros2_ws).
#
#   bash src/amr_rl_bench/scripts/run_bench.sh <출력 디렉터리> <dwa|rl|rl_handover> [gzbench_runner 인자 …]
#
# 띄우는 것 (A2 스모크와 같은 스택 — 제어기만 바뀐다):
#   amr_bringup system.launch.py (navigation 끔, world:=$WORLD — 정적 월드 사본) +
#   amr_rl_controller rl_navigation.launch.py (rl·rl_handover: replace_dwa — BT 기본 제어기 'DWA' 자리를 RL 이,
#   dwa: replace_dwa 없이 — RL 은 'RL' 로 실리지만 쓰지 않는다) + amr_bringup lifecycle_watchdog
#   → Nav2 수명주기 4/4 active 확인 → gzbench_runner.py (시나리오를 연달아) → 종료.
# 환경 변수: WORLD (필수, 정적 월드 sdf), MODEL (ONNX, 필수 — dwa 도 RL 플러그인을 싣는다), MODE (기본 B),
#   HANDOVER_RADIUS (rl_handover, 기본 0.4), WAIT_SYS (기본 60 s), GRACE (watchdog, 기본 40 s),
#   EXTRA_OVERLAY (덧씌울 작업공간 setup.bash), COMMIT (실행기 행에 남길 커밋).
set +u
OUT=$1; CTRL=$2; shift 2
[ -n "$OUT" ] && [ -n "$CTRL" ] || { echo "사용: run_bench.sh <out> <dwa|rl|rl_handover> …" >&2; exit 2; }
[ -n "$WORLD" ] && [ -n "$MODEL" ] || { echo "WORLD·MODEL 환경 변수가 필요하다" >&2; exit 2; }
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash
[ -n "$EXTRA_OVERLAY" ] && source "$EXTRA_OVERLAY"
export ROS_WS=/ros2_ws
{
  echo "loadavg start $(cat /proc/loadavg)"
  echo "env ROS_DOMAIN_ID=$ROS_DOMAIN_ID IGN_PARTITION=$IGN_PARTITION ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
  echo "controller=$CTRL world=$WORLD model=$MODEL mode=${MODE:-B} handover=${HANDOVER_RADIUS:-}"
  echo "overlay=${EXTRA_OVERLAY:-없음} amr_rl_controller=$(ros2 pkg prefix amr_rl_controller 2>&1)"
} > "$OUT/load.txt"
md5sum "$MODEL" >> "$OUT/load.txt" 2>&1
ros2 launch amr_bringup system.launch.py with_navigation:=false with_fleet:=false \
  with_dashboard:=false with_evaluation:=false with_behavior:=false headless:=true world:="$WORLD" \
  > "$OUT/system.log" 2>&1 &
SP=$!
sleep "${WAIT_SYS:-60}"
REPLACE=true; [ "$CTRL" = "dwa" ] && REPLACE=false
HO=""; [ "$CTRL" = "rl_handover" ] && HO="handover_radius:=${HANDOVER_RADIUS:-0.4}"
# shellcheck disable=SC2086
ros2 launch amr_rl_controller rl_navigation.launch.py model_path:="$MODEL" mode:="${MODE:-B}" \
  replace_dwa:=$REPLACE $HO robot_name:=amr_01 use_sim_time:=true > "$OUT/nav.log" 2>&1 &
NP=$!
ros2 run amr_bringup lifecycle_watchdog --ros-args -p use_sim_time:=false \
  -p nodes:="['/amr_01/planner_server','/amr_01/controller_server','/amr_01/behavior_server','/amr_01/bt_navigator']" \
  -p grace:="${GRACE:-40.0}" > "$OUT/watchdog.log" 2>&1 &
WP=$!
for i in $(seq 1 60); do
  sleep 5
  ok=0
  for n in planner_server controller_server behavior_server bt_navigator; do
    timeout 10 ros2 lifecycle get /amr_01/$n 2>/dev/null | grep -q "^active" && ok=$((ok+1))
  done
  echo "t=$((i*5))s active=$ok/4" >> "$OUT/lifecycle.txt"
  [ $ok -eq 4 ] && break
done
for prm in general_goal_checker.xy_goal_tolerance general_goal_checker.yaw_goal_tolerance \
    controller_frequency DWA.plugin DWA.rl.model_path DWA.rl.mode DWA.rl.handover_radius \
    RL.plugin; do
  echo "controller_server $prm: $(timeout 15 ros2 param get /amr_01/controller_server $prm 2>&1)" \
    >> "$OUT/params.txt"
done
timeout 12 ros2 topic hz /clock --window 2000 > "$OUT/clock_hz.txt" 2>&1
ros2 run amr_rl_bench gzbench_runner.py --controller "$CTRL" --out "$OUT" --commit "${COMMIT:-unknown}" \
  "$@" --ros-args -r __ns:=/amr_01 -p use_sim_time:=true > "$OUT/runner.log" 2>&1
echo "runner rc=$?" >> "$OUT/load.txt"
echo "loadavg end $(cat /proc/loadavg)" >> "$OUT/load.txt"
kill -INT $WP $NP; sleep 5; kill -INT $SP; sleep 10; kill -9 $WP $NP $SP 2>/dev/null
grep -E "RLController '|DWA 폴백|RLController:|Failed|failed to|ERROR" "$OUT/nav.log" | head -300 \
  > "$OUT/rl_lines.txt"
grep -E "localization/lost|re-initialization|uniform distribution|spin goal|LOST" "$OUT/system.log" \
  | head -300 > "$OUT/loc_events.txt"
