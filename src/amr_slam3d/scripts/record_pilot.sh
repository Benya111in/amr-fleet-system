#!/usr/bin/env bash
# 파일럿 bag 기록 (보너스 2 4단계). 컨테이너(amr-ai/slam 이미지) 안에서 돈다 — 호스트에서는 gz_slot.sh 로 감싼
# docker run 이 부른다 (docs/research/slam3d/rtabmap.md "재현").
#
#   record_pilot.sh <출력 디렉터리> [route.yaml]
#
# 1) 동적 행위자를 뺀 월드 사본 (amr_lidar3d make_static_world.py — 보고서 S1 '정상, 동적 장애물 정지')
# 2) multi_robot.launch.py: amr_01(3D) 1대, localization 스택 mode:=odom (휠 오도메트리·imu_filter·scan_filter·
#    odom EKF 만 — map→odom 발행자 없음), Nav2·perception·behavior·fleet 없음
# 3) imu_filter_madgwick (방위 재추정) → imu/data_madgwick 도 bag 에 남겨 모든 SLAM 행이 같은 IMU 를 받게 한다
# 4) ros2 bag record (mcap) 를 켠 뒤 route_driver.py 가 GT 되먹임으로 경로를 달린다. 끝나면 기록을 멈춘다
# 5) bag 메타데이터(토픽별 수·주기), 정답 궤적 gt.tum, 경로 단계 기록 route.jsonl
#
# 환경: ROS_DOMAIN_ID·IGN_PARTITION·ROS_LOCALHOST_ONLY 는 호출자가 준다. GIT_HEAD 를 주면 env.txt 에 남긴다.
set -eo pipefail

OUT=${1:?출력 디렉터리}
ROUTE=${2:-}
mkdir -p "$OUT"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /ros2_ws/install/setup.bash
SHARE=$(ros2 pkg prefix amr_slam3d)/share/amr_slam3d
LIB=$(ros2 pkg prefix amr_slam3d)/lib/amr_slam3d
L3D=$(ros2 pkg prefix amr_lidar3d)/lib/amr_lidar3d
SIM=$(ros2 pkg prefix amr_simulation)/share/amr_simulation
BRINGUP=$(ros2 pkg prefix amr_bringup)/share/amr_bringup
ROUTE=${ROUTE:-$SHARE/config/pilot_route.yaml}
ROBOTS=$BRINGUP/config/fleet_spawn_lidar3d.yaml

{
  echo "date: $(date -Iseconds)"
  echo "GIT_HEAD=${GIT_HEAD:-} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-} IGN_PARTITION=${IGN_PARTITION:-}"
  echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-} route=$ROUTE robots=$ROBOTS"
  echo "loadavg: $(cat /proc/loadavg)"
  nvidia-smi --query-gpu=name,utilization.gpu,memory.used,memory.total --format=csv,noheader \
    2>/dev/null || true
} > "$OUT/env.txt"
\cp -f "$ROUTE" "$OUT/route.yaml"

PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do
    kill -INT "$p" 2>/dev/null || true
  done
  for p in "${PIDS[@]}"; do
    for _ in $(seq 1 20); do kill -0 "$p" 2>/dev/null || break; sleep 1; done
    kill -KILL "$p" 2>/dev/null || true
  done
  # 런치가 남긴 Gazebo 서버 (이 컨테이너 PID 공간 안에서만)
  for p in $(pgrep -f 'ign gazebo' || true); do [ "$p" != "$$" ] && kill -KILL "$p" 2>/dev/null || true; done
}
trap cleanup EXIT

wait_for_topic() {   # $1 토픽, $2 상한 [s]
  local end=$((SECONDS + $2))
  while [ $SECONDS -lt $end ]; do
    if ros2 topic list 2>/dev/null | grep -qx "$1"; then return 0; fi
    sleep 3
  done
  echo "토픽 대기 초과: $1" >&2
  return 1
}

python3 "$L3D/make_static_world.py" "$SIM/worlds/warehouse.sdf" "$OUT/world" | tee "$OUT/world.log"

ros2 launch amr_bringup multi_robot.launch.py robots_file:="$ROBOTS" num_robots:=1 \
  world:="$OUT/world/warehouse.sdf" localization_mode:=odom \
  with_localization:=true with_navigation:=false with_perception:=false with_behavior:=false \
  with_fleet:=false with_dashboard:=false with_evaluation:=false lifecycle_watchdog_grace:=0 \
  > "$OUT/launch.log" 2>&1 &
PIDS+=($!)
wait_for_topic /amr_01/odometry/filtered 900
wait_for_topic /amr_01/lidar3d/points 300

ros2 launch amr_slam3d madgwick.launch.py robot:=amr_01 > "$OUT/madgwick.log" 2>&1 &
PIDS+=($!)
wait_for_topic /amr_01/imu/data_madgwick 120

TOPICS=$(python3 -c "from amr_slam3d.rows import RECORD_TOPICS; print(' '.join(RECORD_TOPICS))")
echo "record: $TOPICS" >> "$OUT/env.txt"
# shellcheck disable=SC2086
ros2 bag record -s mcap -o "$OUT/bag" $TOPICS > "$OUT/record.log" 2>&1 &
REC=$!
PIDS+=("$REC")
sleep 8        # 기록기 구독이 붙을 시간 (벽시계)

echo "drive_start_wall: $(date -Iseconds) loadavg: $(cat /proc/loadavg)" >> "$OUT/env.txt"
set +e
python3 "$LIB/route_driver.py" --route "$ROUTE" --robot amr_01 --log "$OUT/route.jsonl" \
  > "$OUT/route.log" 2>&1
DRIVE_RC=$?
set -e
echo "drive_end_wall: $(date -Iseconds) rc=$DRIVE_RC loadavg: $(cat /proc/loadavg)" >> "$OUT/env.txt"
sleep 3
kill -INT "$REC" 2>/dev/null || true
for _ in $(seq 1 60); do kill -0 "$REC" 2>/dev/null || break; sleep 1; done
cleanup
trap - EXIT

ros2 bag info "$OUT/bag" > "$OUT/bag_info.txt" 2>&1 || true
python3 "$LIB/bag_tools.py" stats "$OUT/bag" --out "$OUT/bag_meta.json" > "$OUT/bag_meta.txt" 2>&1
python3 "$LIB/bag_tools.py" tum "$OUT/bag" --topic /amr_01/ground_truth/odom --out "$OUT/gt.tum"
echo "loadavg_end: $(cat /proc/loadavg)" >> "$OUT/env.txt"
exit "$DRIVE_RC"
