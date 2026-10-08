#!/usr/bin/env bash
# 시나리오 bag 녹화 (보너스 2 G 단계, b2-rec). 컨테이너(amr-ai/slam 이미지) 안에서 돈다 — 호스트 구동기
# rec_driver.sh 가 gz_slot.sh 로 감싼 docker run 으로 부른다.
#
#   record_bag.sh <출력 디렉터리> <route.yaml> <월드 변형: s1 | s6-open | s6-queue | s6-wall> [spawn.yaml | -]
#
# 프로토콜 (replay.md §9-1·2·3·5, 사전 등록 docs/research/slam3d/notebook/entries/*_b2-rec_*):
#  1) 월드: gen_warehouse_world.py --variant <V> → <출력>/world/warehouse.sdf (기본 월드 출력은 바이트 동일).
#     생성된 사본의 물리 real_time_factor 를 RTF_TARGET (기본 0.5) 으로 낮춘다 — 시운전 1 회차(sim 81 s, 실제 RTF 0.85)에서
#     EKF 타이머 지연(33–54 ms)·imu_filter 입력 버림(15 ms)·/clock 틱 누락이 공백 검사를 실패시켰다. 시뮬 시각이 벽시계보다
#     느리게 가면 처리 노드가 sim 주기마다 쓸 벽시계 여유가 늘어난다 (물리 스텝·센서 주기·주행은 sim 시각 기준이라 그대로)
#  2) multi_robot.launch.py: amr_01(3D) 1 대, localization mode odom (휠·imu_filter·scan_filter·odom EKF),
#     Nav2·perception·behavior·fleet 없음. **madgwick 은 띄우지 않는다** — 원시 IMU(imu/data_raw·imu/data)만 기록하고
#     madgwick 은 녹화 뒤 madg_lockstep.py 로 파생한다 (기록 때 madgwick 이 입력을 잃었다 — replay.md §1.2)
#  3) route_driver 를 **기록 전에** 띄운다 (0 명령). sim 10 s 동안 정지한 뒤 ready 파일 — 이 노드의 기동 부하가 bag 에
#     들어가지 않게 (파일럿의 sim 9.48 s 정체는 route_driver 기동과 겹쳤다 — replay.md §7)
#  4) ready 뒤 ros2 bag record (mcap, rows.RECORD_TOPICS). 기록기 구독 QoS 를 깊게 (record_qos.yaml: 고빈도 토픽 keep_last
#     1000 — 기본 깊이 10 이면 1 kHz /clock 은 기록기 실행기가 10 ms 만 밀려도 잃는다. 발행자는 모두 reliable·volatile
#     [측정: 시운전 bag metadata.yaml]). 기록기 로그에서 모든 토픽의 "Subscribed to topic" 을 확인하고 3 s(벽시계) 뒤 go 파일.
#     기록 중에는 새 프로세스를 띄우지 않는다 (파일 검사만)
#  5) 주행기: go 부터 hold_start_s (route.yaml, S6 은 depart_at_s 까지) 정지 → 경로 → hold_end_s 정지 → done 파일
#  6) done → 기록기 SIGINT (metadata.yaml 이 생겼는지 확인) → quit 파일 → 주행기 종료 → 런치·Gazebo 정리
#  7) 사후: bag info, stats(토픽별 수·sim 주기), gate(공백 검사 — 통과 못 하면 종료 코드 3), gt.tum, 스폰 파일 사본
#     (<bag>/b2rec_spawn.yaml — replay_row.sh 가 anchor 로 쓴다), (gate 통과면) madgwick 잠금 단계 파생 bag(bag_madg)
#
# 종료 코드: 0 = 통과, 3 = 공백 검사 실패, 4 = 기록기 구독 확인 실패, 5 = 기록기 마무리 실패, 6 = 주행 실패, 그 밖 = 도구 오류
# 환경: ROS_DOMAIN_ID·IGN_PARTITION·ROS_LOCALHOST_ONLY 는 호출자가 준다. GIT_HEAD 를 주면 env.txt 에 남긴다.
set -eo pipefail

OUT=${1:?출력 디렉터리}
ROUTE=${2:?route.yaml}
VARIANT=${3:?월드 변형}
SPAWN_IN=${4:--}
mkdir -p "$OUT"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /ros2_ws/install/setup.bash
SRC=/ros2_ws/src/amr_slam3d
SCRIPTS=$SRC/scripts
export PYTHONPATH=$SRC:${PYTHONPATH:-}
BRINGUP=$(ros2 pkg prefix amr_bringup)/share/amr_bringup
if [ "$SPAWN_IN" = "-" ]; then ROBOTS=$BRINGUP/config/fleet_spawn_lidar3d.yaml; else ROBOTS=$SPAWN_IN; fi
DEPART=$(python3 -c "import sys, yaml; r = yaml.safe_load(open(sys.argv[1]))['route']; \
print(r.get('depart_at_s') if r.get('depart_at_s') is not None else '')" "$ROUTE")

load() { local l; l=$(< /proc/loadavg); echo "$l"; }
stamp() { echo "$1: $(date -Iseconds) loadavg: $(load)" >> "$OUT/env.txt"; }
{
  echo "date: $(date -Iseconds)"
  echo "GIT_HEAD=${GIT_HEAD:-} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-} IGN_PARTITION=${IGN_PARTITION:-}"
  echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-} route=$ROUTE variant=$VARIANT robots=$ROBOTS depart_at=$DEPART"
  echo "loadavg: $(load)"
  nvidia-smi --query-gpu=name,utilization.gpu,memory.used,memory.total --format=csv,noheader \
    2> /dev/null || true
} > "$OUT/env.txt"
\cp -f "$ROUTE" "$OUT/route.yaml"
\cp -f "$ROBOTS" "$OUT/spawn.yaml"

# 비대화형 bash 의 `&` 작업은 SIGINT 를 무시한 채 시작한다 (replay.md §7) — 처분을 되돌린 뒤 exec 한다 (배열 = 단순 명령)
SIGDFL=(python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])')
PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2> /dev/null || true; done
  for p in "${PIDS[@]}"; do
    for _ in $(seq 1 20); do kill -0 "$p" 2> /dev/null || break; sleep 1; done
    kill -KILL "$p" 2> /dev/null || true
  done
  # 런치가 남긴 Gazebo 서버 (이 컨테이너 PID 공간 안에서만)
  for p in $(pgrep -f 'ign gazebo' || true); do [ "$p" != "$$" ] && kill -KILL "$p" 2> /dev/null || true; done
}
trap cleanup EXIT

wait_for_topic() {   # $1 토픽, $2 상한 [s]
  local end=$((SECONDS + $2))
  while [ $SECONDS -lt $end ]; do
    if ros2 topic list 2> /dev/null | grep -qx "$1"; then return 0; fi
    sleep 3
  done
  echo "토픽 대기 초과: $1" >&2
  return 1
}
wait_file() {        # $1 파일, $2 상한 [s], $3 감시할 PID (죽으면 실패)
  local end=$((SECONDS + $2))
  while [ $SECONDS -lt $end ]; do
    [ -e "$1" ] && return 0
    if [ -n "${3:-}" ] && ! kill -0 "$3" 2> /dev/null; then echo "프로세스 $3 종료 — $1 없음" >&2; return 1; fi
    sleep 0.5
  done
  echo "파일 대기 초과: $1" >&2
  return 1
}

mkdir -p "$OUT/world"
python3 /ros2_ws/src/amr_simulation/worlds/gen_warehouse_world.py --variant "$VARIANT" \
  "$OUT/world/warehouse.sdf" > "$OUT/world.log" 2>&1
echo "generated: $(sha256sum < "$OUT/world/warehouse.sdf")" >> "$OUT/world.log"
RTF_TARGET=${RTF_TARGET:-0.5}
sed -i "s#<real_time_factor>1.0</real_time_factor>#<real_time_factor>$RTF_TARGET</real_time_factor>#" \
  "$OUT/world/warehouse.sdf"
if [ "$(grep -c "<real_time_factor>$RTF_TARGET</real_time_factor>" "$OUT/world/warehouse.sdf")" != 1 ]; then
  echo "real_time_factor 를 $RTF_TARGET 로 못 바꿨다" >&2
  exit 7
fi
echo "rtf_target: $RTF_TARGET" >> "$OUT/env.txt"
echo "used (rtf_target $RTF_TARGET): $(sha256sum < "$OUT/world/warehouse.sdf")" >> "$OUT/world.log"

stamp launch_start
"${SIGDFL[@]}" ros2 launch amr_bringup multi_robot.launch.py robots_file:="$ROBOTS" num_robots:=1 \
  world:="$OUT/world/warehouse.sdf" localization_mode:=odom \
  with_localization:=true with_navigation:=false with_perception:=false with_behavior:=false \
  with_fleet:=false with_dashboard:=false with_evaluation:=false lifecycle_watchdog_grace:=0 \
  > "$OUT/launch.log" 2>&1 &
LAUNCH=$!
PIDS+=("$LAUNCH")
wait_for_topic /amr_01/odometry/filtered 900
wait_for_topic /amr_01/lidar3d/points 300
wait_for_topic /amr_01/ground_truth/odom 300
stamp stack_up

M="$OUT/markers"
mkdir -p "$M"
"${SIGDFL[@]}" python3 "$SCRIPTS/route_driver.py" --route "$ROUTE" --robot amr_01 \
  --log "$OUT/route.jsonl" --ready-file "$M/ready" --ready-after 10 --go-file "$M/go" \
  --done-file "$M/done" --quit-file "$M/quit" > "$OUT/route.log" 2>&1 &
DRIVER=$!
PIDS+=("$DRIVER")
wait_file "$M/ready" 600 "$DRIVER"
stamp driver_ready

TOPICS=$(python3 -c "from amr_slam3d.rows import RECORD_TOPICS; print(' '.join(RECORD_TOPICS))")
NT=$(echo "$TOPICS" | wc -w)
echo "record: $TOPICS" >> "$OUT/env.txt"
QOS="$OUT/record_qos.yaml"
# shellcheck disable=SC2086
python3 -c "from amr_slam3d import rows; import sys; sys.stdout.write(rows.record_qos_yaml())" > "$QOS"
# shellcheck disable=SC2086
"${SIGDFL[@]}" ros2 bag record -s mcap -o "$OUT/bag" --qos-profile-overrides-path "$QOS" $TOPICS \
  > "$OUT/record.log" 2>&1 &
REC=$!
PIDS+=("$REC")
stamp record_start
SUB_OK=0
for _ in $(seq 1 240); do
  n=$(grep -o "Subscribed to topic '[^']*'" "$OUT/record.log" 2> /dev/null | sort -u | wc -l || true)
  if [ "${n:-0}" -ge "$NT" ]; then SUB_OK=1; break; fi
  kill -0 "$REC" 2> /dev/null || break
  sleep 0.5
done
echo "record_subscribed: $SUB_OK ($(grep -o "Subscribed to topic '[^']*'" "$OUT/record.log" | sort -u | wc -l)/$NT)" \
  >> "$OUT/env.txt"
if [ "$SUB_OK" != 1 ]; then
  echo "기록기 구독 확인 실패" >&2
  exit 4
fi
sleep 3
touch "$M/go"
stamp go

# 주행: 벽시계 상한 (넘으면 주행 실패)
set +e
wait_file "$M/done" "${DRIVE_TIMEOUT_WALL:-5400}" "$DRIVER"
DRIVE_OK=$?
set -e
stamp drive_done
kill -INT "$REC" 2> /dev/null || true
for _ in $(seq 1 120); do kill -0 "$REC" 2> /dev/null || break; sleep 1; done
REC_CLEAN=0
if ! kill -0 "$REC" 2> /dev/null && [ -f "$OUT/bag/metadata.yaml" ]; then REC_CLEAN=1; fi
echo "record_stop_clean: $REC_CLEAN" >> "$OUT/env.txt"
touch "$M/quit"
for _ in $(seq 1 20); do kill -0 "$DRIVER" 2> /dev/null || break; sleep 1; done
cleanup
trap - EXIT
stamp sim_stopped
if [ "$DRIVE_OK" != 0 ]; then echo "주행 실패 (done 없음)" >&2; exit 6; fi
if [ "$REC_CLEAN" != 1 ]; then echo "기록기 마무리 실패 (metadata.yaml 없음)" >&2; exit 5; fi

# ---- 사후: 메타데이터·공백 검사·정답·파생 ----
\cp -f "$ROBOTS" "$OUT/bag/b2rec_spawn.yaml"
ros2 bag info "$OUT/bag" > "$OUT/bag_info.txt" 2>&1 || true
python3 "$SCRIPTS/bag_tools.py" stats "$OUT/bag" --out "$OUT/bag_meta.json" > "$OUT/bag_meta.txt" 2>&1
GATE_ARGS=()
if [ -n "$DEPART" ]; then GATE_ARGS=(--depart-at "$DEPART"); fi
set +e
python3 "$SCRIPTS/bag_tools.py" gate "$OUT/bag" --out "$OUT/gate.json" "${GATE_ARGS[@]}" \
  > "$OUT/gate.txt" 2>&1
GATE_RC=$?
set -e
echo "gate_rc: $GATE_RC" >> "$OUT/env.txt"
python3 "$SCRIPTS/bag_tools.py" tum "$OUT/bag" --topic /amr_01/ground_truth/odom --out "$OUT/gt.tum" \
  > "$OUT/gt.log" 2>&1
stamp post_done
if [ "$GATE_RC" != 0 ]; then exit 3; fi

# 파생 bag (LIO-SAM·IMU 행용 — 재계산 madgwick 0.1/0.01 + 오라클 방위 토픽). 실패해도 녹화 판정은 그대로 (따로 다시 만든다)
mkdir -p "$OUT/derive"
set +e
python3 "$SCRIPTS/madg_lockstep.py" --in-bag "$OUT/bag" --out-bag "$OUT/bag_madg" \
  --report "$OUT/derive/derive.json" \
  --gain 0.1=/amr_01/imu/data_madgwick_re --gain 0.01=/amr_01/imu/data_madgwick_re_g001 \
  > "$OUT/derive.log" 2>&1
DERIVE_RC=$?
set -e
if [ -d "$OUT/bag_madg" ]; then \cp -f "$ROBOTS" "$OUT/bag_madg/b2rec_spawn.yaml"; fi
echo "derive_rc: $DERIVE_RC" >> "$OUT/env.txt"
stamp all_done
exit 0
