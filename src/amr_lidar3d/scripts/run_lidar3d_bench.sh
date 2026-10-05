#!/usr/bin/env bash
# 3D LiDAR 측정 구동기 (보고서 1–2단계). 컨테이너 안에서 돈다 — 호스트에서는 gz_slot.sh 로 감싼 docker run 이 부른다.
#
#   run_lidar3d_bench.sh load  <출력 디렉터리> <라벨> <robots_file> <측정 s> [프로브 --topic ...]
#       multi_robot.launch.py 를 스택·전역 노드 없이(월드 + 로봇 N대 + 센서 브리지만) 띄우고 lidar3d_probe.py 로 잰다.
#       3D 점군 토픽을 --topic 으로 주면 그 구독이 lazy 체인(전처리 → 브리지 → gz 렌더)을 켠다 = "브리지 켬".
#   run_lidar3d_bench.sh check <출력 디렉터리> [sensor_check 인자 ...]
#       동적 행위자를 뺀 월드에 amr_01(3D) 1대만 띄우고 lidar3d_sensor_check.py 를 돌린다.
#
# 환경: ROS_DOMAIN_ID·IGN_PARTITION 은 호출자가 준다 (s1-sensor: 206 / amr_ai_s1). 로그는 출력 디렉터리에 남는다.
set -eo pipefail

MODE=${1:?mode (load|check)}; OUT=${2:?출력 디렉터리}; shift 2
mkdir -p "$OUT"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /ros2_ws/install/setup.bash
SHARE_BRINGUP=$(ros2 pkg prefix amr_bringup)/share/amr_bringup
SHARE_SIM=$(ros2 pkg prefix amr_simulation)/share/amr_simulation
LIB=$(ros2 pkg prefix amr_lidar3d)/lib/amr_lidar3d

{
  echo "date: $(date -Iseconds)"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-} IGN_PARTITION=${IGN_PARTITION:-} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-}"
  echo "loadavg: $(cat /proc/loadavg)"
  nvidia-smi --query-gpu=name,utilization.gpu,memory.used,memory.total --format=csv,noheader 2>/dev/null || true
} > "$OUT/env.txt"

LAUNCH_PID=""
cleanup() {
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 30); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    kill -KILL "$LAUNCH_PID" 2>/dev/null || true
  fi
  # 런치가 남긴 Gazebo 서버 (이 컨테이너의 PID 공간 안에서만)
  for p in $(pgrep -f 'ign gazebo' || true); do [ "$p" != "$$" ] && kill -KILL "$p" 2>/dev/null || true; done
}
trap cleanup EXIT

NO_STACK=(with_localization:=false with_navigation:=false with_perception:=false with_behavior:=false
          with_fleet:=false with_dashboard:=false with_evaluation:=false lifecycle_watchdog_grace:=0)

wait_for_topic() {   # $1 토픽, $2 상한 [s]
  local end=$((SECONDS + $2))
  while [ $SECONDS -lt $end ]; do
    if ros2 topic list 2>/dev/null | grep -qx "$1"; then return 0; fi
    sleep 3
  done
  return 1
}

wait_for_clock_progress() {   # 일시 정지가 풀려 시뮬레이션 시각이 흐를 때까지
  local end=$((SECONDS + $1)) a b
  while [ $SECONDS -lt $end ]; do
    a=$(timeout 10 ros2 topic echo --once /clock 2>/dev/null | awk '/sec:/{print $2; exit}')
    sleep 3
    b=$(timeout 10 ros2 topic echo --once /clock 2>/dev/null | awk '/sec:/{print $2; exit}')
    if [ -n "$a" ] && [ -n "$b" ] && [ "$b" -gt "$a" ]; then return 0; fi
  done
  return 1
}

case "$MODE" in
  load)
    LABEL=${1:?라벨}; ROBOTS=${2:?robots_file}; DUR=${3:?측정 s}; shift 3
    case "$ROBOTS" in /*) ;; *) ROBOTS="$SHARE_BRINGUP/config/$ROBOTS" ;; esac
    NUM=$(python3 -c "import yaml,sys; print(len(yaml.safe_load(open(sys.argv[1]))['robots']))" "$ROBOTS")
    ros2 launch amr_bringup multi_robot.launch.py robots_file:="$ROBOTS" num_robots:="$NUM" \
      "${NO_STACK[@]}" > "$OUT/launch.log" 2>&1 &
    LAUNCH_PID=$!
    last=$(python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))['robots'][-1]['name'])" "$ROBOTS")
    wait_for_topic "/$last/scan" 600 || { echo "스폰 대기 초과" >&2; exit 3; }
    wait_for_clock_progress 300 || { echo "시뮬레이션 시각이 흐르지 않는다" >&2; exit 3; }
    python3 "$LIB/lidar3d_probe.py" --label "$LABEL" --duration "$DUR" --out "$OUT/probe.json" \
      --proc 'ign gazebo' --proc 'parameter_bridge' --proc 'cloud_preprocessor_node' \
      --proc 'image_bridge' "$@" 2>&1 | tee "$OUT/probe.log"
    ;;
  check)
    python3 "$LIB/make_static_world.py" "$SHARE_SIM/worlds/warehouse.sdf" "$OUT/world" | tee "$OUT/world.log"
    ros2 launch amr_bringup multi_robot.launch.py \
      robots_file:="$SHARE_BRINGUP/config/fleet_spawn_lidar3d.yaml" num_robots:=1 \
      world:="$OUT/world/warehouse.sdf" "${NO_STACK[@]}" > "$OUT/launch.log" 2>&1 &
    LAUNCH_PID=$!
    wait_for_topic "/amr_01/lidar3d/points" 600 || { echo "3D 토픽 대기 초과" >&2; exit 3; }
    wait_for_clock_progress 300 || { echo "시뮬레이션 시각이 흐르지 않는다" >&2; exit 3; }
    sleep 10
    python3 "$LIB/lidar3d_sensor_check.py" --robot amr_01 --world-sdf "$OUT/world/warehouse.sdf" \
      --models-dir "$SHARE_SIM/models" --out "$OUT/check.json" "$@" 2>&1 | tee "$OUT/check.log"
    ;;
  *) echo "모르는 mode: $MODE" >&2; exit 2 ;;
esac
echo "loadavg_end: $(cat /proc/loadavg)" >> "$OUT/env.txt"
