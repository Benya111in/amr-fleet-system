#!/usr/bin/env bash
# 매핑 행 하나를 파일럿 bag 재생으로 돌린다 (보너스 2 5단계). 컨테이너 안에서 돈다:
#   rtabmap3d·rtabmap2d·slam_toolbox → amr-ai/slam 이미지,  liosam_loop·liosam_noloop → amr-ai/liosam 이미지.
# 작업공간(/ros2_ws)은 slam 이미지에서 빌드한 것을 마운트한다. liosam 이미지에서는 amr_slam3d 순수 모듈만 쓴다.
#
#   replay_row.sh <행> <bag 디렉터리> <출력 디렉터리> [재생 배속, 기본 0.5]
#
# 순서: anchor_tf(map → slam_map = 스폰 자세; LIO-SAM 은 + LiDAR 장착 자리) → 행 런치 → slam_probe →
#       ros2 bag play (행의 토픽만 — 기록된 /clock 포함, --clock 은 쓰지 않는다: rows.py 머리말) → 마무리(지도 저장) →
#       probe 종료(probe.json·traj_online.tum·map_world.*) → 로그 판정(log_scan.json) → 처리 누락 판정
#       (match_*.json: 입력 bag ↔ out_bag, C++ 기록기) → 최적화 궤적·점 지도를 월드로 (traj_final.tum, cloud_world.ply).
# 환경: ROS_DOMAIN_ID·ROS_LOCALHOST_ONLY 는 호출자가 준다 (컨테이너마다 격리). 컨테이너 /dev/shm 은 2 GiB 이상
# (docker --shm-size) — 아래 Fast DDS 프로파일이 참가자마다 16 MB SHM 세그먼트를 잡는다.
set -eo pipefail

# 큰 점군(0.8 MB)을 SHM 한 조각으로 보낸다. 기본 SHM 세그먼트(0.5 MB)에 안 들어가면 UDP 조각 전송으로 떨어지고,
# best-effort 구독(LIO-SAM 점군 구독 = best-effort 깊이 5)은 조각 하나만 잃어도 표본을 버린다 [측정: liosam_noloop
# v2 재생 — 1 788 장 중 imageProjection 출력 523 장]. 세그먼트 크기는 쓰는 쪽(bag play)이 정하므로 컨테이너 전체에
# 준다 — 통합 런치가 쓰는 것과 같은 파일 (amr_bringup config, perception.md §8.2 의 영상 실측과 같은 원인).
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/ros2_ws/src/amr_bringup/config/fastdds_multi_robot_localhost.xml}

ROW=${1:?행 (rtabmap3d | rtabmap2d | slam_toolbox | liosam_loop | liosam_noloop)}
BAG=${2:?bag 디렉터리}
OUT=${3:?출력 디렉터리}
RATE=${4:-0.5}
SETTLE=${SETTLE:-15}                 # [s, 벽시계] 재생 끝 뒤 처리 꼬리를 기다리는 시간
mkdir -p "$OUT"

# shellcheck disable=SC1091
[ -n "${ROS_DISTRO:-}" ] || source /opt/ros/humble/setup.bash
if [ -f /opt/lio_sam_ws/install/setup.bash ]; then
  # liosam 이미지: 우리 작업공간 C++ 산출물은 slam 이미지 라이브러리로 빌드됐으니 소싱하지 않고 순수 모듈만 쓴다
  export PYTHONPATH=/ros2_ws/src/amr_slam3d:${PYTHONPATH:-}
  # shellcheck disable=SC1091
  source /opt/lio_sam_ws/install/setup.bash
  SCRIPTS=/ros2_ws/src/amr_slam3d/scripts
else
  # shellcheck disable=SC1091
  source /ros2_ws/install/setup.bash
  SCRIPTS=$(ros2 pkg prefix amr_slam3d)/lib/amr_slam3d
fi
SPAWN_YAML=/ros2_ws/src/amr_bringup/config/fleet_spawn_lidar3d.yaml

eval "$(python3 - "$ROW" "$SPAWN_YAML" <<'EOF'
import shlex, sys, yaml
from amr_slam3d import rows
r = rows.row(sys.argv[1])
spawn = [x for x in yaml.safe_load(open(sys.argv[2]))['robots'] if x['name'] == rows.ROBOT][0]
print(f"LAUNCH={shlex.quote(r.launch)}")
print(f"PLAY_TOPICS={shlex.quote(' '.join(r.play_topics))}")
print(f"POSE={spawn['x']!r},{spawn['y']!r},{spawn['yaw']!r}")
print('ANCHORS=(' + ' '.join(shlex.quote(f'{p}|{c}|{v}') for p, c, v in r.anchors) + ')')
print(f"INPUT={shlex.quote(r.input_topic)}")
print(f"PROC_TOPICS={shlex.quote(' '.join(t for t, _, _ in r.processed))}")
print('PRE_LAUNCH=(' + ' '.join(shlex.quote(p) for p in r.pre_launch) + ')')
print(f"TAIL_EXEMPT={int(r.tail_exempt)}")
print(f"ROW_BAG={shlex.quote(r.bag)} ORACLE={int(r.oracle)}")
EOF
)"

# 행이 틀 토픽이 bag 에 다 있는지 재생 전에 본다 (파생 bag 토픽을 원 bag 으로 돌리는 실수 차단)
python3 - "$ROW" "$BAG/metadata.yaml" <<'EOF' || exit 3
import sys
from amr_slam3d import rows
r = rows.row(sys.argv[1])
miss = rows.missing_play_topics(r, rows.bag_topics(open(sys.argv[2], encoding='utf-8').read()))
if miss:
    sys.exit(f'bag 에 없는 재생 토픽 {miss} — 행 {r.name} 은 bag 종류 {r.bag!r} 가 필요하다')
EOF

{
  echo "date: $(date -Iseconds) row=$ROW bag=$BAG rate=$RATE settle=$SETTLE"
  echo "GIT_HEAD=${GIT_HEAD:-} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-}"
  echo "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE shm=$(df -h /dev/shm | tail -1)"
  echo "launch: $LAUNCH"
  echo "play_topics: $PLAY_TOPICS"
  echo "spawn_pose: $POSE anchors: ${ANCHORS[*]} pre_launch: ${PRE_LAUNCH[*]}"
  echo "row_bag: $ROW_BAG oracle: $ORACLE tail_exempt: $TAIL_EXEMPT image: ${IMAGE:-} replay_id: ${REPLAY_ID:-}"
  echo "loadavg: $(cat /proc/loadavg)"
} > "$OUT/env.txt"

PIDS=()
LAUNCH_PID=""
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
  for p in "${PIDS[@]}"; do
    # 행 런치는 RTAB-Map 이 DB 에 최적화 그래프를 쓰는 동안 기다린다 (60 s), 나머지는 20 s
    lim=20; [ "$p" = "$LAUNCH_PID" ] && lim=60
    n=0
    while [ "$n" -lt "$lim" ] && kill -0 "$p" 2>/dev/null; do sleep 1; n=$((n + 1)); done
    if kill -0 "$p" 2>/dev/null; then
      kill -KILL "$p" 2>/dev/null || true
      [ "$p" = "$LAUNCH_PID" ] && echo "launch_exit: SIGKILL after ${lim}s" >> "$OUT/env.txt"
    elif [ "$p" = "$LAUNCH_PID" ]; then
      echo "launch_exit: clean after ${n}s" >> "$OUT/env.txt"
    fi
  done
}
trap cleanup EXIT

# 비대화형 bash 의 `&` 작업은 SIGINT 를 무시(SIG_IGN)한 채 시작하고 그 처분이 exec 로 자식에게 이어진다 — s3 재생에서
# 행 런치 6 개가 SIGINT 에 반응하지 않고 SIGKILL 됐다 (row.log 에 SIGINT 처리 줄 없음, host_*.log "Killed").
# [측정 b2-offline tmp/sigtest.sh] `&` 로 띄운 ros2 launch 의 /proc/<pid>/status SigIgn 에 SIGINT 비트가 있다.
# 처분을 기본으로 되돌린 뒤 exec 한다. **셸 함수로 감싸면 안 된다** — 함수를 `&` 로 부르면 서브셸이 생겨 $! 가 런치가
# 아니라 그 서브셸을 가리키고 SIGINT 가 런치에 닿지 않는다 (첫 고침이 그래서 실패: launch_exit "SIGKILL after 60s").
# 배열을 단순 명령으로 실행하면 bash 가 fork 한 자식이 곧바로 python3 → ros2 로 exec 되어 $! = 런치 PID 다
# [측정 tmp/sigtest2.sh: SIGINT 1 s 뒤 "user interrupted with ctrl-c", 두 노드 "finished cleanly"].
SIGDFL=(python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])')

for a in "${ANCHORS[@]}"; do
  IFS='|' read -r parent child via <<< "$a"
  args=(--parent "$parent" --child "$child")
  if [ -z "$via" ]; then
    args+=(--pose "$POSE")
  else
    args+=(--pose 0,0,0 --via "${via%%>*}" "${via##*>}")
  fi
  python3 "$SCRIPTS/anchor_tf.py" "${args[@]}" > "$OUT/anchor_${child//\//_}.log" 2>&1 &
  PIDS+=($!)
done

i=0
for pl in "${PRE_LAUNCH[@]}"; do
  # shellcheck disable=SC2086
  "${SIGDFL[@]}" ros2 launch $pl > "$OUT/pre_launch_$i.log" 2>&1 &
  PIDS+=($!)
  i=$((i + 1))
done

# LIO-SAM save_map 은 $HOME + destination 을 지우고 다시 만든다 → HOME 을 출력 디렉터리로
export HOME="$OUT"
# shellcheck disable=SC2086
"${SIGDFL[@]}" ros2 launch $LAUNCH database_path:="$OUT/rtabmap.db" > "$OUT/row.log" 2>&1 &
LAUNCH_PID=$!
PIDS+=("$LAUNCH_PID")
python3 "$SCRIPTS/slam_probe.py" --row "$ROW" --out "$OUT" > "$OUT/probe.log" 2>&1 &
PROBE=$!
REC=""
if [ -n "$PROC_TOPICS" ]; then
  # 처리 출력의 권위 있는 개수 — C++ 기록기 (Python 측정 노드는 부하에 뒤처질 수 있다)
  # shellcheck disable=SC2086
  ros2 bag record -s mcap -o "$OUT/out_bag" $PROC_TOPICS /tf > "$OUT/out_record.log" 2>&1 &
  REC=$!
  PIDS+=("$REC")
fi
sleep 15                              # 노드 기동 (벽시계)

QOS="$OUT/qos_overrides.yaml"
cat > "$QOS" <<'EOF'
/tf_static:
  history: keep_all
  reliability: reliable
  durability: transient_local
EOF
echo "play_start_wall: $(date -Iseconds) loadavg: $(cat /proc/loadavg)" >> "$OUT/env.txt"
# shellcheck disable=SC2086
ros2 bag play "$BAG" --rate "$RATE" --disable-keyboard-controls \
  --qos-profile-overrides-path "$QOS" --topics $PLAY_TOPICS > "$OUT/play.log" 2>&1
echo "play_end_wall: $(date -Iseconds) loadavg: $(cat /proc/loadavg)" >> "$OUT/env.txt"
sleep "$SETTLE"

case "$ROW" in
  liosam_*)
    timeout 120 ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
      "{resolution: 0.0, destination: /liosam_map}" > "$OUT/save_map.log" 2>&1 || true
    ;;
esac

kill -INT "$PROBE" 2>/dev/null || true
for _ in $(seq 1 60); do kill -0 "$PROBE" 2>/dev/null || break; sleep 1; done
if [ -n "$REC" ]; then
  kill -INT "$REC" 2>/dev/null || true
  for _ in $(seq 1 60); do kill -0 "$REC" 2>/dev/null || break; sleep 1; done
fi
cleanup
trap - EXIT

# ---- 마무리: 로그 판정, 행별 산출물 ----
python3 - "$ROW" "$OUT" <<'EOF' > "$OUT/log_scan.json"
import json, sys
from pathlib import Path
from amr_slam3d import rows
r = rows.row(sys.argv[1])
text = (Path(sys.argv[2]) / 'row.log').read_text(errors='replace')
print(json.dumps(rows.scan_log(text, r), ensure_ascii=False, indent=1))
EOF

anchor_of() {   # probe.json 의 world ← 프레임 변환 → x,y,z,qx,qy,qz,qw
  python3 -c "import json,sys; a=json.load(open(sys.argv[1]))['anchors'][sys.argv[2]]; \
print(','.join(repr(float(v)) for v in a))" "$OUT/probe.json" "$1"
}

for t in $PROC_TOPICS; do
  python3 "$SCRIPTS/bag_tools.py" match --in-bag "$BAG" --in-topic "$INPUT" --out-bag "$OUT/out_bag" \
    --out-topic "$t" --out "$OUT/match_$(echo "$t" | tr '/' '_' | sed 's/^_//').json" \
    --tail-exempt "$TAIL_EXEMPT" \
    >> "$OUT/match.log" 2>&1 || true
done

case "$ROW" in
  rtabmap*)
    if [ -f "$OUT/rtabmap.db" ]; then
      rtabmap-export --poses --poses_format 10 --opt 2 --output_dir "$OUT" --output rtabmap \
        "$OUT/rtabmap.db" > "$OUT/export_poses.log" 2>&1 || true
      rtabmap-info "$OUT/rtabmap.db" > "$OUT/db_info.txt" 2>&1 || true
      TF=$(anchor_of slam_map) || TF=''
      if [ -n "$TF" ] && [ -f "$OUT/rtabmap_poses.txt" ]; then
        python3 "$SCRIPTS/bag_tools.py" poses "$OUT/rtabmap_poses.txt" "$OUT/traj_final.tum" \
          --tf "$TF" > "$OUT/traj_final.log" 2>&1 || true
        if [ "${ROW#rtabmap3d}" != "$ROW" ]; then
          python3 "$SCRIPTS/bag_tools.py" assemble "$BAG" --topic "$INPUT" \
            --poses "$OUT/traj_final.tum" --out "$OUT/cloud_world.ply" > "$OUT/cloud_world.log" 2>&1 || true
        fi
      fi
    fi
    ;;
  liosam_*)
    TF=$(anchor_of amr_01/liosam_odom) || TF=''
    if [ -n "$TF" ] && [ -f "$OUT/liosam_map/GlobalMap.pcd" ]; then
      python3 "$SCRIPTS/bag_tools.py" cloud "$OUT/liosam_map/GlobalMap.pcd" \
        "$OUT/cloud_world.ply" --tf "$TF" > "$OUT/cloud_world.log" 2>&1 || true
    fi
    ;;
esac
echo "loadavg_end: $(cat /proc/loadavg)" >> "$OUT/env.txt"
