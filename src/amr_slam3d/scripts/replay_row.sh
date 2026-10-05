#!/usr/bin/env bash
# 매핑 행 하나를 파일럿 bag 재생으로 돌린다 (보너스 2 5단계). 컨테이너 안에서 돈다:
#   rtabmap3d·rtabmap2d·slam_toolbox → amr-ai/slam 이미지,  liosam_loop·liosam_noloop → amr-ai/liosam 이미지.
# 작업공간(/ros2_ws)은 slam 이미지에서 빌드한 것을 마운트한다. liosam 이미지에서는 amr_slam3d 순수 모듈만 쓴다.
#
#   replay_row.sh <행> <bag 디렉터리> <출력 디렉터리> [재생 배속, 기본 0.5]
#
# 순서: anchor_tf(map → slam_map = 스폰 자세; LIO-SAM 은 + LiDAR 장착 자리) → 행 런치 → slam_probe →
#       ros2 bag play (행의 토픽만, /clock 은 --clock 200 으로 bag 시각에서 새로 낸다) → 마무리(지도 저장) →
#       probe 종료(probe.json·traj_online.tum·map_world.*) → 로그 판정(log_scan.json) → 점 지도 월드 변환.
# 환경: ROS_DOMAIN_ID·ROS_LOCALHOST_ONLY 는 호출자가 준다 (컨테이너마다 격리).
set -eo pipefail

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
EOF
)"

{
  echo "date: $(date -Iseconds) row=$ROW bag=$BAG rate=$RATE settle=$SETTLE"
  echo "GIT_HEAD=${GIT_HEAD:-} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-}"
  echo "launch: $LAUNCH"
  echo "play_topics: $PLAY_TOPICS"
  echo "spawn_pose: $POSE anchors: ${ANCHORS[*]}"
  echo "loadavg: $(cat /proc/loadavg)"
} > "$OUT/env.txt"

PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
  for p in "${PIDS[@]}"; do
    for _ in $(seq 1 20); do kill -0 "$p" 2>/dev/null || break; sleep 1; done
    kill -KILL "$p" 2>/dev/null || true
  done
}
trap cleanup EXIT

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

# LIO-SAM save_map 은 $HOME + destination 을 지우고 다시 만든다 → HOME 을 출력 디렉터리로
export HOME="$OUT"
# shellcheck disable=SC2086
ros2 launch $LAUNCH database_path:="$OUT/rtabmap.db" > "$OUT/row.log" 2>&1 &
PIDS+=($!)
python3 "$SCRIPTS/slam_probe.py" --row "$ROW" --out "$OUT" > "$OUT/probe.log" 2>&1 &
PROBE=$!
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
ros2 bag play "$BAG" --rate "$RATE" --clock 200 --disable-keyboard-controls \
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

case "$ROW" in
  rtabmap3d|rtabmap2d)
    if [ -f "$OUT/rtabmap.db" ]; then
      rtabmap-export --poses --poses_format 10 --opt 2 --output_dir "$OUT" --output rtabmap \
        "$OUT/rtabmap.db" > "$OUT/export_poses.log" 2>&1 || true
      rtabmap-export --cloud --scan --opt 2 --output_dir "$OUT" --output rtabmap \
        "$OUT/rtabmap.db" > "$OUT/export_cloud.log" 2>&1 || true
      rtabmap-export --params "$OUT/rtabmap.db" > "$OUT/db_params.ini" 2>&1 || true
      TF=$(anchor_of slam_map) || TF=''
      if [ -n "$TF" ] && [ -f "$OUT/rtabmap_cloud.ply" ]; then
        python3 "$SCRIPTS/bag_tools.py" cloud "$OUT/rtabmap_cloud.ply" "$OUT/cloud_world.ply" \
          --tf "$TF" > "$OUT/cloud_world.log" 2>&1 || true
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
