#!/usr/bin/env bash
# 단계 C 묶음 하나를 새 시도 디렉터리(a1, a2, …)로 돈다 — Gazebo 세션 하나. c_lane.sh·c_master.sh 가 부른다
# (exclusive.sh b1-gzc 안이라 gz_slot.sh 는 통과한다).
#   c_batch.sh <lane> <묶음 디렉터리> <dwa|rl|rl_handover> <select> <start> <count> <policy_id> <모델 파일> [train_seed]
# 측정 코드는 고정 사본 $R/ws (git archive, Release 빌드)를 읽기 전용으로 마운트한다 — 작업 트리를 고쳐도 측정은
# 바뀌지 않는다. 시도마다 HEAD.txt 에 사본 커밋·작업 트리 HEAD·status·diff --stat·런타임 경로 차이를 남긴다.
set -u
R=/home/CAPYI_AI/runs/bonus1/b1-gzc; C=$R/c2; WT=/home/CAPYI_AI/wt/b1-gzc
LANE=$1; BD=$2; CTRL=$3; SEL=$4; START=$5; COUNT=$6; PID=$7; MODELF=$8; TS=${9:-}
mkdir -p "$BD"
n=1; while [ -e "$BD/a$n" ]; do n=$((n + 1)); done
A=$BD/a$n; mkdir -p "$A"
NAME=amr-ai-b1-gzc-l$LANE
SNAP=$(sed -n 1p "$R/ws/SNAPSHOT_COMMIT")
{
  echo "snapshot $(sed -n 2p "$R/ws/SNAPSHOT_COMMIT") (고정 작업공간 $R/ws, 읽기 전용 마운트)"
  echo "worktree_head $(git -C $WT rev-parse HEAD)"
  echo "worktree_status:"; git -C $WT status --porcelain
  echo "worktree_diff_stat:"; git -C $WT diff --stat
  echo "runtime_diff_vs_snapshot (src·scenarios_v1 — 비어야 한다; 측정은 사본으로 돈다):"
  git -C $WT diff --stat "$SNAP" HEAD -- src rl/eval/gazebo/scenarios_v1.json
  echo "lane $LANE ctrl $CTRL select $SEL start $START count $COUNT policy $PID model $MODELF train_seed $TS"
  echo "model_sha256 $(sha256sum "$R/models/$MODELF" | cut -c1-64)"
  echo "world_sha256 $(sha256sum "$R/world/static/warehouse.sdf" | cut -c1-64)"
  echo "scenarios_sha256 $(sha256sum "$R/ws/rl/eval/gazebo/scenarios_v1.json" | cut -c1-64)"
  echo "EXCLUSIVE_HOLDER=${EXCLUSIVE_HOLDER:-없음}"
  nvidia-smi --query-gpu=memory.used,memory.free,utilization.gpu --format=csv,noheader
  echo "containers: $(docker ps --format '{{.Names}}' | tr '\n' ' ')"
} > "$A/HEAD.txt" 2>&1
date -Iseconds > "$A/started.txt"
echo "load_start $(cat /proc/loadavg)" > "$A/host_load.txt"
# 이 레인 이름의 컨테이너가 남아 있으면 (레인 잠금 아래이므로 이 과제의 것) 멈춘다
if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  docker stop -t 30 "$NAME" > /dev/null 2>&1; docker rm -f "$NAME" > /dev/null 2>&1
  echo "남은 컨테이너 $NAME 정리 $(date '+%T')" >> "$A/host_load.txt"
fi
HO=(); [ "$CTRL" = rl_handover ] && HO=(--handover-radius 0.4)
TSA=(); [ -n "$TS" ] && TSA=(--train-seed "$TS")
REL=${A#"$C"/}
timeout -s INT -k 120 "${BATCH_TMAX:-10800}" /home/CAPYI_AI/env/gz_slot.sh b1-gzc \
  docker run --rm --name "$NAME" --gpus all --shm-size=8g \
  -e ROS_DOMAIN_ID="$LANE" -e IGN_PARTITION="b1gzc_l$LANE" -e ROS_LOCALHOST_ONLY=1 \
  -e WORLD=/w/warehouse.sdf -e MODEL="/models/$MODELF" -e MODE=B -e COMMIT="$SNAP" -e HANDOVER_RADIUS=0.4 \
  -v "$R/ws":/ros2_ws:ro -v "$C":/runs -v "$R/world/static":/w:ro -v "$R/models":/models:ro -w /ros2_ws \
  amr-ai/rl:0.3 bash src/amr_rl_bench/scripts/run_bench.sh "/runs/$REL" "$CTRL" \
    --scenarios /ros2_ws/rl/eval/gazebo/scenarios_v1.json --run-tag "$(basename "$BD")_a$n" \
    --world-sdf /ros2_ws/src/amr_simulation/worlds/warehouse.sdf \
    --models-dir /ros2_ws/src/amr_simulation/models \
    --select "$SEL" --start "$START" --count "$COUNT" --policy-id "$PID" "${TSA[@]}" "${HO[@]}" \
  > "$A/docker.log" 2>&1
rc=$?
echo "rc=$rc" >> "$A/docker.log"
docker stop -t 20 "$NAME" > /dev/null 2>&1
echo "load_end $(cat /proc/loadavg)" >> "$A/host_load.txt"
python3 "$R/driver/c_state.py" attempt-end "$A" > /dev/null
date -Iseconds > "$A/ended.txt"
exit $rc
