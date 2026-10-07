#!/usr/bin/env bash
# C4 동적 장애물 스트레스 (시나리오 08 왕복, 결과 전에 사전 등록) — 본 측정(c_master.sh)이 끝난 뒤
# `exclusive.sh b1-gzc` 안에서 DOMAIN 37 로: DWA 30 → RL(대표 정책) 30. 재개 가능 (끝난 팔은 건너뛴다).
#   nohup setsid bash c4_run.sh --after-main >> $R/c4/c4.log 2>&1 < /dev/null &
#     --after-main: c2/master_ended.txt 가 생길 때까지 기다린 뒤 exclusive.sh 로 자기 자신을 다시 부른다.
# loadavg 는 리다이렉트로만 읽는다 (특수 파일 경로를 프로그램 인자로 넘기지 않는다).
set -u
R=/home/CAPYI_AI/runs/bonus1/b1-gzc; C4=$R/c4; D=$R/driver; WT=/home/CAPYI_AI/wt/b1-gzc
mkdir -p "$C4/runs" "$R/locks"
lavg() { local a b c r; read -r a b c r < /proc/loadavg; echo "$a $b $c"; }
if [ "${1:-}" = --after-main ]; then
  exec 5> "$R/locks/c4_wait.lock"
  flock -n 5 || { echo "$(date '+%F %T') c4 대기자가 이미 있다"; exit 0; }
  echo "$(date '+%F %T') 본 측정 끝을 기다린다 (c2/master_ended.txt)"
  until [ -e "$R/c2/master_ended.txt" ]; do sleep 60; done
  echo "$(date '+%F %T') 본 측정 끝 — exclusive 로 C4"
  exec /home/CAPYI_AI/env/exclusive.sh b1-gzc bash "$D/c4_run.sh"
fi
exec 4> "$R/locks/c4.lock"
flock -n 4 || { echo "$(date '+%F %T') C4 가 이미 돈다"; exit 0; }
echo "$(date '+%F %T') C4 시작 EXCLUSIVE_HOLDER=${EXCLUSIVE_HOLDER:-없음} load=$(lavg)"
(
  exec 6> "$R/locks/c4_sampler.lock"
  flock -n 6 || exit 0
  while :; do
    g=$(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits | head -1 | tr -d ' ')
    names=$(docker ps --format '{{.Names}}' | tr '\n' ' ')
    nis=$(echo "$names" | tr ' ' '\n' | grep -c -E 'b1-(ppo10|sac|dr|sweep|binfra|ppo10-eval)-|isaac')
    printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$(date +%s)" "$(date '+%F %T')" "$(lavg | tr ' ' ',')" "$g" "$nis" "$names"
    sleep 10
  done
) >> "$C4/sampler.tsv" 2>&1 &
SPID=$!
SNAP=$(sed -n 1p "$R/ws/SNAPSHOT_COMMIT")
for arm in dwa rl; do
  BD=$C4/runs/$arm
  if [ -e "$BD/DONE" ]; then echo "$arm 끝 — 건너뜀"; continue; fi
  if [ "$arm" = dwa ]; then
    pid=dwa; model=v12_s1.onnx; ts=()
  else
    g=$(python3 "$D/c_state.py" gate "$R/gate")
    if [ "${g%% *}" != READY ]; then echo "RL 문 닫힘 ($g) — RL 팔 건너뜀"; continue; fi
    read -r _ seed model _ <<< "$g"
    pid="B1_ppo10_s$seed"; ts=(--train-seed "$seed")
  fi
  st=$(python3 "$D/c_state.py" status "$BD" 0 30)
  case "$st" in DONE*) continue ;; FAILED*) echo "$arm FAILED — 건너뜀"; continue ;; esac
  read -r _ first rem _ <<< "$st"
  n=1; while [ -e "$BD/a$n" ]; do n=$((n + 1)); done
  A=$BD/a$n; mkdir -p "$A"
  {
    echo "snapshot $(sed -n 2p "$R/ws/SNAPSHOT_COMMIT") (고정 작업공간, 읽기 전용)"
    echo "worktree_head $(git -C $WT rev-parse HEAD)"
    echo "worktree_status:"; git -C $WT status --porcelain
    echo "worktree_diff_stat:"; git -C $WT diff --stat
    echo "lane 37 ctrl $arm select all start $first count $rem policy $pid model $model"
    echo "model_sha256 $(sha256sum "$R/models/$model" | cut -c1-64)"
    echo "world_sha256 $(sha256sum "$R/ws/src/amr_simulation/worlds/warehouse.sdf" | cut -c1-64)"
    echo "scenarios_sha256 $(sha256sum "$C4/scenarios_c4_s08.json" | cut -c1-64)"
    echo "EXCLUSIVE_HOLDER=${EXCLUSIVE_HOLDER:-없음}"
    echo "containers: $(docker ps --format '{{.Names}}' | tr '\n' ' ')"
  } > "$A/HEAD.txt" 2>&1
  date -Iseconds > "$A/started.txt"
  echo "load_start $(lavg)" > "$A/host_load.txt"
  NAME=amr-ai-b1-gzc-c4
  if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
    docker stop -t 30 "$NAME" > /dev/null 2>&1; docker rm -f "$NAME" > /dev/null 2>&1
  fi
  timeout -s INT -k 120 7200 /home/CAPYI_AI/env/gz_slot.sh b1-gzc \
    docker run --rm --name "$NAME" --gpus all --shm-size=8g \
    -e ROS_DOMAIN_ID=37 -e IGN_PARTITION=b1gzc_c4 -e ROS_LOCALHOST_ONLY=1 \
    -e WORLD=/ros2_ws/src/amr_simulation/worlds/warehouse.sdf -e MODEL="/models/$model" -e MODE=B \
    -e COMMIT="$SNAP" -e HANDOVER_RADIUS=0.4 \
    -v "$R/ws":/ros2_ws:ro -v "$C4":/c4 -v "$R/models":/models:ro -w /ros2_ws \
    amr-ai/rl:0.3 bash src/amr_rl_bench/scripts/run_bench.sh "/c4/runs/$arm/a$n" "$arm" \
      --scenarios /c4/scenarios_c4_s08.json --run-tag "c4_${arm}_a$n" \
      --world-sdf /ros2_ws/src/amr_simulation/worlds/warehouse.sdf \
      --models-dir /ros2_ws/src/amr_simulation/models \
      --select all --start "$first" --count "$rem" --policy-id "$pid" "${ts[@]}" \
    > "$A/docker.log" 2>&1
  echo "rc=$?" >> "$A/docker.log"
  docker stop -t 20 "$NAME" > /dev/null 2>&1
  echo "load_end $(lavg)" >> "$A/host_load.txt"
  python3 "$D/c_state.py" attempt-end "$A" > /dev/null
  date -Iseconds > "$A/ended.txt"
  echo "$(date '+%F %T') $arm a$n → $(python3 "$D/c_state.py" status "$BD" 0 30)"
done
kill $SPID 2> /dev/null
date -Iseconds > "$C4/c4_ended.txt"
