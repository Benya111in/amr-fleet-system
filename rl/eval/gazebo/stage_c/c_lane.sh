#!/usr/bin/env bash
# 단계 C 레인 드라이버 — 목록의 묶음을 차례로 돈다. 끝난 묶음(DONE)은 건너뛰고, 끊긴 묶음은 첫 빠진 exec_index 부터
# 새 시도로 잇는다 (재개 가능). RL·근접 인계 묶음은 문($R/gate/rl_ready.json, STOP_RL 없음)이 열려야 돈다 —
# 문이 닫혀 있으면 같은 목록의 다음 DWA 묶음을 먼저 돈다 (사전 등록 규칙). c_master.sh 가 부른다.
#   c_lane.sh <lane> <목록 파일: 'batch ctrl start count' 줄>
set -u
R=/home/CAPYI_AI/runs/bonus1/b1-gzc; C=$R/c2; D=$R/driver
LANE=$1; LIST=$2
mkdir -p "$R/locks" "$C/batches"
exec 8> "$R/locks/lane$LANE.lock"
flock -n 8 || { echo "$(date '+%F %T') lane $LANE 드라이버가 이미 돈다 — 끝" >> "$C/lane$LANE.log"; exit 0; }
log() { echo "$(date '+%F %T') [lane $LANE] $*" >> "$C/lane$LANE.log"; }
log "시작 pid=$$ list=$LIST load=$(cut -d' ' -f1-3 /proc/loadavg)"
while :; do
  if [ -e "$R/gate/STOP_ALL" ]; then log "STOP_ALL — 끝"; exit 0; fi
  pick=""; pending=0
  while read -r b ctrl start count; do
    st=$(python3 "$D/c_state.py" status "$C/batches/$b" "$start" "$count")
    case "$st" in DONE*|FAILED*) continue ;; esac
    pending=$((pending + 1))
    if [ "$ctrl" != dwa ]; then
      g=$(python3 "$D/c_state.py" gate "$R/gate")
      [ "${g%% *}" = READY ] || continue
    fi
    pick="$b $ctrl $start $count $st"
    break
  done < "$LIST"
  if [ -z "$pick" ]; then
    if [ "$pending" -eq 0 ]; then log "목록의 묶음 모두 끝 (DONE/FAILED)"; exit 0; fi
    sleep 60
    continue
  fi
  read -r b ctrl start count _ first rem streak <<< "$pick"
  if [ "$ctrl" = dwa ]; then
    pid=dwa; model=v12_s1.onnx; ts=""
  else
    read -r _ seed model _ <<< "$(python3 "$D/c_state.py" gate "$R/gate")"
    pid="B1_ppo10_s$seed"; ts=$seed
    [ "$ctrl" = rl_handover ] && pid="$pid+ho0.4"
  fi
  log "묶음 $b ($ctrl) exec_index $first.. 남은 $rem (진전 없는 연속 $streak) policy $pid"
  bash "$D/c_batch.sh" "$LANE" "$C/batches/$b" "$ctrl" all "$first" "$rem" "$pid" "$model" "$ts"
  rc=$?
  st=$(python3 "$D/c_state.py" status "$C/batches/$b" "$start" "$count")
  log "묶음 $b 시도 끝 rc=$rc → $st load=$(cut -d' ' -f1-3 /proc/loadavg)"
  case "$st" in
    FAILED*) echo "$(date '+%F %T') b1-gzc: 묶음 $b 실패 ($st) — 레인 $LANE 은 다음 묶음으로 간다" \
               >> /home/CAPYI_AI/logs/orch/ALERT_b1-gzc.txt ;;
  esac
  sleep 5
done
