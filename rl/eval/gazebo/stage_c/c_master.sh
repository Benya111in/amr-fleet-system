#!/usr/bin/env bash
# 단계 C 본 측정 주 드라이버 — 반드시 `exclusive.sh b1-gzc` 한 번 안에서 (gz·cpu 슬롯 독점, 레인 둘이 같은 독점을 쓴다).
#   nohup setsid /home/CAPYI_AI/env/exclusive.sh b1-gzc bash $R/driver/c_master.sh >> $R/c2/master.log 2>&1 < /dev/null &
# 1) 부하 표집기(10 s: loadavg·GPU 사용률·메모리·컨테이너) 2) 파일럿(사전 등록 — smoke 20 을 레인 둘에서 DWA 로 함께,
# 셈에 넣지 않음; 레인별 RTF 중앙 < 0.75 면 레인 하나) 3) 레인 드라이버들 → 끝날 때까지 기다림.
# 다시 띄우면 끝난 단계·묶음은 건너뛴다. 같은 드라이버 두 개는 잠금이 막는다.
set -u
R=/home/CAPYI_AI/runs/bonus1/b1-gzc; C=$R/c2; D=$R/driver
mkdir -p "$C/pilot" "$C/batches" "$R/locks" "$R/gate"
exec 7> "$R/locks/master.lock"
flock -n 7 || { echo "$(date '+%F %T') master 가 이미 돈다 — 끝"; exit 0; }
log() { echo "$(date '+%F %T') [master] $*" | tee -a "$C/master_events.log"; }
log "시작 pid=$$ EXCLUSIVE_HOLDER=${EXCLUSIVE_HOLDER:-없음} load=$(cut -d' ' -f1-3 /proc/loadavg)"
(
  exec 6> "$R/locks/sampler.lock"
  flock -n 6 || exit 0
  while :; do
    g=$(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits | head -1 | tr -d ' ')
    names=$(docker ps --format '{{.Names}}' | tr '\n' ' ')
    nis=$(echo "$names" | tr ' ' '\n' | grep -c -E 'b1-(ppo10|sac|dr|sweep|binfra|ppo10-eval)-|isaac')
    printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$(date +%s)" "$(date '+%F %T')" \
      "$(cut -d' ' -f1-3 /proc/loadavg | tr ' ' ',')" "$g" "$nis" "$names"
    sleep 10
  done
) >> "$C/sampler.tsv" 2>&1 &
SPID=$!
if [ ! -e "$C/pilot/DECISION.json" ]; then
  log "파일럿: smoke 20 × 레인 35·36 (DWA, 셈에 넣지 않음)"
  PP=()
  for L in 35 36; do
    bash "$D/c_batch.sh" "$L" "$C/pilot/l$L" dwa smoke 0 20 dwa v12_s1.onnx &
    PP+=($!)
  done
  for p in "${PP[@]}"; do wait "$p"; done
  log "파일럿 결정: $(python3 "$D/c_state.py" pilot-decide "$C/pilot")"
fi
lanes=$(python3 -c "import json;print(' '.join(map(str, json.load(open('$C/pilot/DECISION.json'))['lanes'])))")
if [ -z "$lanes" ]; then
  log "파일럿이 멈춤 조건 — 레인을 띄우지 않는다 (DECISION.json 의 stop)"
  echo "$(date '+%F %T') b1-gzc: 파일럿 멈춤 조건 — $C/pilot/DECISION.json" >> /home/CAPYI_AI/logs/orch/ALERT_b1-gzc.txt
  kill $SPID 2> /dev/null; exit 3
fi
python3 "$D/c_state.py" lists "$D/schedule_c.tsv" "$lanes" "$C" > /dev/null
log "레인 $lanes 띄움"
LP=()
for L in $lanes; do
  bash "$D/c_lane.sh" "$L" "$C/lane$L.list" &
  LP+=($!)
done
for p in "${LP[@]}"; do wait "$p"; done
log "레인 모두 끝 load=$(cut -d' ' -f1-3 /proc/loadavg)"
kill $SPID 2> /dev/null
date -Iseconds > "$C/master_ended.txt"
