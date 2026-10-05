#!/usr/bin/env bash
# 컨테이너 하나의 GPU 메모리를 주기적으로 기록한다 (호스트에서, 컨테이너가 끝나면 멈춘다).
#   rl/isaac/scripts/gpu_monitor.sh <컨테이너 이름> <출력 csv> [주기 s]
# 열: 시각, GPU 전체 사용 MiB, 이 컨테이너 프로세스 합 MiB, loadavg 1 분
set -u
NAME=$1
OUT=$2
PERIOD=${3:-5}
echo "time,gpu_used_MiB,container_MiB,loadavg1" > "$OUT"
for _ in $(seq 1 60); do docker ps --format '{{.Names}}' | grep -qx "$NAME" && break; sleep 2; done
while docker ps --format '{{.Names}}' | grep -qx "$NAME"; do
  PIDS=$(docker top "$NAME" -eo pid 2>/dev/null | tail -n +2 | tr '\n' ' ')
  USED=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
  MINE=0
  while IFS=, read -r pid mem; do
    pid=${pid// /}; mem=${mem// /}; mem=${mem%MiB}
    for p in $PIDS; do [ "$p" = "$pid" ] && MINE=$((MINE + mem)); done
  done < <(nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader,nounits)
  echo "$(date +%s),$USED,$MINE,$(cut -d' ' -f1 /proc/loadavg)" >> "$OUT"
  sleep "$PERIOD"
done
