#!/usr/bin/env bash
# 보너스 4 P5b 측정 띄우기 (b4-p5b — 노트 「P5b 사전 등록」). 재개 가능: 드라이버가 끝난 짝(.done)을 건너뛴다.
#   bash p5b_launch.sh            : b2-rerec 녹화 끝(RECORDING_DONE) 또는 <원자료>/GO 를 기다린 뒤
#                                   CPU_SLOTS=0 exclusive.sh b4-p5b 로 gz 슬롯 둘을 쥐고 아래 --inside 를 부른다.
#   bash p5b_launch.sh --inside   : (exclusive 안) 새 짝 마감을 정하고(처음 한 번 — DEADLINE.txt, 다시 띄워도 그대로) 드라이버.
# 같은 드라이버 둘을 막는다: <원자료>/launch.lock (flock). 멈추려면 touch <원자료>/STOP (다음 짝 전에 멈춤).
set -u
RAW=${P5B_RAW:-/home/CAPYI_AI/runs/b24/b4-p5b/raw}
B=$(dirname "$RAW")
REPO=${P5B_REPO:-/home/CAPYI_AI/wt/b4-p5b}
HERE=$(cd "$(dirname "$0")" && pwd)
WAIT_FILE=${P5B_WAIT_FILE:-/home/CAPYI_AI/runs/b24/b2-rerec/RECORDING_DONE}
ORDER=${P5B_ORDER:-SC2,C3,C1,F5L}
HOURS=${P5B_DEADLINE_HOURS:-40}
mkdir -p "$RAW"
log() { echo "$(date -Iseconds) $*" >> "$RAW/launch.log"; echo "$(date -Iseconds) $*"; }

if [ "${1:-}" = --inside ]; then
    if [ ! -s "$RAW/DEADLINE.txt" ]; then
        date -Iseconds -d "+${HOURS} hours" > "$RAW/DEADLINE.txt"
        log "새 짝 마감 = 드라이버 첫 시작 + ${HOURS} h = $(cat "$RAW/DEADLINE.txt") (사전 등록 줄이는 규칙)"
    fi
    DL=$(cat "$RAW/DEADLINE.txt")
    log "드라이버 시작 EXCLUSIVE_HOLDER=${EXCLUSIVE_HOLDER:-} 순서 $ORDER 마감 $DL $(uptime)"
    exec python3 "$HERE/p5_driver.py" --raw "$RAW" --ws-new "$B/ws_new" --ws-old "$B/ws_old" \
        --repo "$REPO" --order "$ORDER" --t-start 60 --no-new-pair-after "$DL" \
        --agent b4-p5b --run-prefix P5b --stack-mem 28g --server-mem 6g --mem-min-gb 32
fi

exec 9> "$RAW/launch.lock"
if ! flock -n 9; then
    log "이미 다른 p5b_launch 가 돈다 (launch.lock) — 끝"
    exit 3
fi
log "시작 pid $$ — 기다림: $WAIT_FILE 또는 $RAW/GO"
while [ ! -e "$WAIT_FILE" ] && [ ! -e "$RAW/GO" ]; do
    [ -e "$RAW/STOP" ] && { log "STOP — 기다리다 멈춤"; exit 1; }
    sleep 60
done
log "기다림 끝: $(ls -l --time-style=full-iso "$WAIT_FILE" "$RAW/GO" 2>/dev/null | awk '{print $6, $7, $9}' | tr '\n' ' ')"
CPU_SLOTS=0 /home/CAPYI_AI/env/exclusive.sh b4-p5b bash "$HERE/p5b_launch.sh" --inside
rc=$?
log "exclusive 끝 rc=$rc"
exit $rc
