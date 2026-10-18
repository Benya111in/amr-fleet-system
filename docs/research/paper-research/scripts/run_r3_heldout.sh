#!/usr/bin/env bash
# 3 라운드 held-out D 전부 (prereg_r3 §4): chain_sim_x 먼저, 그다음 ROS 를 두 줄(레인 7 + 7 = 14, 도메인 70–83)로.
#   줄 A (레인 0–6) : c1 → c3
#   줄 B (레인 7–13): c2 → o
#   run_r3_heldout.sh <commit> <dirty> ["<줄 A>" "<줄 B>"] [skip-chain]
# 끝난 실행은 건너뛰므로 다시 부르면 이어서 돈다. 세션과 분리하려고 systemd-run 으로 부른다.
set -u
C=$1; D=$2
TA=${3:-"c1 c3"}; TB=${4:-"c2 o"}; SKIPCHAIN=${5:-}
H=$(cd "$(dirname "$0")" && pwd)
R=/home/CAPYI_AI/paper_research/runs/r3
J=/home/CAPYI_AI/paper_research/bin/jlog
PY=/root/miniconda3/bin/python3
export PYTHONDONTWRITEBYTECODE=1
if [ -z "$SKIPCHAIN" ]; then
  mkdir -p "$R/chain"
  $J pr-harness "R3 STAGE chain START run_dir=$R/chain (dwa_full·dwa_R·dwa_R2·dwa_R3·dwa_textbook_tuned × C1·C2·C3 대응, 24 프로세스, 하네스 $C)"
  $PY "$H/orchestrate_r3.py" chain --commit "$C" --dirty "$D" --procs 24 >> "$R/chain/orchestrate.log" 2>&1
  rc=$?
  $J pr-harness "R3 STAGE chain END run_dir=$R/chain n=$(cat "$R"/chain/raw/*.jsonl 2>/dev/null | wc -l) rc=$rc"
fi
run() {
  local st=$1 l0=$2
  mkdir -p "$R/$st"
  $J pr-harness "R3 STAGE $st START run_dir=$R/$st (레인 $l0–$((l0 + 6)), 하네스 $C)"
  $PY "$H/orchestrate_r3.py" ros "$st" --commit "$C" --dirty "$D" --lanes 7 --lane0 "$l0" \
    >> "$R/$st/orchestrate.log" 2>&1
  local rc=$?
  $J pr-harness "R3 STAGE $st END run_dir=$R/$st n=$(cat "$R/$st"/raw/*.jsonl 2>/dev/null | wc -l) rc=$rc"
}
( for st in $TA; do run "$st" 0; done ) &
A=$!
( for st in $TB; do run "$st" 7; done ) &
B=$!
wait $A; wait $B
echo done
