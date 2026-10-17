#!/usr/bin/env bash
# held-out B ROS 단계 전부 (prereg §5 + §8): 레인 14 개(도메인 70–83)를 두 줄로 나눠 동시에 돈다.
#   줄 A (레인 0–6) : c1 → c3 → o → m
#   줄 B (레인 7–13): c2 → c4 → m_clr
# 각 단계는 orchestrate_x.py ros <단계> (끝난 실행은 건너뛰므로 다시 부르면 이어서 돈다).
#   run_heldout_x.sh <commit> <dirty>
set -u
C=$1; D=$2
H=$(cd "$(dirname "$0")" && pwd)
R=/home/CAPYI_AI/paper_research/runs
J=/home/CAPYI_AI/paper_research/bin/jlog
run() {   # run <단계> <레인 시작>
  local st=$1 l0=$2
  mkdir -p "$R/$st"
  $J pr-harness "STAGE $st START run_dir=$R/$st (레인 $l0–$((l0 + 6)), 하네스 $C)"
  PYTHONDONTWRITEBYTECODE=1 python3 "$H/orchestrate_x.py" ros "$st" --commit "$C" --dirty "$D" \
    --lanes 7 --lane0 "$l0" >> "$R/$st/orchestrate.log" 2>&1
  local rc=$?
  local n; n=$(cat "$R/$st"/raw/*.jsonl 2>/dev/null | wc -l)
  $J pr-harness "STAGE $st END run_dir=$R/$st n=$n rc=$rc"
}
( run c1 0; run c3 0; run o 0; run m 0 ) &
A=$!
( run c2 7; run c4 7; run m_clr 7 ) &
B=$!
wait $A; wait $B
echo done
