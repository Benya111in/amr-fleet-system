#!/usr/bin/env bash
# held-out B ROS 단계 전부 (prereg §5 + §8): 레인 14 개(도메인 70–83)를 두 줄로 나눠 동시에 돈다.
#   줄 A (레인 0–6) : c1 → c3 → o → m
#   줄 B (레인 7–13): c2 → c4 → m_clr
# 각 단계는 orchestrate_x.py ros <단계> (끝난 실행은 건너뛰므로 다시 부르면 이어서 돈다).
#   run_heldout_x.sh <commit> <dirty> ["<줄 A 단계들>" "<줄 B 단계들>"]
# 줄을 직접 주면 그 단계들만 돈다 (중단 뒤 이어 돌기·줄 균형 맞추기). 세션이 끝나도 살아 있게
# setsid -w 로 부르는 것을 권한다.
set -u
C=$1; D=$2
TA=${3:-"c1 c3 o m"}; TB=${4:-"c2 c4 m_clr"}
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
( for st in $TA; do run "$st" 0; done ) &
A=$!
( for st in $TB; do run "$st" 7; done ) &
B=$!
wait $A; wait $B
echo done
