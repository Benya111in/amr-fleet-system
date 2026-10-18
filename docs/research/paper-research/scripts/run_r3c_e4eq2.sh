#!/usr/bin/env bash
# R3-C E4 동치 보충 (MPPI 순서 엇갈림) 구동기. E1–E3·E1 chain 이 끝난(E13_DONE) 뒤 레인 14–19 에서 돈다.
#   run_r3c_e4eq2.sh <commit>
# run_r3c.sh 는 대기 중인 구동기가 읽고 있어 고치지 않고 따로 둔다.
set -u
C=$1
H=$(cd "$(dirname "$0")" && pwd)
R=/home/CAPYI_AI/paper_research/runs/r3c
J=/home/CAPYI_AI/paper_research/bin/jlog
PY=/root/miniconda3/bin/python3
export PYTHONDONTWRITEBYTECODE=1
while [ ! -e "$R/E13_DONE" ]; do sleep 30; done
mkdir -p "$R/e4eq2"
$J pr-r3c "R3-C STAGE e4eq2 START run_dir=$R/e4eq2 (MPPI 선택 팔 감쌈·안 감쌈 구성·실행 순서 A/B × 반복 3 × 튜닝-20 매끈 4, ws_r3c, 하네스 $C)"
$PY "$H/orchestrate_r3c.py" ros e4eq2 --commit "$C" --dirty 0 --ws /home/CAPYI_AI/paper_research/ws_r3c \
  >> "$R/orchestrate_e4eq2.log" 2>&1
rc=$?
n=$(cat "$R"/e4eq2/raw/*.jsonl 2>/dev/null | wc -l)
/home/CAPYI_AI/paper_research/bin/seal "$R/e4eq2" > /dev/null
$J pr-r3c "R3-C STAGE e4eq2 END run_dir=$R/e4eq2 n=$n rc=$rc"
touch "$R/E4EQ2_DONE"
