#!/usr/bin/env bash
# R3-C 단계 구동기 (prereg_r3c.md §3·부록 A). systemd-run 으로 세션과 분리해 부른다. 관문은 이 스크립트가 기다린다.
#   run_r3c.sh equiv <commit>   ws_r3c 빌드 chain_sim_x 비트 동일 확인 (held-out A 60 × dwa_full·Z02 × 잡음 없음·seed 1)
#   run_r3c.sh e2 <commit>      E2 X00 × held-out D C1 60 (레인 14–19)
#   run_r3c.sh e5 <commit>      E5 P1·P3 사슬 탐침, P4 와인드업 gtest (ws_r3c 빌드)
#   run_r3c.sh e13 <commit>     GO_E13 표지(변형 sha256 커밋 뒤 만든다)를 기다려 E4 동치 → E1+E3 ROS (c1·c3·o 한 줄) → E1 chain
#   run_r3c.sh e4 <commit>      관문(R3 held-out 봉인·E1–E3 봉인·GO_E4 표지·다른 레인 컨테이너 없음·1 분 부하 < 3)을 기다려 E4 측정
set -u
CMD=$1; C=$2
H=$(cd "$(dirname "$0")" && pwd)
R=/home/CAPYI_AI/paper_research/runs/r3c
RW=/home/CAPYI_AI/paper_research/runs
J=/home/CAPYI_AI/paper_research/bin/jlog
SEAL=/home/CAPYI_AI/paper_research/bin/seal
LOAD=/home/CAPYI_AI/paper_research/bin/load
PY=/root/miniconda3/bin/python3
WS=/home/CAPYI_AI/paper_research/ws
W3=/home/CAPYI_AI/paper_research/ws_r3c
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$R"
nrun() { cat "$1"/raw/*.jsonl 2>/dev/null | wc -l; }
rate() { cat "$1"/raw/*.jsonl 2>/dev/null | $PY -c "
import sys, json
r=[json.loads(l) for l in sys.stdin if l.strip()]
print(f'{min(x[\"cmd_rate_hz\"] for x in r):.2f}' if r else 'nan', sum(1 for x in r if x['cmd_rate_hz']<18.0))"; }

ros() {   # ros <단계(쉼표)> [추가 인자…]
  local st=$1; shift
  local tag=${st//,/+}
  for s in ${st//,/ }; do mkdir -p "$R/$s"; done
  $J pr-r3c "R3-C STAGE $tag START run_dir=$R/{${st}} (하네스 $C)"
  $PY "$H/orchestrate_r3c.py" ros "$st" --commit "$C" --dirty 0 "$@" >> "$R/orchestrate_${tag}.log" 2>&1
  local rc=$?
  local msg=""
  for s in ${st//,/ }; do msg="$msg $s n=$(nrun "$R/$s") rate_min/<18Hz=$(rate "$R/$s" | sed 's# #/#')"; done
  $J pr-r3c "R3-C STAGE $tag END rc=$rc$msg"
  return $rc
}

case "$CMD" in
  equiv)
    D=$R/equiv_chain/$(date -u +%Y%m%dT%H%M%SZ)
    mkdir -p "$D/cfg"
    $J pr-r3c "R3-C STAGE equiv_chain START run_dir=$D (ws 빌드 eab6f8e 대 ws_r3c 빌드 $C, held-out A 60 × dwa_full·Z02 × 잡음 없음·seed 1)"
    $PY "$H/params_r3.py" "$D/cfg" > /dev/null
    for side in ws:$WS r3c:$W3; do
      lab=${side%%:*}; src=${side#*:}
      docker run --rm --name amr-ai-pr-chain-r3c --cpus 4 --memory 8g -v "$src":/ros2_ws:ro -v "$WS":/pr_ws:ro \
        -v /home/CAPYI_AI/runs/dwa-abl:/abl:ro -v "$RW":/runs amr-ai/rl:0.3 nice -n 10 python3 \
        /pr_ws/src/amr_nav_research/scripts/run_equiv_x.py run \
        --bin /ros2_ws/install/amr_nav_research/lib/amr_nav_research/chain_sim_x --label "$lab" \
        --out "${D/$RW//runs}" --cfg-dir "${D/$RW//runs}/cfg" --arm dwa_full=dwa_full.cfg --arm Z02=dwa_z_Z02.cfg \
        --paths /abl/paths --workers 4 --commit "$C" >> "$D/run.log" 2>&1
    done
    docker run --rm --name amr-ai-pr-chain-r3c --cpus 2 -v "$WS":/pr_ws:ro -v "$RW":/runs amr-ai/rl:0.3 python3 \
      /pr_ws/src/amr_nav_research/scripts/run_equiv_x.py compare --out "${D/$RW//runs}" --a ws --b r3c >> "$D/run.log" 2>&1
    rc=$?
    $SEAL "$D" > /dev/null
    sm=$($PY -c "import json,sys; s=json.load(open(sys.argv[1])); print('pass', s['pass'], '동일', s['totals'])" "$D/summary.json" 2>&1)
    $J pr-r3c "R3-C STAGE equiv_chain END run_dir=$D rc=$rc ($sm)"
    ;;
  e2)
    ros e2
    $SEAL "$R/e2" > /dev/null
    ;;
  e5)
    D=$R/e5
    mkdir -p "$D/p13"
    $J pr-r3c "R3-C STAGE e5 START run_dir=$D (P1·P3 probe_chain_r3c, P4 test_r3c_windup; ws_r3c 빌드 $C)"
    docker run --rm --name amr-ai-pr-chain-r3c --cpus 2 -v "$W3":/ros2_ws:ro -v "$RW":/runs amr-ai/rl:0.3 bash -c \
      'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
       /ros2_ws/install/amr_nav_research/lib/amr_nav_research/probe_chain_r3c /runs/r3c/e5/p13 && \
       /ros2_ws/build/amr_nav_research/test_r3c_windup' > "$D/e5.log" 2>&1
    rc=$?
    grep '^\[P4\]' "$D/e5.log" > "$D/p4.txt"
    $SEAL "$D" > /dev/null
    $J pr-r3c "R3-C STAGE e5 END run_dir=$D rc=$rc"
    ;;
  e13)
    while [ ! -e "$R/GO_E13" ]; do sleep 30; done
    C=$(cat "$R/GO_E13")
    ros e4eq --ws "$W3"
    $SEAL "$R/e4eq" > /dev/null
    ros c1,c3,o --expand
    for s in c1 c3 o; do $SEAL "$R/$s" > /dev/null; done
    mkdir -p "$R/chain"
    $J pr-r3c "R3-C STAGE chain START run_dir=$R/chain (dwa_R3 + 변형 × C1·C2·C3 대응, 8 프로세스, 빌드 ws eab6f8e, 하네스 $C)"
    $PY "$H/orchestrate_r3c.py" chain --commit "$C" --dirty 0 --procs 8 >> "$R/chain/orchestrate.log" 2>&1
    rc=$?
    $SEAL "$R/chain" > /dev/null
    $J pr-r3c "R3-C STAGE chain END run_dir=$R/chain n=$(cat "$R"/chain/raw/*.jsonl 2>/dev/null | wc -l) rc=$rc"
    touch "$R/E13_DONE"
    ;;
  e4)
    msg=0
    while true; do
      ok=1
      for s in c1 c2 c3 o; do [ -e "$RW/r3/$s/SEALED" ] || ok=0; done
      for s in c1 c3 o chain e4eq; do [ -e "$R/$s/SEALED" ] || ok=0; done
      [ -e "$R/E13_DONE" ] || ok=0
      [ -e "$R/GO_E4" ] || ok=0          # E4 동치 결과를 커밋한 뒤에 만드는 표지 (prereg_r3c §7)
      [ -z "$(docker ps --format '{{.Names}}' | grep '^amr-ai-pr-ros-')" ] || ok=0
      l1=$($LOAD | awk '{print $1}')
      awk -v l="$l1" 'BEGIN{exit !(l < 3.0)}' || ok=0
      [ $ok = 1 ] && break
      if [ $msg = 0 ] && [ -e "$R/E13_DONE" ]; then
        $J pr-r3c "R3-C E4 측정 대기: 관문 미충족 (부하 $l1, 다른 레인 $(docker ps --format '{{.Names}}' | grep -c '^amr-ai-pr-ros-'))"
        msg=1
      fi
      sleep 60
    done
    C=$(cat "$R/GO_E4")
    mkdir -p "$R/e4"
    ( while true; do echo "$(date '+%F %T') $($LOAD)"; sleep 60; done ) >> "$R/e4/load.log" 2>&1 &
    LP=$!
    ros e4 --ws "$W3" --e4 --no-guard --stagger 5
    kill $LP 2>/dev/null
    $SEAL "$R/e4" > /dev/null
    touch "$R/E4_DONE"
    ;;
  *) echo "알 수 없는 명령 $CMD" >&2; exit 2 ;;
esac
