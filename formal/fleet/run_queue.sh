#!/usr/bin/env bash
# 여러 TLC 설정을 차례로 돌린다 (한 번에 하나 -- 공유 CPU 보호).
# 사용법: formal/fleet/run_queue.sh <모듈>:<cfg> [<모듈>:<cfg> ...]
#   예) formal/fleet/run_queue.sh QuorumVote:cfg/QV_N3_Z1_R2_C1.cfg TaskLock:cfg/TL_N3_T2_R3_C1.cfg
# TLC_TIMEOUT 등 환경 변수는 run_tlc.sh 로 그대로 넘어간다.
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for item in "$@"; do
  mod="${item%%:*}"
  cfg="${item#*:}"
  echo "== $mod $cfg $(date -Is)"
  "$here/run_tlc.sh" "$mod" "$cfg"
done
