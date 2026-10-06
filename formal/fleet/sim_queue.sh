#!/usr/bin/env bash
# 무작위 시뮬레이션 묶음 (전수 검사가 닿지 않는 N=5 범위의 보조 증거). 시드 고정.
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
N_TRACES="${N_TRACES:-1000000}"
"$here/run_tlc.sh" QuorumVote cfg/QV_N5_Z2_R3_C2_forget_sim.cfg -simulate "num=$N_TRACES" -depth 100 -seed 1
"$here/run_tlc.sh" QuorumVote cfg/QV_N5_Z2_R3_C2_sim.cfg -simulate "num=$N_TRACES" -depth 100 -seed 1
"$here/run_tlc.sh" TaskLock cfg/TL_N5_T2_R3_C2_sim.cfg -simulate "num=$N_TRACES" -depth 100 -seed 1
