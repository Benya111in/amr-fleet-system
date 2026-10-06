#!/usr/bin/env bash
# 학습 정책 출력 일치 전체 (보너스 1 A2) — 호스트에서 실행. 워크트리는 빌드돼 있어야 한다
# (install/amr_rl_controller/lib/amr_rl_controller/parity_dump).
#   run_parity.sh <출력 디렉터리> <s0=run_dir,s1=run_dir,...> [워크트리]
# 단계: (1) amr-ai/rl:0.3 — 입력 묶음 (gen_states.py, prep-isaac)
#       (2) amr-ai/isaaclab:2.3.2 — 학습 때의 rsl_rl ActorCritic 출력 (CPU, GPU 여유가 있으면 CUDA 도)
#       (3) amr-ai/rl:0.3 — C++ parity_dump (끝-끝 상태·같은 관측·반복·별개 프로세스·intra_op 4)
#       (4) amr-ai/rl:0.3 — parity_check.py check → <출력>/report.json
set -euo pipefail
OUT=$(realpath -m "$1"); RUNS=$2; WT=$(realpath "${3:-$(dirname "$0")/../../../..}")
mkdir -p "$OUT"/data "$OUT"/cpp "$OUT"/rsl
RUNS_C=$(echo "$RUNS" | sed 's#=/home/CAPYI_AI/runs/#=/runs/#g')
TOOLS=/ros2_ws/src/amr_rl_controller/tools/parity
RL="docker run --rm --name amr-ai-b1-plugin-parity -v $WT:/ros2_ws -v $OUT:/out \
  -v /home/CAPYI_AI/runs:/runs:ro -w /ros2_ws amr-ai/rl:0.3 bash -lc"
echo "load_start $(cat /proc/loadavg)" > "$OUT"/load.txt
date -Iseconds > "$OUT"/started.txt
(cd "$WT" && git rev-parse HEAD && git status --porcelain) > "$OUT"/HEAD.txt

$RL "python3 $TOOLS/gen_states.py --out /out/data && \
     python3 $TOOLS/parity_check.py prep-isaac --runs '$RUNS_C' --out /out/data" \
  > "$OUT"/1_gen.log 2>&1

FREE=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1 || echo 0)
CUDA=""; [ "${FREE:-0}" -ge 4000 ] && CUDA="--cuda"
echo "gpu_free_mib $FREE cuda_flag '$CUDA'" >> "$OUT"/load.txt
for kv in ${RUNS//,/ }; do
  s=${kv%%=*}; run=${kv#*=}; runc=${run/\/home\/CAPYI_AI\/runs/\/runs}
  docker run --rm --name amr-ai-b1-plugin-rsl --gpus all -e ACCEPT_EULA=Y -v "$WT":/ros2_ws -v "$OUT":/out \
    -v /home/CAPYI_AI/runs:/runs:ro --entrypoint bash amr-ai/isaaclab:2.3.2 -lc \
    "/isaac-sim/python.sh $TOOLS/rsl_rl_check.py --pt $runc/policy_final.pt $CUDA \
       --out /out/rsl/rsl_$s.npz --obs /out/data/golden_obs_ref.npy /out/data/random_obs_ref.npy \
       /out/data/normal_obs.npy /out/data/isaac_obs_$s.npy" >> "$OUT"/2_rsl.log 2>&1
done

PD=/ros2_ws/install/amr_rl_controller/lib/amr_rl_controller/parity_dump
CMDS="source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && set -e && \
  $PD --model /dev/null --out /out/cpp/integrate_only --integrate /out/data/integrate_in.f64 || true; \
  cp /out/cpp/integrate_only_integrate.f64 /out/cpp/integrate.f64"
for kv in ${RUNS//,/ }; do
  s=${kv%%=*}; run=${kv#*=}; m=${run/\/home\/CAPYI_AI\/runs/\/runs}/policy_final.onnx
  for name in golden random; do
    CMDS="$CMDS && $PD --model $m --states /out/data/${name}_states.jsonl --out /out/cpp/${s}_${name}_e2e"
  done
  for pair in golden_ref:golden_obs_ref random_ref:random_obs_ref normal:normal_obs isaac:isaac_obs_$s; do
    name=${pair%%:*}; f=${pair#*:}
    CMDS="$CMDS && python3 -c \"import numpy as n; n.load('/out/data/$f.npy').astype(n.float32).tofile('/out/data/$f.f32')\""
    CMDS="$CMDS && $PD --model $m --obs /out/data/$f.f32 --out /out/cpp/${s}_${name}"
    CMDS="$CMDS && $PD --model $m --obs /out/data/$f.f32 --repeat 1 --out /out/cpp/${s}_${name}_p2"
    CMDS="$CMDS && $PD --model $m --obs /out/data/$f.f32 --repeat 1 --threads 4 --out /out/cpp/${s}_${name}_t4"
  done
done
$RL "$CMDS" > "$OUT"/3_cpp.log 2>&1

$RL "python3 $TOOLS/parity_check.py check --data /out/data --cpp /out/cpp --rsl /out/rsl \
     --runs '$RUNS_C' --out /out/report.json" > "$OUT"/4_check.log 2>&1 || true
date -Iseconds > "$OUT"/ended.txt
echo "load_end $(cat /proc/loadavg)" >> "$OUT"/load.txt
tail -2 "$OUT"/4_check.log
