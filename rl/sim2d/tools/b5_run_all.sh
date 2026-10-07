#!/usr/bin/env bash
# B5 본 실행 드라이버 — 끊겨도 다시 부르면 끝난 run(summary.json complete)은 건너뛴다.
#   nohup setsid bash rl/sim2d/tools/b5_run_all.sh <lane> > <log> 2>&1 &
# lane: dwa   — DWA 주(W1) 다음 보조(W1i), cpu_slot.sh 안 (C++ 스레드 16)
#       rl    — RL 시드 0–9 주(W1) 다음 보조(W1i), cpu_slot.sh 안 (onnxruntime 스레드 8)
#       plan  — 평가 행 경로 계획 (주 경로 파일이 없을 때만)
# 컨테이너: amr-ai/rl:0.3, 이름 amr-ai-b1-b5-<용도>, --rm, 기본 브리지 네트워크,
# 워크트리·runs 를 호스트와 같은 경로로 마운트 (raw_dir·git 이 호스트 경로로 남게).
set -u
ID=b1-b5
WT=/home/CAPYI_AI/wt/b1-b5
R=/home/CAPYI_AI/runs/bonus1/b1-b5
A=/home/CAPYI_AI/runs/bonus1/b1-ppo10-eval/assets
P=/home/CAPYI_AI/runs/bonus1/b1-ppo10/train
WORLD=$R/world/world_isaac_obb_r025_p000.yaml
PLANS=$R/plans/plans_eval_v1.npz
POOL=$A/paths_58eaa34ae5_s1.npz
SLOT=/home/CAPYI_AI/env/cpu_slot.sh
lane=${1:?lane}

DOCK=(--rm --gpus all --shm-size=4g -v "$WT:$WT" -v /home/CAPYI_AI/repo:/home/CAPYI_AI/repo:ro
      -v "$R:$R" -v "$A:$A:ro" -v "$P:$P:ro" -e TZ=Asia/Seoul -e "SIM2D_CACHE=$R/cache" -w "$WT")

is_done() {
  python3 -c "import json,sys; sys.exit(0 if json.load(open('$1/summary.json')).get('complete') else 1)" \
    2>/dev/null
}

run_one() {   # run_one <tag W1|W1i> <arm> <seed|-> <threads>
  local tag=$1 arm=$2 seed=$3 th=$4 out name
  local extra=()
  if [ "$arm" = dwa ]; then
    out=$R/eval/B5_dwa_$tag; name=dwa-$tag
  else
    out=$R/eval/B5_rl_s${seed}_$tag; name=rl$seed-$tag
    extra=(--policy "$P/B1_ppo10_s$seed/policy_final.onnx")
  fi
  if [ "$tag" = W1 ]; then extra+=(--paths-npz "$PLANS"); fi
  if is_done "$out"; then echo "$(date '+%F %T') 건너뜀 $out"; return 0; fi
  echo "$(date '+%F %T') 시작 $out"
  "$SLOT" "$ID" docker run --name "amr-ai-$ID-$name" "${DOCK[@]}" amr-ai/rl:0.3 \
    python3 rl/sim2d/tools/b5_crosseval.py --arm "$arm" --world "$WORLD" --pool "$POOL" "${extra[@]}" \
    --seed 4242 --threads "$th" --out "$out" --run-id "$(basename "$out")"
  echo "$(date '+%F %T') 끝 rc=$? $out"
}

case $lane in
  plan)
    if [ ! -f "$PLANS" ]; then
      "$SLOT" "$ID" docker run --name "amr-ai-$ID-plan" "${DOCK[@]}" amr-ai/rl:0.3 \
        python3 rl/sim2d/tools/b5_plan_paths.py --pool "$POOL" --geom "$A/warehouse_58eaa34ae5.npz" \
        --world-dir "$R/world" --threads 16 --out "$PLANS"
    fi ;;
  dwa) for tag in W1 W1i; do run_one "$tag" dwa - 16; done ;;
  rl) for tag in W1 W1i; do for s in 0 1 2 3 4 5 6 7 8 9; do run_one "$tag" rl "$s" 8; done; done ;;
  *) echo "lane?" >&2; exit 2 ;;
esac
