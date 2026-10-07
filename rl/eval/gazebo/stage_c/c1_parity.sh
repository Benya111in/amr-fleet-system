#!/usr/bin/env bash
# C1-a: 대표 정책 ONNX 의 Python ORT 대 C++ OnnxPolicy 일치 (A2 도구 run_parity.sh 를 고정 사본에서 — 컨테이너 이름만
# amr-ai-b1-gzc-* 로, 작업공간은 읽기 전용, git 대신 SNAPSHOT_COMMIT). 호스트에서:
#   c1_parity.sh <seed>     → /home/CAPYI_AI/runs/bonus1/b1-gzc/c1/out_s<seed>/report.json
# 입력 묶음: golden 17 · 무작위 계약 상태 4096 · 표준정규 4096 · Isaac 기록 관측(대표 정책의 집합 평가와 판정 평가 obs_batch).
set -euo pipefail
S=$1
R=/home/CAPYI_AI/runs/bonus1/b1-gzc; T=/home/CAPYI_AI/runs/bonus1/b1-ppo10/train/B1_ppo10_s$S
E=/home/CAPYI_AI/runs/bonus1/b1-ppo10-eval/eval/B1e_ppo10_s${S}_final_sets
RUN=$R/c1/runs/s$S
mkdir -p "$RUN/eval_final_sets" "$RUN/eval_final_3-15" "$RUN/eval_final_3-30"
\cp -f "$T/policy_final.onnx" "$T/policy_final.pt" "$RUN/"
\cp -f "$E/obs_batch.npz" "$RUN/eval_final_sets/"
\cp -f "$T/B1_ppo10_s${S}_final_canonical_3-15/obs_batch.npz" "$RUN/eval_final_3-15/"
\cp -f "$T/B1_ppo10_s${S}_final_canonical_3-30/obs_batch.npz" "$RUN/eval_final_3-30/"
sha256sum "$RUN/policy_final.onnx" "$T/policy_final.onnx" > "$RUN/sha256.txt"
SRC=$R/ws/src/amr_rl_controller/tools/parity/run_parity.sh
P=$R/c1/run_parity_gzc.sh
sed -e 's/amr-ai-b1-plugin-parity/amr-ai-b1-gzc-parity/; s/amr-ai-b1-plugin-rsl/amr-ai-b1-gzc-rsl/' \
    -e 's#-v \$WT:/ros2_ws -v#-v $WT:/ros2_ws:ro -v#' \
    -e 's#-v "\$WT":/ros2_ws -v#-v "$WT":/ros2_ws:ro -v#' \
    -e 's#(cd "\$WT" \&\& git rev-parse HEAD \&\& git status --porcelain)#cat "$WT/SNAPSHOT_COMMIT"#' \
    "$SRC" > "$P"
diff "$SRC" "$P" > "$R/c1/run_parity_gzc.diff" || true
bash "$P" "$R/c1/out_s$S" "s$S=$RUN" "$R/ws"
