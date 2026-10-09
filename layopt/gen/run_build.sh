#!/usr/bin/env bash
# 배치 묶음 만들기 (호스트 진입점): 생성기 커밋을 재고 amr-ai/rl:0.3 컨테이너에서 만든다.
#   layopt/gen/run_build.sh one <spec_id>            # 묶음 하나 → runs/layopt/bundles/<spec_id> (봉인)
#   layopt/gen/run_build.sh all [--workers N]        # 540 개 전부 (있는 묶음은 건너뜀, cpu_slot.sh 안에서 부를 것)
# 작업 트리에 커밋 안 한 변경이 있으면 거부한다 (ALLOW_DIRTY=1 이면 spec.json 에 generator_dirty=true 로 남기고 진행).
set -euo pipefail
WT=$(cd "$(dirname "$0")/../.." && pwd)
RUNS=/home/CAPYI_AI/runs/layopt
COMMIT=$(git -C "$WT" rev-parse HEAD)
DIRTY=()
if [ -n "$(git -C "$WT" status --porcelain)" ]; then
    if [ "${ALLOW_DIRTY:-0}" != 1 ]; then
        echo "작업 트리에 커밋 안 한 변경이 있다 — 커밋하거나 ALLOW_DIRTY=1" >&2
        exit 2
    fi
    DIRTY=(--dirty)
fi
MODE=${1:?one|all}
shift
case "$MODE" in
    one) CMD=(python3 -m layopt.gen.build_bundle --spec "${1:?spec_id}" --commit "$COMMIT" "${DIRTY[@]}") ;;
    all) CMD=(python3 -m layopt.gen.build_all --commit "$COMMIT" "${DIRTY[@]}" "$@") ;;
    *) echo "모드: one | all" >&2; exit 2 ;;
esac
exec docker run --rm --name "amr-ai-layopt-gen-bundle-$$" --network none \
    -v "$WT":/ros2_ws:ro -v "$RUNS":"$RUNS" -w /ros2_ws -e PYTHONDONTWRITEBYTECODE=1 \
    amr-ai/rl:0.3 "${CMD[@]}"
