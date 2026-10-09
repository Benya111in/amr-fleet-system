#!/usr/bin/env bash
# 배치 묶음 하나로 5 대 Gazebo 실행 한 번 (layopt E3, docs/research/layout-opt/gen.md §5). 호스트에서 포그라운드로.
#   layopt/gen/run_e3.sh <spec_id> [LO|LOd|LOsmoke] [p5_driver.py 추가 인자 …]
# 환경 (선택):
#   E3_WS     빌드한 작업공간 (기본 /home/CAPYI_AI/runs/layopt/tmp/gen/ws/96a8883 — git archive 고정본)
#   E3_RAW    원자료 루트 (기본 /home/CAPYI_AI/runs/layopt/e3) → <E3_RAW>/<UTC>_<spec_id>_<실험>/
#   E3_AGENT  실행 목록 agent → 이 작업 트리 docs/research/layout-opt/notebook/runs/<E3_AGENT>.jsonl (기본 layopt-e3)
#   E3_SLOT   gz_slot.sh id (기본 layopt-e3)
# 드라이버가 스택 컨테이너를 gz_slot.sh 로 감싸 포그라운드에서 돈다. 반환: 0 유효, 4 무효, 1 STOP, 2 인자 오류.
set -euo pipefail
WT=$(cd "$(dirname "$0")/../.." && pwd)
SPEC=${1:?사용: run_e3.sh <spec_id> [LO|LOd|LOsmoke] [추가 인자]}
EXP=${2:-LO}
shift $(( $# >= 2 ? 2 : 1 ))
WS=${E3_WS:-/home/CAPYI_AI/runs/layopt/tmp/gen/ws/96a8883}
RAW=${E3_RAW:-/home/CAPYI_AI/runs/layopt/e3}
TAG=$(date -u +%Y%m%dT%H%M%SZ)_${SPEC}_${EXP}
mkdir -p "$RAW"
exec python3 "$WT/tests/integration/fleet_p5/p5_driver.py" \
    --raw "$RAW" --ws-new "$WS" --ws-old "$WS" --repo "$WT" \
    --bundle "/home/CAPYI_AI/runs/layopt/bundles/$SPEC" \
    --order "$EXP" --run-one "$TAG" --domain 59 --t-start 60 \
    --agent-id layopt-e3 --slot-id "${E3_SLOT:-layopt-e3}" --partition amr_ai_layopt_e3 \
    --notebook docs/research/layout-opt/notebook --stage E3 --run-id-prefix '' \
    --record-agent "${E3_AGENT:-layopt-e3}" --seal "$@"
