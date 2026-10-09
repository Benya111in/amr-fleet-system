#!/usr/bin/env bash
# E1 스윕을 컨테이너(amr-ai/rl:0.3)에서 cpu_slot.sh 를 거쳐 포그라운드로 돈다.
#   layopt/sim/run_sweep.sh <tag> [--resume <run_dir>] [--no-slot] -- <sweep.py 인자 (--out 빼고)>
# 새 실행은 /home/CAPYI_AI/runs/layopt/e1/<UTC YYYYmmddTHHMMSSZ>_<tag> (이미 있으면 거부).
# --resume <dir> 은 그 디렉터리에 이어 한다 (봉인된 디렉터리는 거부).
# 커밋 · 미커밋 여부를 호스트에서 읽어 LAYOPT_COMMIT · LAYOPT_DIRTY 로 넘긴다 (컨테이너는 git 을 못 본다).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WT="$(cd "${HERE}/../.." && pwd)"
[ $# -ge 1 ] || { echo "사용: $0 <tag> [--resume <dir>] [--no-slot] -- <sweep 인자>" >&2; exit 2; }
TAG=$1; shift
OUT=""; SLOT=1
while [ $# -gt 0 ]; do
  case "$1" in
    --resume) OUT=$2; shift 2 ;;
    --no-slot) SLOT=0; shift ;;
    --) shift; break ;;
    *) echo "모르는 인자: $1" >&2; exit 2 ;;
  esac
done
if [ -z "$OUT" ]; then
  OUT=/home/CAPYI_AI/runs/layopt/e1/$(date -u +%Y%m%dT%H%M%SZ)_${TAG}
  [ -e "$OUT" ] && { echo "이미 있다: $OUT" >&2; exit 3; }
  mkdir -p "$OUT"
else
  [ -d "$OUT" ] || { echo "없다: $OUT" >&2; exit 3; }
  [ -e "$OUT/SEALED" ] && { echo "봉인됨: $OUT" >&2; exit 3; }
fi
COMMIT=$(git -C "$WT" rev-parse --short HEAD)
if [ -n "$(git -C "$WT" status --porcelain --untracked-files=no)" ]; then DIRTY=true; else DIRTY=false; fi
echo "run_dir=$OUT commit=$COMMIT dirty=$DIRTY"
CMD=(docker run --rm --name "amr-ai-layopt-sim-${TAG}" --cpus 12 --network none
     -e LAYOPT_COMMIT="$COMMIT" -e LAYOPT_DIRTY="$DIRTY" -e PYTHONDONTWRITEBYTECODE=1
     -v "$WT":/ros2_ws:ro -v /home/CAPYI_AI/runs/layopt/bundles:/home/CAPYI_AI/runs/layopt/bundles:ro
     -v "$OUT":"$OUT" -w /ros2_ws amr-ai/rl:0.3
     python3 -m layopt.sim.sweep --out "$OUT" "$@")
if [ "$SLOT" = 1 ]; then
  exec /home/CAPYI_AI/env/cpu_slot.sh layopt-sim "${CMD[@]}"
else
  exec "${CMD[@]}"
fi
