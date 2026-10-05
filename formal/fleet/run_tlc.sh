#!/usr/bin/env bash
# TLC 를 amr-ai/rl:0.3 컨테이너 안에서 돌린다 (호스트에 아무것도 설치하지 않는다).
#
# 사용법: formal/fleet/run_tlc.sh <모듈> <cfg 경로(formal/fleet 기준)> [추가 TLC 옵션...]
#   예) formal/fleet/run_tlc.sh RA_LiveMembers cfg/RA_any_N5.cfg
#
# 환경 변수
#   TLA_JAR  : tla2tools.jar 경로 (기본 /home/CAPYI_AI/downloads/tla/tla2tools-1.7.4.jar)
#   OUT_DIR  : 로그 디렉터리 (기본 /home/CAPYI_AI/logs/agents/f2-tla/tlc)
#   WORKERS  : TLC 작업자 수 (기본 4, 공유 서버 규칙상 4 이하)
#   HEAP     : JVM 최대 힙 (기본 8g)
#   IMAGE    : 컨테이너 이미지 (기본 amr-ai/rl:0.3)
#   TLC_TIMEOUT : 컨테이너 안 TLC 시간 상한(초, 기본 3000). 넘으면 TLC 를 멈추고 rc=124
#                 (그때는 마지막 Progress 줄만 남는다 -- 전수 검사 미완)
# 결과: $OUT_DIR/<cfg 이름>.log (TLC 원문), 첫 줄들에 판·명령·HEAD 를 남긴다.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mod="${1:?모듈 이름}"
cfg="${2:?cfg 경로}"
shift 2
TLA_JAR="${TLA_JAR:-/home/CAPYI_AI/downloads/tla/tla2tools-1.7.4.jar}"
OUT_DIR="${OUT_DIR:-/home/CAPYI_AI/logs/agents/f2-tla/tlc}"
WORKERS="${WORKERS:-4}"
HEAP="${HEAP:-8g}"
IMAGE="${IMAGE:-amr-ai/rl:0.3}"
TLC_TIMEOUT="${TLC_TIMEOUT:-3000}"
if [ "$WORKERS" -gt 4 ]; then echo "WORKERS<=4 (공유 서버 규칙)" >&2; exit 2; fi
name="$(basename "$cfg" .cfg)"
mkdir -p "$OUT_DIR"
log="$OUT_DIR/$name.log"
jar_dir="$(dirname "$TLA_JAR")"
jar_name="$(basename "$TLA_JAR")"
{
  echo "# model=$mod cfg=$cfg workers=$WORKERS heap=$HEAP image=$IMAGE timeout=${TLC_TIMEOUT}s"
  echo "# jar=$TLA_JAR sha256=$(sha256sum "$TLA_JAR" | cut -d' ' -f1)"
  echo "# head=$(git -C "$here" rev-parse HEAD) dirty=$(git -C "$here" status --porcelain -- . | wc -l)"
  echo "# spec_sha256=$(sha256sum "$here/$mod.tla" | cut -d' ' -f1) cfg_sha256=$(sha256sum "$here/$cfg" | cut -d' ' -f1)"
  echo "# start=$(date -Is) loadavg=$(cut -d' ' -f1-3 /proc/loadavg)"
} > "$log"
# 정확히 무엇을 돌렸는지 남긴다 (스펙·cfg 사본)
mkdir -p "$OUT_DIR/inputs/$name"
cat "$here/$mod.tla" > "$OUT_DIR/inputs/$name/$mod.tla"
cat "$here/$cfg" > "$OUT_DIR/inputs/$name/$(basename "$cfg")"
rc=0
docker run --rm --name "amr-ai-f2-tla-tlc-$name" --network none --cpus "$WORKERS" \
  -v "$here":/spec:ro -v "$jar_dir":/tla:ro -v "$OUT_DIR":/out -w /spec \
  "$IMAGE" bash -c "timeout $TLC_TIMEOUT java -XX:+UseParallelGC -Xmx$HEAP -cp /tla/$jar_name tlc2.TLC \
     -workers $WORKERS -deadlock -cleanup -metadir /out/states/$name \
     -config $cfg $* $mod.tla" >> "$log" 2>&1 || rc=$?
echo "# end=$(date -Is) rc=$rc loadavg=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$log"
grep -E 'states generated|distinct states|depth of the complete|Error:|is violated|No error has been found|Finished in|Model checking completed' "$log" || true
echo "log: $log (rc=$rc)"
exit 0
