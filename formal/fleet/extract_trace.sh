#!/usr/bin/env bash
# TLC 로그에서 위반 트레이스를 뽑아 traces/<cfg 이름>.txt 로 저장한다.
# 사용법: formal/fleet/extract_trace.sh <TLC 로그 경로>
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
log="${1:?로그 경로}"
name="$(basename "$log" .log)"
out="$here/traces/$name.txt"
mkdir -p "$here/traces"
{
  grep -E '^# (model|jar|head|spec_sha256|start|end)' "$log"
  grep -E '^TLC2 Version' "$log"
  echo
  echo "## 상태마다 바뀐 변수만 (trace_diff.py)"
  python3 "$here/trace_diff.py" "$log"
  echo
  echo "## TLC 원문"
  sed -n '/^Error: Invariant/,/states generated/p' "$log" | sed 's/[[:space:]]*$//'
} > "$out"
echo "$out ($(grep -cE '^State [0-9]+:' "$log") states)"
