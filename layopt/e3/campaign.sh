#!/usr/bin/env bash
# layopt 실험 0·4 — 5 대 Gazebo(E3) 캠페인. 배치를 번갈아(A B C D A B C D …) 한 번씩 돌려 시간대·부하 교란을 고르게 나눈다.
# 사용: campaign.sh <캠페인 디렉터리>   (디렉터리 안 specs.txt = 배치 목록, 매 라운드 다시 읽는다 → 무릎점을 나중에 더할 수 있다)
# 환경: TARGET(배치당 유효 목표, 기본 3), STRETCH(시간이 남으면 올릴 목표, 기본 5), MAXTRY(배치당 시도 상한, 기본 8)
# 멈추기: 캠페인 디렉터리에 STOP 파일을 만들면 지금 실행이 끝난 뒤 멈춘다.
# 실행마다 exclusive.sh 로 gz·cpu 슬롯 전부를 잡는다 (공유 부하 아래 5 대 기동이 무너졌다 — gen.md §6).
set -u
C=${1:?캠페인 디렉터리}
R=/home/CAPYI_AI/runs/layopt
RUN=/home/CAPYI_AI/wt/layopt/layopt/gen/run_e3.sh
TARGET=${TARGET:-3}; STRETCH=${STRETCH:-5}; MAXTRY=${MAXTRY:-8}
LOG=$C/campaign.jsonl
touch "$LOG"
count() {  # $1 spec, $2 필드(valid|try)
  python3 -I -c "
import json,sys
n=0
for l in open('$LOG'):
    d=json.loads(l)
    if d['spec']=='$1' and ('$2'=='try' or d.get('rc')==0): n+=1
print(n)"
}
goal=$TARGET
while :; do
  [ -e "$C/STOP" ] && { $R/bin/jlog main "E3 캠페인 STOP 파일로 멈춤 ($C)"; break; }
  mapfile -t SPECS < <(grep -v '^#' "$C/specs.txt" | sed '/^$/d')
  ran=0
  for s in "${SPECS[@]}"; do
    [ -e "$C/STOP" ] && break
    v=$(count "$s" valid); t=$(count "$s" try)
    if [ "$v" -ge "$goal" ] || [ "$t" -ge "$MAXTRY" ]; then continue; fi
    st=$(date -u +%FT%TZ); l0=$($R/bin/load)
    $R/bin/jlog main "E3 시작 $s (유효 $v/$goal, 시도 $t) 부하 $l0"
    /home/CAPYI_AI/env/exclusive.sh layopt-e3 env E3_AGENT=layopt-e3 E3_STAGGER=15 "$RUN" "$s" LO
    rc=$?
    tag=$(ls -1t $R/e3 | grep "_${s}_LO$" | head -1)
    printf '{"spec": "%s", "tag": "%s", "rc": %d, "started": "%s", "ended": "%s", "load_start": "%s", "load_end": "%s"}\n' \
      "$s" "$tag" "$rc" "$st" "$(date -u +%FT%TZ)" "$l0" "$($R/bin/load)" >> "$LOG"
    $R/bin/jlog main "E3 끝 $s rc=$rc ($( [ $rc -eq 0 ] && echo 유효 || echo 무효)) 태그 $tag 부하 $($R/bin/load)"
    $R/bin/snapshot || true
    ran=1
  done
  if [ "$ran" -eq 0 ]; then
    if [ "$goal" -lt "$STRETCH" ]; then goal=$((goal + 1)); $R/bin/jlog main "E3 캠페인: 모든 배치가 목표 도달 → 목표 $goal 로 올림"; continue; fi
    $R/bin/jlog main "E3 캠페인 완료 (목표 $goal, $C)"; break
  fi
done
