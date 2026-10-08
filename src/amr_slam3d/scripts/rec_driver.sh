#!/usr/bin/env bash
# 재개 가능한 녹화 구동기 (보너스 2 G 단계, 호스트에서 nohup setsid 로 띄운다 — 감독이 끊겨도 이어지게).
#
#   rec_driver.sh <worktree> <녹화 루트> <계획 파일>
#
# 계획 파일: 빈 줄·# 줄은 무시. 줄마다 "<id> <route 이름> <월드 변형> <spawn 이름 | -> <씨드 | ->"
#   route 이름 → <worktree>/src/amr_slam3d/config/routes/<이름>.yaml, spawn 이름 → config/spawn/<이름>.yaml
# id 마다:
#   - <루트>/state/<id>.ok (통과한 태그) 또는 <id>.failed 가 있으면 건너뛴다 (끝난 것은 다시 하지 않는다)
#   - 시도 k = 1..MAX_ATTEMPTS (기본 3, 사전 등록): 태그 <id>_a<k>. 그 디렉터리가 이미 있으면 (끊긴 시도 포함) 다음 k
#   - 시도 디렉터리에 HEAD.txt·diff_stat.txt·status.txt·started.txt·loadavg_start.txt 를 쓰고 gz_slot.sh 로
#     docker run … record_bag.sh, 끝나면 rc.txt·loadavg_end.txt, rec_report.py registry → notebook.py register
#   - rc 0 (공백 검사 통과) 이면 state/<id>.ok ← 태그. 아니면 다음 시도. MAX 회 모두 실패면 state/<id>.failed
# 로그: <루트>/driver.log. 같은 구동기를 두 개 띄우지 않게 <루트>/driver.lock (flock -n) 을 잡는다.
# 환경: DOMAIN (기본 47), MAX_ATTEMPTS (3), IMAGE (amr-ai/slam:0.1), AGENT (b2-rec), NB (노트 경로, 저장소 기준)
set -uo pipefail
WT=${1:?worktree}; ROOT=${2:?녹화 루트}; PLAN=${3:?계획 파일}
DOMAIN=${DOMAIN:-47}; MAX=${MAX_ATTEMPTS:-3}; IMAGE=${IMAGE:-amr-ai/slam:0.1}; AGENT=${AGENT:-b2-rec}
NB=${NB:-docs/research/slam3d/notebook}
CFG=$WT/src/amr_slam3d/config
mkdir -p "$ROOT/state" "$ROOT/rec"
exec 9> "$ROOT/driver.lock"
if ! flock -n 9; then echo "이미 다른 구동기가 돈다 ($ROOT/driver.lock)" >&2; exit 1; fi
LOG=$ROOT/driver.log
load() { local l; l=$(< /proc/loadavg); echo "$l"; }
say() { echo "$(date '+%F %T') $*" | tee -a "$LOG"; }

say "DRIVER start pid=$$ wt=$WT plan=$PLAN max=$MAX domain=$DOMAIN load=$(load)"
while read -r id route variant spawn seed _rest; do
  [ -z "${id:-}" ] && continue
  case "$id" in \#*) continue ;; esac
  if [ -e "$ROOT/state/$id.ok" ] || [ -e "$ROOT/state/$id.failed" ]; then
    say "SKIP $id ($(ls "$ROOT/state/$id".* | xargs -n1 basename))"
    continue
  fi
  passed=""
  for k in $(seq 1 "$MAX"); do
    tag="${id}_a${k}"
    d="$ROOT/rec/$tag"
    if [ -e "$d" ]; then
      say "SKIP-ATTEMPT $tag (이미 있음 — 끊긴 시도는 다시 쓰지 않는다)"
      if [ -f "$d/rc.txt" ] && [ "$(cat "$d/rc.txt")" = "0" ]; then passed=$tag; break; fi
      continue
    fi
    mkdir -p "$d"
    git -C "$WT" rev-parse HEAD > "$d/HEAD.txt"
    git -C "$WT" diff --stat > "$d/diff_stat.txt"
    git -C "$WT" status --porcelain -- src > "$d/status.txt"
    date -Iseconds > "$d/started.txt"
    load > "$d/loadavg_start.txt"
    sp=-
    [ "$spawn" != "-" ] && sp=/ros2_ws/src/amr_slam3d/config/spawn/$spawn.yaml
    name="amr-ai-$AGENT-gz"
    docker rm -f "$name" > /dev/null 2>&1 || true
    say "START $tag route=$route variant=$variant spawn=$spawn head=$(cut -c1-7 "$d/HEAD.txt") load=$(load)"
    /home/CAPYI_AI/env/gz_slot.sh "$AGENT" docker run --rm --name "$name" --gpus all --shm-size=8g \
      -e ROS_DOMAIN_ID="$DOMAIN" -e IGN_PARTITION="b2rec_$tag" -e ROS_LOCALHOST_ONLY=1 \
      -e GIT_HEAD="$(cat "$d/HEAD.txt")" \
      -v "$WT":/ros2_ws:ro -v "$d":/out -w /ros2_ws "$IMAGE" \
      bash -lc "/ros2_ws/src/amr_slam3d/scripts/record_bag.sh /out /ros2_ws/src/amr_slam3d/config/routes/$route.yaml $variant $sp" \
      > "$d/host.log" 2>&1 < /dev/null
    rc=$?
    echo "$rc" > "$d/rc.txt"
    load > "$d/loadavg_end.txt"
    say "END $tag rc=$rc load=$(load)"
    python3 "$WT/src/amr_slam3d/scripts/rec_report.py" registry "$d" --run-id "B2rec-$tag" \
      --scenario "$id" --attempt "$k" --rc "$rc" --seed "${seed:--}" --out "$d/registry.json" \
      >> "$LOG" 2>&1 \
      && python3 "$WT/scripts/notebook.py" --nb "$NB" register --file "$d/registry.json" >> "$LOG" 2>&1 \
      || say "REGISTER-FAIL $tag"
    if [ "$rc" = 0 ]; then passed=$tag; break; fi
  done
  if [ -n "$passed" ]; then
    echo "$passed" > "$ROOT/state/$id.ok"
    say "OK $id → $passed"
  else
    echo "attempts=$MAX" > "$ROOT/state/$id.failed"
    say "FAILED $id ($MAX 회 모두 실패)"
  fi
done < "$PLAN"
say "DRIVER end load=$(load)"
