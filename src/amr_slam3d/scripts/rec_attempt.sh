#!/usr/bin/env bash
# 녹화 시도 하나 (호스트, rec_driver.sh 가 gz_slot.sh 안에서 부른다 — 슬롯을 얻은 뒤에 출처를 쓴다).
#
#   rec_attempt.sh <worktree> <시도 디렉터리> <태그> <route 이름> <월드 변형> <spawn 경로(컨테이너 안) | ->
#
# 시도 디렉터리에 HEAD.txt·diff_stat.txt·status.txt(src 미커밋)·started.txt·loadavg_start.txt 를 쓰고
# docker run (amr-ai-<AGENT>-gz, --rm, 포그라운드) 으로 record_bag.sh 를 돈다. 반환 코드 = record_bag.sh 의 것.
# 환경: DOMAIN·IMAGE·AGENT (rec_driver.sh 가 준다), MEM_LIMIT (컨테이너 --memory, 기본 28g — COORDINATION §12).
# <worktree> 가 git archive 사본(SNAPSHOT_HEAD 파일 있음)이면 HEAD.txt = 그 커밋, diff·status 는 비운다 (b2-rerec:
# 녹화가 도는 동안 작업 worktree 를 고쳐도 녹화 코드가 바뀌지 않게 — bash 는 스크립트를 읽어 가며 실행한다).
set -uo pipefail
WT=${1:?}; D=${2:?}; TAG=${3:?}; ROUTE=${4:?}; VARIANT=${5:?}; SP=${6:?}
DOMAIN=${DOMAIN:-47}; IMAGE=${IMAGE:-amr-ai/slam:0.1}; AGENT=${AGENT:-b2-rec}; MEM_LIMIT=${MEM_LIMIT:-28g}
# 시작 조건: loadavg 1 분값 < LOAD_MAX (기본 8, b4-s12 짝 실행과 같은 문턱) — 다른 사용자 부하가 몰릴 때 시작하지 않는다
# (s1_seed2 의 두 실패가 부하 급증과 겹쳤다). LOAD_WAIT_MAX (기본 21600 s) 넘게 기다리면 그대로 시작하고 적어 둔다.
LOAD_MAX=${LOAD_MAX:-8}; LOAD_WAIT_MAX=${LOAD_WAIT_MAX:-21600}
w0=$SECONDS
while :; do
  read -r l1 _ < /proc/loadavg
  if awk -v a="$l1" -v b="$LOAD_MAX" 'BEGIN { exit !(a < b) }'; then break; fi
  if [ $((SECONDS - w0)) -ge "$LOAD_WAIT_MAX" ]; then echo "load_wait_timeout" > "$D/load_wait.txt"; break; fi
  sleep 30
done
echo "load_wait_s: $((SECONDS - w0)) load1_at_start: $l1 limit: $LOAD_MAX" >> "$D/load_wait.txt"
if [ -f "$WT/SNAPSHOT_HEAD" ]; then
  tr -d '[:space:]' < "$WT/SNAPSHOT_HEAD" > "$D/HEAD.txt"; echo >> "$D/HEAD.txt"
  echo "snapshot (git archive $(cat "$D/HEAD.txt")) — 미커밋 변경 없음" > "$D/diff_stat.txt"
  : > "$D/status.txt"
else
  git -C "$WT" rev-parse HEAD > "$D/HEAD.txt"
  git -C "$WT" diff --stat > "$D/diff_stat.txt"
  git -C "$WT" status --porcelain -- src > "$D/status.txt"
fi
date -Iseconds > "$D/started.txt"
l=$(< /proc/loadavg)
echo "$l" > "$D/loadavg_start.txt"
NAME="amr-ai-$AGENT-gz"
docker rm -f "$NAME" > /dev/null 2>&1 || true
exec docker run --rm --name "$NAME" --gpus all --shm-size=8g --memory "$MEM_LIMIT" \
  -e ROS_DOMAIN_ID="$DOMAIN" -e IGN_PARTITION="b2rec_$TAG" -e ROS_LOCALHOST_ONLY=1 \
  -e GIT_HEAD="$(cat "$D/HEAD.txt")" \
  -v "$WT":/ros2_ws:ro -v "$D":/out -w /ros2_ws "$IMAGE" \
  bash -lc "/ros2_ws/src/amr_slam3d/scripts/record_bag.sh /out /ros2_ws/src/amr_slam3d/config/routes/$ROUTE.yaml $VARIANT $SP"
