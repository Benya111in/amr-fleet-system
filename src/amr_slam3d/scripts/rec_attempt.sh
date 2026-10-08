#!/usr/bin/env bash
# 녹화 시도 하나 (호스트, rec_driver.sh 가 gz_slot.sh 안에서 부른다 — 슬롯을 얻은 뒤에 출처를 쓴다).
#
#   rec_attempt.sh <worktree> <시도 디렉터리> <태그> <route 이름> <월드 변형> <spawn 경로(컨테이너 안) | ->
#
# 시도 디렉터리에 HEAD.txt·diff_stat.txt·status.txt(src 미커밋)·started.txt·loadavg_start.txt 를 쓰고
# docker run (amr-ai-<AGENT>-gz, --rm, 포그라운드) 으로 record_bag.sh 를 돈다. 반환 코드 = record_bag.sh 의 것.
# 환경: DOMAIN·IMAGE·AGENT (rec_driver.sh 가 준다).
set -uo pipefail
WT=${1:?}; D=${2:?}; TAG=${3:?}; ROUTE=${4:?}; VARIANT=${5:?}; SP=${6:?}
DOMAIN=${DOMAIN:-47}; IMAGE=${IMAGE:-amr-ai/slam:0.1}; AGENT=${AGENT:-b2-rec}
git -C "$WT" rev-parse HEAD > "$D/HEAD.txt"
git -C "$WT" diff --stat > "$D/diff_stat.txt"
git -C "$WT" status --porcelain -- src > "$D/status.txt"
date -Iseconds > "$D/started.txt"
l=$(< /proc/loadavg)
echo "$l" > "$D/loadavg_start.txt"
NAME="amr-ai-$AGENT-gz"
docker rm -f "$NAME" > /dev/null 2>&1 || true
exec docker run --rm --name "$NAME" --gpus all --shm-size=8g \
  -e ROS_DOMAIN_ID="$DOMAIN" -e IGN_PARTITION="b2rec_$TAG" -e ROS_LOCALHOST_ONLY=1 \
  -e GIT_HEAD="$(cat "$D/HEAD.txt")" \
  -v "$WT":/ros2_ws:ro -v "$D":/out -w /ros2_ws "$IMAGE" \
  bash -lc "/ros2_ws/src/amr_slam3d/scripts/record_bag.sh /out /ros2_ws/src/amr_slam3d/config/routes/$ROUTE.yaml $VARIANT $SP"
