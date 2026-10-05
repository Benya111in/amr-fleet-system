#!/usr/bin/env bash
# Isaac Lab 컨테이너에서 rl/isaac 스크립트를 돈다 (마운트·환경변수는 /home/CAPYI_AI/env/verify/verify_isaac.sh 그대로).
#   rl/isaac/run_isaac.sh <태그> <worktree 기준 스크립트 경로> [인자…]
# 예: rl/isaac/run_isaac.sh smoke rl/isaac/scripts/smoke_env.py --num_envs 256
# 컨테이너 이름 amr-ai-r1-isaac-<태그>, --rm. 시작 전 GPU 여유 ≥ 16 GB 를 확인한다 (COORDINATION §3).
set -uo pipefail
A=/home/CAPYI_AI
C=$A/cache/isaac
WT=$(cd "$(dirname "$0")/../.." && pwd)
TAG=$1
SCRIPT=$2
shift 2
mkdir -p $C/{kit,ov,pip,glcache,computecache,logs,data,docs} $A/runs/r1-isaac
FREE=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)
echo "GPU free MiB: $FREE"
[ "$FREE" -ge 16000 ] || { echo "SKIP: GPU 여유 부족 ($FREE MiB < 16000)"; exit 3; }
exec docker run --rm --name amr-ai-r1-isaac-$TAG --gpus all --shm-size=8g \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N -e OMNI_KIT_ACCEPT_EULA=YES \
  -e ROS_DOMAIN_ID=214 -e IGN_PARTITION=amr_ai_r1 -e PYTHONUNBUFFERED=1 ${EXTRA_DOCKER_ARGS:-} \
  -v $C/kit:/isaac-sim/kit/cache -v $C/ov:/root/.cache/ov -v $C/pip:/root/.cache/pip \
  -v $C/glcache:/root/.cache/nvidia/GLCache -v $C/computecache:/root/.nv/ComputeCache \
  -v $C/logs:/root/.nvidia-omniverse/logs -v $C/data:/root/.local/share/ov/data -v $C/docs:/root/Documents \
  -v "$WT":/workspace/amr -v $A/runs/r1-isaac:$A/runs/r1-isaac \
  --entrypoint bash amr-ai/isaaclab:2.3.2 -lc \
  "cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/amr/$SCRIPT $*"
