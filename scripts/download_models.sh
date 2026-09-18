#!/usr/bin/env bash
# YOLOv8 가중치 내려받기 (명세 6장).
# 가중치는 대용량이라 git 에 넣지 않는다(.gitignore). 최초 1회 실행한다.
set -euo pipefail

MODEL_DIR="$(dirname "$0")/../src/amr_perception/models"
mkdir -p "${MODEL_DIR}"
cd "${MODEL_DIR}"

# n(nano) = CPU 10FPS 목표용, s(small) = GPU 30FPS 목표용
# 명세 요구: CPU 10FPS 이상 또는 GPU 30FPS 이상
for MODEL in yolov8n.pt yolov8s.pt; do
    if [ -f "${MODEL}" ]; then
        echo "[models] ${MODEL} 이미 존재"
    else
        echo "[models] ${MODEL} 다운로드"
        wget -q --show-progress \
            "https://github.com/ultralytics/assets/releases/download/v8.3.0/${MODEL}"
    fi
done

echo
echo "[models] 완료. 저장 위치: ${MODEL_DIR}"
ls -lh ./*.pt
