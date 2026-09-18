#!/usr/bin/env bash
# 호스트 준비 스크립트 — 새 머신에서 이 프로젝트를 돌리기 위한 1회성 세팅.
# 이 서버(Rocky Linux 8)에서는 이미 적용되어 있으므로 재실행할 필요는 없다.
set -euo pipefail

log() { echo -e "\033[1;32m[setup]\033[0m $*"; }
warn() { echo -e "\033[1;33m[warn]\033[0m $*"; }

# --- 1. Docker ---------------------------------------------------------------
if ! command -v docker >/dev/null 2>&1; then
    log "Docker CE 설치"
    dnf config-manager --add-repo https://download.docker.com/linux/centos/docker-ce.repo
    dnf install -y docker-ce docker-ce-cli containerd.io \
                   docker-buildx-plugin docker-compose-plugin
    systemctl enable --now docker
else
    log "Docker 이미 설치됨: $(docker --version)"
fi

# --- 2. NVIDIA Container Toolkit ---------------------------------------------
if ! command -v nvidia-ctk >/dev/null 2>&1; then
    log "nvidia-container-toolkit 설치"
    curl -sSL https://nvidia.github.io/libnvidia-container/stable/rpm/nvidia-container-toolkit.repo \
        -o /etc/yum.repos.d/nvidia-container-toolkit.repo
    dnf install -y nvidia-container-toolkit
    nvidia-ctk runtime configure --runtime=docker
    systemctl restart docker
else
    log "nvidia-container-toolkit 이미 설치됨"
fi

# --- 3. GPU 컨테이너 동작 검증 -----------------------------------------------
log "GPU 컨테이너 접근 검증"
if docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi >/dev/null 2>&1; then
    log "GPU 패스스루 정상"
else
    warn "GPU 패스스루 실패 — nvidia-smi 및 드라이버를 확인하라"
fi

# --- 4. 헤드리스 렌더링 확인 --------------------------------------------------
# DISPLAY 가 없어도 Gazebo 센서 렌더링이 되려면 EGL 디바이스가 보여야 한다.
log "EGL 헤드리스 렌더링 확인"
python3 - <<'PY' || warn "EGL 확인 실패 — 헤드리스 렌더링이 안 될 수 있다"
import ctypes
egl = ctypes.CDLL('libEGL.so.1')
egl.eglGetProcAddress.restype = ctypes.c_void_p
addr = egl.eglGetProcAddress(b'eglQueryDevicesEXT')
proto = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_int,
                         ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_int))
fn = proto(addr)
devs = (ctypes.c_void_p * 16)()
n = ctypes.c_int(0)
fn(16, devs, ctypes.byref(n))
print(f"[setup] EGL 디바이스 {n.value}개 감지")
assert n.value > 0
PY

log "호스트 준비 완료. 다음: docker compose build dev"
