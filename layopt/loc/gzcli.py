"""
Gazebo Fortress 조작 — `ign service` (tests/integration/amr_itest/gz.py 와 같은 방식).

ros_gz_bridge 에 set_pose/create 브리지가 없어 ign CLI 를 subprocess 로 부른다. IGN_PARTITION 은 환경 변수를
그대로 물려받는다. 더미 로봇은 정적 상자(0.60 × 0.40 × 0.30, 중심 z 0.18 = 실제 차체와 같은 높이 범위
0.03–0.33 m, LiDAR 평면 0.20 m 를 가린다)로 만든다.
"""

import math
import subprocess
from typing import Tuple

DUMMY_SIZE = (0.60, 0.40, 0.30)
DUMMY_Z = 0.18          # config/robot_params.yaml base_link_height (차체 박스 중심)


def pose_request(model: str, x: float, y: float, z: float, yaw: float) -> str:
    """ignition.msgs.Pose 텍스트 (yaw 만 있는 쿼터니언)."""
    return (f'name: "{model}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
            f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')


def dummy_sdf(name: str, x: float, y: float, yaw: float = 0.0) -> str:
    """정적 로봇 모양 상자 SDF (작은따옴표만 — ign --req 문자열 안에 넣는다)."""
    sx, sy, sz = DUMMY_SIZE
    geom = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
    return (f"<sdf version='1.8'><model name='{name}'><static>true</static>"
            f"<pose>{x} {y} {DUMMY_Z} 0 0 {yaw}</pose><link name='link'>"
            f"<collision name='collision'>{geom}</collision>"
            f"<visual name='visual'>{geom}</visual></link></model></sdf>")


def _service(world: str, service: str, reqtype: str, req: str,
             timeout_ms: int = 15000, tries: int = 3) -> Tuple[bool, str]:
    """
    Ign service 호출. 공유 부하(loadavg 40+)에서 5 s 안에 답이 오지 않는 일이 있어(실험 1 s003 실측:
    "Service call timed out") 15 s 상한으로 tries 번까지 다시 부른다. set_pose·create 는 같은 요청을
    다시 보내도 결과가 같다(멱등) — create 는 같은 이름이 이미 있으면 실패하므로 세션 시작에만 쓴다.
    """
    cmd = ['ign', 'service', '-s', f'/world/{world}/{service}',
           '--reqtype', reqtype, '--reptype', 'ignition.msgs.Boolean',
           '--timeout', str(timeout_ms), '--req', req]
    text = ''
    for _ in range(max(tries, 1)):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=timeout_ms / 1000.0 + 5.0, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            text = str(exc)
            continue
        text = (out.stdout + out.stderr).strip()
        if out.returncode == 0 and 'data: true' in text:
            return True, text
    return False, text


def set_pose(world: str, model: str, x: float, y: float, yaw: float, z: float = 0.02,
             timeout_ms: int = 15000) -> Tuple[bool, str]:
    """/world/<world>/set_pose (재시도 포함)."""
    return _service(world, 'set_pose', 'ignition.msgs.Pose', pose_request(model, x, y, z, yaw),
                    timeout_ms)


def create_dummy(world: str, name: str, x: float, y: float, yaw: float = 0.0,
                 timeout_ms: int = 15000) -> Tuple[bool, str]:
    """/world/<world>/create — 정적 더미 상자 (재시도 없음: 같은 이름 중복 생성 방지)."""
    return _service(world, 'create', 'ignition.msgs.EntityFactory',
                    f'sdf: "{dummy_sdf(name, x, y, yaw)}"', timeout_ms, tries=1)


def remove(world: str, name: str, timeout_ms: int = 15000) -> Tuple[bool, str]:
    """/world/<world>/remove (모델)."""
    return _service(world, 'remove', 'ignition.msgs.Entity', f'name: "{name}", type: MODEL',
                    timeout_ms)
