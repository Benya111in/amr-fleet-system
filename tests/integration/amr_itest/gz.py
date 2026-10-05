"""
Gazebo(Fortress) 조작: ign CLI 로 모델 자세 설정 (시나리오 05 납치, 10 도킹 시작 자세).

ros_gz_bridge(0.244) 에는 set_pose 서비스 브리지가 없어서 `ign service` 를 subprocess 로 부른다.
IGN_PARTITION 은 환경 변수를 그대로 물려받는다 (하네스 실행 셸과 같은 Gazebo 파티션).
"""

import math
import subprocess
from typing import Tuple


def pose_request(model: str, x: float, y: float, z: float, yaw: float) -> str:
    """ignition.msgs.Pose 텍스트 (yaw 만 있는 쿼터니언)."""
    return (f'name: "{model}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
            f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')


def box_sdf(name: str, x: float, y: float, sx: float, sy: float, sz: float,
            yaw: float = 0.0) -> str:
    """정적 박스 모델 SDF (작은따옴표만 써서 ign --req 문자열에 그대로 넣는다)."""
    geom = f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
    return (f"<sdf version='1.8'><model name='{name}'><static>true</static>"
            f"<pose>{x} {y} {sz / 2} 0 0 {yaw}</pose><link name='link'>"
            f"<collision name='collision'>{geom}</collision>"
            f"<visual name='visual'>{geom}</visual></link></model></sdf>")


def set_pose(world: str, model: str, x: float, y: float, yaw: float, z: float = 0.02,
             timeout_ms: int = 5000) -> Tuple[bool, str]:
    """/world/<world>/set_pose 호출. 반환 (성공, 출력)."""
    return _service(world, 'set_pose', 'ignition.msgs.Pose',
                    pose_request(model, x, y, z, yaw), timeout_ms)


def spawn_box(world: str, name: str, x: float, y: float, sx: float = 1.0, sy: float = 1.0,
              sz: float = 1.0, timeout_ms: int = 5000, yaw: float = 0.0) -> Tuple[bool, str]:
    """정적 박스 장애물 생성 (/world/<world>/create) — 경로 차단·근접 정지·벽 접근 시험."""
    return _service(world, 'create', 'ignition.msgs.EntityFactory',
                    f'sdf: "{box_sdf(name, x, y, sx, sy, sz, yaw)}"', timeout_ms)


def remove(world: str, name: str, timeout_ms: int = 5000) -> Tuple[bool, str]:
    """모델 제거 (/world/<world>/remove)."""
    return _service(world, 'remove', 'ignition.msgs.Entity', f'name: "{name}", type: MODEL',
                    timeout_ms)


def _service(world: str, service: str, reqtype: str, req: str,
             timeout_ms: int) -> Tuple[bool, str]:
    cmd = ['ign', 'service', '-s', f'/world/{world}/{service}',
           '--reqtype', reqtype, '--reptype', 'ignition.msgs.Boolean',
           '--timeout', str(timeout_ms), '--req', req]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True,
                             timeout=timeout_ms / 1000.0 + 5.0, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    text = (out.stdout + out.stderr).strip()
    return out.returncode == 0 and 'data: true' in text, text
