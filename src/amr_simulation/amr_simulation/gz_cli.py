"""
Gazebo Fortress 월드 서비스 호출 (`ign service` CLI) — payload_manager_node 의 화물 생성·제거.

Fortress 의 gz-transport 11 에는 파이썬 바인딩이 없어 amr_description scripts/gz_world.py 와 같이 CLI 를 부른다.
IGN_PARTITION 은 환경에서 물려받는다 (서버와 같아야 서비스가 보인다).
UserCommands 의 create/remove 응답 data: true 는 "요청 접수" 일 뿐이다 (다음 스텝에 처리, 이름 충돌 등 실패는 서버 로그에만
남는다) → 결과 확인은 호출 쪽이 한다 (payload_manager_node 는 DetachableJoint state 로 확인).
"""

import subprocess
from typing import Callable, Sequence

Runner = Callable[[list, float], subprocess.CompletedProcess]


def ign(args: list, timeout: float) -> subprocess.CompletedProcess:
    """`ign` CLI 실행. 시간 초과·실행 파일 없음은 예외 대신 returncode -1 로 돌려준다."""
    try:
        return subprocess.run(["ign", *args], capture_output=True, text=True, timeout=timeout,
                              check=False)
    except subprocess.TimeoutExpired as e:
        return subprocess.CompletedProcess(e.cmd, -1, e.stdout or "", e.stderr or "")
    except OSError as e:
        return subprocess.CompletedProcess(["ign", *args], -1, "", str(e))


def proto_string(s: str) -> str:
    """Protobuf 텍스트 형식 문자열 리터럴."""
    return '"' + (s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
                  .replace("\r", "\\r").replace("\t", "\\t")) + '"'


def create_request(name: str, sdf: str, pose: Sequence[float]) -> str:
    """ignition.msgs.EntityFactory 텍스트. pose = (x, y, z, qx, qy, qz, qw)."""
    x, y, z, qx, qy, qz, qw = pose
    return (f"sdf: {proto_string(sdf)} name: {proto_string(name)} allow_renaming: false "
            f"pose {{ position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} "
            f"orientation {{ x: {qx:.9f} y: {qy:.9f} z: {qz:.9f} w: {qw:.9f} }} }}")


def remove_request(name: str) -> str:
    """ignition.msgs.Entity 텍스트 (모델 이름으로 제거)."""
    return f"name: {proto_string(name)} type: MODEL"


def parse_pose_v(text: str) -> dict:
    """
    `ign topic -e` 의 ignition.msgs.Pose_V 텍스트 → {이름: (x, y, z, qx, qy, qz, qw)}.

    proto3 텍스트 형식은 0 인 필드를 생략하므로 없는 값은 0 이다 (w 도 — 회전이 없으면 w: 1 이 찍힌다).
    """
    poses, cur, sub = {}, None, None
    for line in text.splitlines():
        s = line.strip()
        if line == "pose {":
            cur, sub = {"name": "", "position": {}, "orientation": {}}, None
        elif cur is None:
            continue
        elif line == "}":
            if cur["name"]:
                p, q = cur["position"], cur["orientation"]
                poses[cur["name"]] = (p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0),
                                      q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0),
                                      q.get("w", 0.0))
            cur = None
        elif s.startswith("name:") and sub is None:
            cur["name"] = s.split(":", 1)[1].strip().strip('"')
        elif s in ("position {", "orientation {"):
            sub = s.split()[0]
        elif s == "}":
            sub = None
        elif sub is not None and ":" in s:
            key, val = s.split(":", 1)
            cur[sub][key.strip()] = float(val)
    return poses


class GzWorld:
    """한 월드의 create / remove 서비스와 모델 자세 조회."""

    def __init__(self, world: str, runner: Runner = ign, timeout_ms: int = 5000):
        self.world = world
        self.run = runner
        self.timeout_ms = timeout_ms

    def _call(self, service: str, reqtype: str, req: str) -> bool:
        r = self.run(["service", "-s", f"/world/{self.world}/{service}", "--reqtype", reqtype,
                      "--reptype", "ignition.msgs.Boolean", "--timeout", str(self.timeout_ms),
                      "--req", req], self.timeout_ms / 1000.0 + 10.0)
        return r.returncode == 0 and "data: true" in r.stdout

    def create(self, name: str, sdf: str, pose: Sequence[float]) -> bool:
        return self._call("create", "ignition.msgs.EntityFactory", create_request(name, sdf, pose))

    def remove(self, name: str) -> bool:
        return self._call("remove", "ignition.msgs.Entity", remove_request(name))

    def poses(self, timeout: float = 20.0) -> dict:
        """/world/<w>/pose/info 한 메시지 (모델·링크 자세, 월드 좌표). 실패하면 빈 dict."""
        r = self.run(["topic", "-e", "-n", "1", "-t", f"/world/{self.world}/pose/info"], timeout)
        return parse_pose_v(r.stdout) if r.returncode == 0 else {}
