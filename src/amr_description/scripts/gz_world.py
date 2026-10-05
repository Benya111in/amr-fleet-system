#!/usr/bin/env python3
"""
Gazebo Fortress 월드 조작 도구 — 확인된 스폰과 일시정지 해제.

    gz_world.py spawn   --world warehouse --name amr_01 [--topic /amr_01/robot_description]
                        [--x 0 --y 0 --z 0.02 --yaw 0]
    gz_world.py unpause --world warehouse [--wait-models amr_01,amr_02]
    gz_world.py exists  --world warehouse --name amr_01        # 종료 코드 0 = 모델 있음

spawn.launch.py(로봇 1대)와 amr_simulation 의 unpause.launch.py(월드) 가 이 스크립트를 쓴다.
gz 쪽 호출은 `ign service` CLI 로 한다 (Fortress 의 gz-transport 11 에는 파이썬 바인딩이 없다).
IGN_PARTITION 은 환경에서 그대로 물려받는다 — 서버와 같은 값이어야 서비스가 보인다.

spawn (ros_gz_sim create 대체, 결정)
  1. /world/<w>/create 서비스가 뜰 때까지 기다린다 (--world-timeout, 기본 900 s: 고부하에서 월드 로드가 수 분 걸린 사례).
  2. 같은 이름의 모델이 이미 있으면 실패한다 (allow_renaming 을 쓰지 않는다 — 모델 이름 = 네임스페이스).
  3. robot_description 토픽(transient_local)에서 URDF 를 받아 XML 주석을 뺀 뒤 create 서비스를 부른다 (응답 대기 30 s).
     ros_gz_sim create(0.244.26)는 응답을 5 s 만 기다리고 시간 초과에도 종료 코드 0 을 낸다 (확인) — 쓰지 않는 이유.
  4. create 응답(data: true)은 "요청 접수" 일 뿐이라(UserCommands 가 다음 스텝에 처리) scene/info 에서 모델이 실제로
     생겼는지 확인한다 (--verify-timeout, 기본 120 s). 응답이 시간 초과여도 재요청하지 않는다(중복 생성 방지) — 확인만 한다.
  실패하면 ERROR 를 찍고 종료 코드 1 → spawn.launch.py 가 런치 전체를 내린다 (조용한 실패 금지).

unpause
  --wait-models 의 모델이 모두 생길 때까지 기다린 뒤 /world/<w>/control 에 pause: false 를 보낸다.
  Fortress 센서의 다음 갱신 시각은 sim 0 에서 시작하므로, sim 시간 T 에 스폰된 로봇은 T × 주기 만큼 몰아서 갱신하며
  따라잡는다(폭주). 월드를 일시정지(sim 0)로 띄우고 전 로봇을 스폰·확인한 뒤 해제하면 첫 메시지부터 정상 주기다.
"""

import argparse
import math
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

LOG = "[gz_world]"


def log(msg: str) -> None:
    print(f"{LOG} {msg}", flush=True)


def err(msg: str) -> None:
    print(f"{LOG} ERROR: {msg}", file=sys.stderr, flush=True)


def ign(args: list, timeout: float) -> subprocess.CompletedProcess:
    """`ign` CLI 실행. 시간 초과 시 returncode -1 인 결과를 돌려준다 (예외 대신)."""
    try:
        return subprocess.run(["ign", *args], capture_output=True, text=True, timeout=timeout,
                              check=False)
    except subprocess.TimeoutExpired as e:
        return subprocess.CompletedProcess(e.cmd, -1, e.stdout or "", e.stderr or "")


def service_listed(service: str) -> bool:
    out = ign(["service", "-l"], timeout=15.0).stdout
    return service in out.split()


def wait_service(service: str, timeout: float) -> bool:
    t0 = time.monotonic()
    last_note = t0
    while time.monotonic() - t0 < timeout:
        if service_listed(service):
            return True
        if time.monotonic() - last_note > 30.0:
            log(f"{service} 대기 중 ({time.monotonic() - t0:.0f} s)")
            last_note = time.monotonic()
        time.sleep(1.0)
    return False


def parse_scene_models(text: str) -> set:
    """ignition.msgs.Scene DebugString 에서 최상위 model 블록의 name 만 뽑는다 (light 등 다른 블록 제외)."""
    names = set()
    block = None
    for line in text.splitlines():
        m = re.match(r"^(\w+) \{$", line)
        if m:
            block = m.group(1)
            continue
        if line == "}":
            block = None
            continue
        if block == "model":
            m = re.match(r'^  name: "(.*)"$', line)
            if m:
                names.add(m.group(1))
    return names


def model_names(world: str) -> set:
    """현재 월드의 모델 이름 집합. 서비스 호출이 실패하면 None."""
    r = ign(["service", "-s", f"/world/{world}/scene/info", "--reqtype", "ignition.msgs.Empty",
             "--reptype", "ignition.msgs.Scene", "--timeout", "20000", "--req", ""], timeout=40.0)
    if r.returncode != 0 or "model {" not in r.stdout:
        return None
    return parse_scene_models(r.stdout)


def wait_models(world: str, names: list, timeout: float) -> set:
    """모델 이름들이 모두 생길 때까지 기다린다. 끝까지 없는 이름의 집합을 돌려준다 (빈 집합 = 성공)."""
    t0 = time.monotonic()
    missing = set(names)
    while True:
        present = model_names(world)
        if present is not None:
            missing = set(names) - present
            if not missing:
                return missing
        if time.monotonic() - t0 > timeout:
            return missing
        time.sleep(1.0)


def get_robot_description(topic: str, timeout: float) -> str:
    """robot_state_publisher 가 latched 로 내는 robot_description 을 한 번 받는다."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=None)
    node = rclpy.create_node("gz_world_spawn")
    got = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, topic, lambda m: got.append(m.data), qos)
    t0 = time.monotonic()
    try:
        while not got and time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return got[0] if got else ""


def strip_comments(xml_text: str) -> str:
    """XML 주석 제거 (ElementTree 는 주석을 버린다). 요청 크기를 줄이고 비ASCII 주석을 없앤다."""
    return ET.tostring(ET.fromstring(xml_text), encoding="unicode")


def proto_string(s: str) -> str:
    """Protobuf 텍스트 형식 문자열 리터럴."""
    return '"' + (s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
                  .replace("\r", "\\r").replace("\t", "\\t")) + '"'


def create_request(name: str, sdf: str, x: float, y: float, z: float, yaw: float) -> str:
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return (f"sdf: {proto_string(sdf)} name: {proto_string(name)} allow_renaming: false "
            f"pose {{ position {{ x: {x} y: {y} z: {z} }} "
            f"orientation {{ x: 0 y: 0 z: {qz} w: {qw} }} }}")


def cmd_spawn(a) -> int:
    create = f"/world/{a.world}/create"
    log(f"{a.name}: {create} 대기 (최대 {a.world_timeout:.0f} s)")
    if not wait_service(create, a.world_timeout):
        err(f"{create} 서비스가 {a.world_timeout:.0f} s 안에 뜨지 않았다 (월드 이름/IGN_PARTITION 확인)")
        return 1

    present = model_names(a.world)
    if present is None:
        err(f"/world/{a.world}/scene/info 조회 실패")
        return 1
    if a.name in present:
        err(f"모델 '{a.name}' 이 이미 있다 — 이름(= 네임스페이스)이 겹친다")
        return 1

    topic = a.topic or f"/{a.name}/robot_description"
    urdf = get_robot_description(topic, a.description_timeout)
    if not urdf:
        err(f"{topic} 를 {a.description_timeout:.0f} s 안에 받지 못했다 (description.launch.py 확인)")
        return 1

    req = create_request(a.name, strip_comments(urdf), a.x, a.y, a.z, a.yaw)
    t0 = time.monotonic()
    r = ign(["service", "-s", create, "--reqtype", "ignition.msgs.EntityFactory",
             "--reptype", "ignition.msgs.Boolean", "--timeout", "30000", "--req", req],
            timeout=60.0)
    if "data: true" in r.stdout:
        log(f"{a.name}: create 요청 접수 ({time.monotonic() - t0:.1f} s) → 생성 확인")
    else:
        # 시간 초과여도 서버가 나중에 처리할 수 있으므로 재요청하지 않고 확인 단계로 넘어간다
        log(f"{a.name}: create 응답 없음/거부 (rc={r.returncode}, {r.stdout.strip()!r} "
            f"{r.stderr.strip()!r}) → 생성 여부만 확인")

    missing = wait_models(a.world, [a.name], a.verify_timeout)
    if missing:
        err(f"모델 '{a.name}' 이 {a.verify_timeout:.0f} s 안에 월드에 나타나지 않았다 — 스폰 실패")
        return 1
    log(f"{a.name}: 스폰 확인 (x={a.x}, y={a.y}, z={a.z}, yaw={a.yaw})")
    return 0


def cmd_unpause(a) -> int:
    control = f"/world/{a.world}/control"
    if not wait_service(control, a.world_timeout):
        err(f"{control} 서비스가 {a.world_timeout:.0f} s 안에 뜨지 않았다")
        return 1
    names = [n for n in (a.wait_models or "").split(",") if n]
    if names:
        log(f"모델 대기: {', '.join(names)} (최대 {a.verify_timeout:.0f} s)")
        missing = wait_models(a.world, names, a.verify_timeout)
        if missing:
            err(f"모델이 나타나지 않아 일시정지를 풀지 않는다: {', '.join(sorted(missing))}")
            return 1
    for attempt in range(5):
        r = ign(["service", "-s", control, "--reqtype", "ignition.msgs.WorldControl",
                 "--reptype", "ignition.msgs.Boolean", "--timeout", "5000",
                 "--req", "pause: false"], timeout=20.0)
        if "data: true" in r.stdout:
            log(f"/world/{a.world} 일시정지 해제" + (f" ({len(names)}대 확인 후)" if names else ""))
            return 0
        log(f"일시정지 해제 재시도 {attempt + 1} ({r.stdout.strip()!r})")
        time.sleep(1.0)
    err(f"{control} 호출 실패")
    return 1


def cmd_exists(a) -> int:
    present = model_names(a.world)
    return 0 if present is not None and a.name in present else 1


def main(argv=None) -> int:
    # launch_ros Node 가 붙이는 --ros-args ... 를 걷어낸다
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp):
        sp.add_argument("--world", default="warehouse")
        sp.add_argument("--world-timeout", type=float, default=900.0,
                        help="서비스가 뜰 때까지 기다리는 시간 [s]")
        sp.add_argument("--verify-timeout", type=float, default=120.0,
                        help="모델 생성 확인 대기 [s]")

    sp = sub.add_parser("spawn")
    common(sp)
    sp.add_argument("--name", required=True)
    sp.add_argument("--topic", default="")
    sp.add_argument("--description-timeout", type=float, default=60.0)
    sp.add_argument("--x", type=float, default=0.0)
    sp.add_argument("--y", type=float, default=0.0)
    sp.add_argument("--z", type=float, default=0.02)
    sp.add_argument("--yaw", type=float, default=0.0)
    su = sub.add_parser("unpause")
    common(su)
    su.add_argument("--wait-models", default="", help="쉼표로 구분한 모델 이름")
    se = sub.add_parser("exists")
    common(se)
    se.add_argument("--name", required=True)
    a = p.parse_args(argv)
    return {"spawn": cmd_spawn, "unpause": cmd_unpause, "exists": cmd_exists}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
