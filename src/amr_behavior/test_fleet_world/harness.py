"""
분산 플릿 launch 시험 공용 도구 (b4-wire).

실제 task_executor_node · fleet_agent · fleet_adapter · fleet_manager(manager_shim) + 가짜 세계
(fake_fleet_world.py) + 가짜 /clock. Nav2 · Gazebo · 인식 · 위치추정은 없다.

판정 원자료는 파일이다: 에이전트 사건 로그(events_<로봇>.jsonl), 실행기 로그(exec_<로봇>.log), 가짜 세계의 실제 자세
(poses.jsonl). 토픽 구독(상태 · task_status · payload/attach)은 진행 판단과 교차 확인에 쓴다.
"""

import collections
import json
import math
import os
import pathlib
import re
import signal
import subprocess
import sys
import tempfile
import time

from amr_msgs.msg import AgentState, Task
from amr_msgs.srv import AssignTask
from launch.actions import ExecuteProcess
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

HERE = pathlib.Path(__file__).resolve().parent
PKG = HERE.parent                                   # src/amr_behavior
REPO_SRC = PKG.parent                               # src
FLEET_CFG = REPO_SRC / 'amr_fleet' / 'config'
ZONES = FLEET_CFG / 'traffic_zones.yaml'
TRAFFIC = FLEET_CFG / 'traffic.yaml'
DIST_XML = PKG / 'behavior_trees' / 'task_executor_distributed.xml'
BEHAVIOR_YAML = PKG / 'config' / 'behavior.yaml'
EXE = os.environ.get('TASK_EXECUTOR_EXE', '')
BODY_R = 0.36
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
ATTACH_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
BOARD_QOS = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def raw_dir(env: str, prefix: str) -> pathlib.Path:
    d = pathlib.Path(os.environ[env]) if os.environ.get(env) else pathlib.Path(
        tempfile.mkdtemp(prefix=prefix))
    d.mkdir(parents=True, exist_ok=True)
    return d


def comm_yaml(raw: pathlib.Path, seed: int, loss: float, partitions: str = '') -> pathlib.Path:
    path = raw / 'comm.yaml'
    text = (f'seed: {seed}\nlinks:\n'
            f'  - {{src: "amr_*", dst: "amr_*", comm_latency_ms: [0.0, 100.0], '
            f'drop_rate: {loss}}}\n'
            '  - {src: "fleet", dst: "*", comm_latency_ms: [0.0, 100.0], drop_rate: 0.0}\n')
    if partitions:
        text += 'partitions:\n' + partitions
    path.write_text(text, encoding='utf-8')
    return path


def processes(raw: pathlib.Path, robots, poses, comm: pathlib.Path, seed: int,
              sim_start: float = 100.0, agent_params=(), with_adapters: bool = True):
    """가짜 /clock · 세계 + 로봇마다 실행기 · 에이전트 (· 어댑터) — launch ExecuteProcess 목록."""
    (raw / 'store').mkdir(exist_ok=True)
    py = sys.executable
    env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM='0', RCUTILS_LOGGING_USE_STDOUT='1',
               PYTHONUNBUFFERED='1')
    out = [
        ExecuteProcess(cmd=[py, '-m', 'amr_fleet.agent.fake_clock', '--ros-args',
                            '-p', f'start_s:={sim_start}'], name='fake_clock', output='log'),
        ExecuteProcess(cmd=['bash', '-c', f'exec {py} {HERE / "fake_fleet_world.py"} '
                            f'--robots {",".join(robots)} '
                            f'--poses="{";".join(",".join(str(v) for v in p) for p in poses)}" '
                            f'--out {raw} --ros-args -p use_sim_time:=true '
                            f'> {raw}/world.log 2>&1'], name='world', output='log', env=env)]
    for r, (x, y, yaw) in zip(robots, poses):
        override = raw / f'exec_{r}.yaml'
        override.write_text(
            '/**/task_executor_node:\n  ros__parameters:\n'
            f'    robot_id: {r}\n    bt_xml: {DIST_XML}\n    fleet:\n      distributed: true\n'
            '    groot:\n      enabled: false\n    charger_claims_topic: ""\n'
            '    allow_undocked_tasks: true\n    battery_low_percent: -1.0\n'
            f'    waiting_pose: [{x}, {y}, {yaw}]\n    use_sim_time: true\n', encoding='utf-8')
        out.append(ExecuteProcess(
            cmd=['bash', '-c', f'exec {EXE} --ros-args -r __ns:=/{r} '
                 f'-r assign_task:=executor/assign_task --params-file {BEHAVIOR_YAML} '
                 f'--params-file {override} > {raw}/exec_{r}.log 2>&1'],
            name=f'exec_{r}', output='log', env=env))
        cmd = [py, '-m', 'amr_fleet.agent.node', '--ros-args', '-r', f'__ns:=/{r}',
               '-p', 'use_sim_time:=true', '-p', f'store_dir:={raw / "store"}',
               '-p', f'event_log:={raw}/events_{r}.jsonl', '-p', 'modes.resync_s:=3.0',
               '-p', 'proxy_assign_task:=true', '-p', 'path_gate.enabled:=true',
               '-p', f'zones_file:={ZONES}', '-p', f'traffic_params_file:={TRAFFIC}',
               '-p', 'release_gate:=zone_map', '-p', 'simulate_latency:=true',
               '-p', f'comm_model_file:={comm}', '-p', f'seed:={seed}',
               '-p', 'task_hold_s:=60.0', '-p', 'vote.tentative_s:=15.0']
        for kv in agent_params:
            cmd += ['-p', kv]
        out.append(ExecuteProcess(
            # exec — 없으면 launch 의 SIGINT/SIGTERM/SIGKILL 이 bash 에만 가고 에이전트는 고아로 남아 다음 실행
            # (같은 도메인)에 섞인다 (W-launch 판정 실행 1 회차에서 발견 — 결과 뒤 변경, 노트에 적음)
            cmd=['bash', '-c', 'exec ' + ' '.join(cmd) + f' > {raw}/agent_{r}.log 2>&1'],
            name=f'agent_{r}', output='log', env=env))
        if with_adapters:
            out.append(ExecuteProcess(
                cmd=[py, '-m', 'amr_fleet.fleet_adapter_node', '--ros-args', '-r', f'__ns:=/{r}',
                     '-p', f'robot_id:={r}', '-p', 'use_sim_time:=true',
                     '-p', 'simulate_latency:=false', '-p', f'log_dir:={raw / "adapter"}'],
                name=f'adapter_{r}', output='log'))
    return out


class Observer:
    """시험 프로세스의 관찰 · 조작 노드."""

    def __init__(self, raw: pathlib.Path, robots):
        self.raw = raw
        self.robots = list(robots)
        self.node = rclpy.create_node('fleet_wire_observer')
        self.state = {}
        self.status = []                       # (wall, robot, task_id, status)
        self.attach = []                       # (wall, robot, item)
        self.assign = {}
        self.servers = {}
        for r in self.robots:
            self.node.create_subscription(AgentState, f'/{r}/fleet_agent/state',
                                          lambda m, r=r: self.state.__setitem__(r, m), LATCHED)
            self.node.create_subscription(
                Task, f'/{r}/task_status',
                lambda m, r=r: self.status.append((time.time(), r, m.task_id, m.status)), 10)
            self.node.create_subscription(
                String, f'/{r}/payload/attach',
                lambda m, r=r: self.attach.append((time.time(), r, m.data)), ATTACH_QOS)
            self.assign[r] = self.node.create_client(AssignTask, f'/{r}/assign_task')
        self.request = self.node.create_publisher(String, '/fleet/task_request', 50)
        self.board = self.node.create_publisher(Task, '/fleet/task_board', BOARD_QOS)

    # ------------------------------------------------------------------ 진행
    def spin_for(self, seconds, until=None, step=None) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if step is not None:
                step()
            if until is not None and until():
                return True
        return until is None

    def call(self, client, req, timeout=8.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    def modes(self):
        return {r: (self.state[r].mode if r in self.state else None) for r in self.robots}

    def all_mode(self, code) -> bool:
        return all(m == code for m in self.modes().values())

    # ------------------------------------------------------------------ 작업
    def submit(self, task_id, robot, pickup, dropoff, item='small'):
        """중앙 작업 (JSON /fleet/task_request, 고정 로봇)."""
        doc = {'task_id': task_id, 'robot_id': robot, 'item_type': item,
               'pickup': {'x': pickup[0], 'y': pickup[1], 'yaw': pickup[2]},
               'dropoff': {'x': dropoff[0], 'y': dropoff[1], 'yaw': dropoff[2]}}
        self.request.publish(String(data=json.dumps(doc)))

    def proxy_assign(self, r, task_id, epoch, leader, pickup, dropoff, item='small'):
        """에이전트 프록시 assign_task 직접 호출 — (응답 성공, 메시지)."""
        req = AssignTask.Request()
        t = req.task
        t.task_id, t.robot_id, t.item_type = task_id, r, item
        t.epoch, t.leader_id = int(epoch), leader
        t.header.stamp = self.node.get_clock().now().to_msg()   # 실행기가 수락을 로그에 남긴다
        for pose, (x, y, yaw) in ((t.pickup_pose, pickup), (t.dropoff_pose, dropoff)):
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        resp = self.call(self.assign[r], req)
        return (None, None) if resp is None else (bool(resp.success), str(resp.message))

    def post(self, task_id, pickup, dropoff, item='small', priority=0):
        """작업 게시판 게시 (협상 대상)."""
        t = Task()
        t.task_id, t.item_type, t.priority, t.status = task_id, item, priority, Task.STATUS_PENDING
        for pose, (x, y, yaw) in ((t.pickup_pose, pickup), (t.dropoff_pose, dropoff)):
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.board.publish(t)

    def statuses(self, task_id):
        return [(r, s) for _, r, t, s in self.status if t == task_id]

    def terminal(self, task_id):
        return [(r, s) for r, s in self.statuses(task_id)
                if s in (Task.STATUS_COMPLETED, Task.STATUS_FAILED)]

    # ------------------------------------------------------------------ 서버 (실제 관리자 + 하트비트)
    def server(self, name, *args):
        log = open(self.raw / f'{name}.log', 'w')
        p = subprocess.Popen([sys.executable, '-m', 'amr_fleet.agent.manager_shim',
                              '--log-dir', str(self.raw / name), *args,
                              '--ros-args', '-p', 'use_sim_time:=true'],
                             stdout=log, stderr=subprocess.STDOUT)
        self.servers[name] = p
        return p

    def kill(self, name) -> float:
        p = self.servers[name]
        t = time.time()
        os.kill(p.pid, signal.SIGKILL)
        p.wait(timeout=10)
        return t

    def endpoints(self):
        out = {}
        for topic in ('/fleet_p2p/vote', '/fleet_p2p/beacon', '/fleet_p2p/neg'):
            pubs = self.node.get_publishers_info_by_topic(topic)
            subs = self.node.get_subscriptions_info_by_topic(topic)
            out[topic] = (sum(1 for e in pubs if e.node_name == 'fleet_agent'),
                          sum(1 for e in subs if e.node_name == 'fleet_agent'))
        return out

    def duplicates(self):
        """같은 이름 · 이름공간 노드가 둘 이상 — 앞선 실행의 고아 프로세스 등이 섞였다 (격리 실패)."""
        c = collections.Counter(self.node.get_node_names_and_namespaces())
        return {f'{ns.rstrip("/")}/{n}': k for (n, ns), k in c.items() if k > 1}

    def close(self):
        for p in self.servers.values():
            if p.poll() is None:
                p.kill()
                p.wait(timeout=10)
        self.node.destroy_node()


# ---------------------------------------------------------------------- 원자료 분석
def file_events(raw: pathlib.Path):
    out = []
    for f in sorted(raw.glob('events_*.jsonl')):
        for line in f.read_text(encoding='utf-8').splitlines():
            if line.strip():
                out.append(json.loads(line))
    return out


def intervals(events, resource):
    """로봇별 허가 구간 [(t_start, t_end, robot)] — 에이전트 'held' 사건의 sim 시각."""
    out, open_ = [], {}
    for e in sorted((e for e in events if e['event'] == 'held'), key=lambda e: e['t']):
        r = e['robot']
        if resource in e['added']:
            open_[r] = e['t']
        if resource in e['removed'] and r in open_:
            out.append((open_.pop(r), e['t'], r))
    for r, t in open_.items():
        out.append((t, float('inf'), r))
    return sorted(out)


def overlaps(iv):
    bad = []
    for i, (s1, e1, r1) in enumerate(iv):
        for s2, e2, r2 in iv[i + 1:]:
            if r1 != r2 and s2 < e1:
                bad.append(((s1, e1, r1), (s2, e2, r2)))
    return bad


def zone_polygons():
    from amr_fleet.traffic_zones import load_layout
    zones, _ = load_layout(str(ZONES), True)
    return {z.zone_id: z.polygon for z in zones if z.polygon is not None}


def _disc_overlaps(x, y, r, poly) -> bool:
    from amr_fleet.agent.core import disc_clear_of_polygon
    return not disc_clear_of_polygon(x, y, r, poly)


def _center_in(x, y, poly) -> bool:
    from amr_fleet.traffic_geometry import points_in_polygon
    import numpy as np
    return bool(points_in_polygon(np.array([[x, y]]), np.asarray(poly))[0])


def true_cooccupancy(raw: pathlib.Path, polys, robots):
    """
    가짜 세계의 실제 자세로 센 구역 동시 점유 — (원판 기준 사건, 중심 기준 사건, 구역별 점유 표본 수).

    원판 = 외접 반경 0.36 m 원판이 구역 다각형과 겹침 (보수적), 중심 = 중심점이 다각형 안. 사건 = 두 대 이상이 같은
    구역을 같은 표본 시각(sim 10 Hz)에 점유한 연속 구간.
    """
    disc, center, occ = [], [], collections.Counter()
    last_disc, last_center = {}, {}
    for line in (raw / 'poses.jsonl').read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        rec = json.loads(line)
        t, p = rec['t'], rec['p']
        for z, poly in polys.items():
            ind = sorted(r for r in robots
                         if r in p and _disc_overlaps(p[r][0], p[r][1], BODY_R, poly))
            inc = sorted(r for r in robots if r in p and _center_in(p[r][0], p[r][1], poly))
            if ind:
                occ[z] += 1
            for kind, who, last, sink in (('disc', ind, last_disc, disc),
                                          ('center', inc, last_center, center)):
                if len(who) >= 2:
                    if last.get(z) != tuple(who):
                        sink.append({'t': t, 'zone': z, 'robots': who})
                    last[z] = tuple(who)
                else:
                    last.pop(z, None)
    return disc, center, dict(occ)


LOG_RE = {
    'accept': re.compile(r'assign_task 수락 (\S+)'),
    'reject': re.compile(r'assign_task 거절 (\S+): (.*)$'),
    'failed': re.compile(r'작업 실패: (\S+) \(([^)]*)\)'),
    'done': re.compile(r'작업 완료: (\S+)'),
}


def exec_log(raw: pathlib.Path, r):
    """실행기 로그에서 (종류, 작업, 사유) 목록."""
    path = raw / f'exec_{r}.log'
    out = []
    if not path.exists():
        return out
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        for kind, rx in LOG_RE.items():
            m = rx.search(line)
            if m:
                out.append((kind, m.group(1), m.group(2) if m.lastindex and m.lastindex > 1
                            else ''))
    return out


def loads(obs: Observer, raw: pathlib.Path):
    """적재(payload/attach 비어 있지 않음)를 그 로봇의 그때 작업에 붙인다 — (로봇, 작업, 벽시계)."""
    current = {}
    events = sorted([(t, 's', r, tid, s) for t, r, tid, s in obs.status]
                    + [(t, 'a', r, item, None) for t, r, item in obs.attach])
    out = []
    for t, kind, r, x, s in events:
        if kind == 's':
            if s == Task.STATUS_IN_PROGRESS:
                current[r] = x
            elif current.get(r) == x:
                current.pop(r, None)
        elif x:
            out.append((r, current.get(r, '?'), t))
    return out
