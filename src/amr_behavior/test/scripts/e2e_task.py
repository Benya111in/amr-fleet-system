#!/usr/bin/env python3
"""
단일 로봇 종단 작업 시험 (amr_behavior).

플릿처럼 assign_task 를 부르고 대기-이동-인식-도킹-적재-이동-도킹-하역-복귀 전 과정을 기록한다.

기록 (JSON lines, 모두 sim 시각 = ground_truth/odom stamp 기준 + wall):
  executor/phase 전이, task_status, payload/attach·mass, perception/detected_objects 중 box (거리),
  safety/zone·estop_active, 도킹 끝 시점 진값 오차 (docking_server 로그 대신 GT: DOCKING → 다음 단계 전이
  시점의 자세를 도크 목표 자세와 비교), assign_task 호출 → 첫 cmd_vel(최종) 까지 지연.

    python3 e2e_task.py --pickup dock_1 --dropoff dock_a --item medium \
        --out log/bhv_trials/e2e.jsonl
"""

import argparse
import importlib.util
import json
import math
import sys
import threading
import time

from amr_msgs.msg import DetectedObjectArray, Task
from amr_msgs.srv import AssignTask
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String, UInt8
import yaml

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def marker_faces(world_gen):
    spec = importlib.util.spec_from_file_location('gen_world', world_gen)
    w = importlib.util.module_from_spec(spec)
    argv = sys.argv
    sys.argv = ['gen']
    spec.loader.exec_module(w)
    sys.argv = argv
    faces = {}
    for name, (_x, y, kind, _m) in w.DOCKS.items():
        faces[name] = ((-w.HALF_X + 0.02, y, 0.0) if kind == 'inbound'
                       else (w.HALF_X - 0.02, y, math.pi))
        faces[name + '_pad'] = (_x, y)
    for name, (x, _m) in w.CHARGERS.items():
        faces[f'charger_{name}'] = (x, w.CHARGER_Y + w.STATION_FRONT + 0.02, math.pi / 2)
    return faces


class E2E(Node):
    def __init__(self, ns, out):
        super().__init__('bhv_e2e')
        self.ns = ns
        self.out = out
        self.lock = threading.Lock()
        self.gt = None
        self.sim = None
        self.phase = None
        self.events = []
        self.status = []
        self.first_cmd_after = None
        self.assign_wall = None
        self.box_seen = []
        self.create_subscription(Odometry, f'/{ns}/ground_truth/odom', self._gt, 20)
        self.create_subscription(String, f'/{ns}/executor/phase', self._phase, LATCHED)
        self.create_subscription(Task, f'/{ns}/task_status', self._task, 10)
        self.create_subscription(String, f'/{ns}/payload/attach', self._attach, LATCHED)
        self.create_subscription(Float32, f'/{ns}/payload/mass', self._mass, LATCHED)
        self.create_subscription(UInt8, f'/{ns}/safety/zone', self._zone, 10)
        self.create_subscription(Bool, f'/{ns}/safety/estop_active', self._estop, LATCHED)
        self.create_subscription(DetectedObjectArray, f'/{ns}/perception/detected_objects',
                                 self._objects, qos_profile_sensor_data)
        self.create_subscription(Twist, f'/{ns}/cmd_vel', self._cmd, 10)
        self.assign = self.create_client(AssignTask, f'/{ns}/assign_task')

    def log(self, kind, **kw):
        with self.lock:
            row = {'kind': kind, 'sim': round(self.sim, 3) if self.sim else None,
                   'wall': round(time.monotonic(), 3)}
            if self.gt:
                row['gt'] = [round(self.gt[0], 4), round(self.gt[1], 4), round(self.gt[2], 5)]
        row.update(kw)
        self.events.append(row)
        self.out.write(json.dumps(row, ensure_ascii=False) + '\n')
        self.out.flush()
        print(json.dumps(row, ensure_ascii=False), flush=True)

    def _gt(self, m):
        p = m.pose.pose
        with self.lock:
            self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))
            self.sim = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _phase(self, m):
        self.phase = m.data
        self.log('phase', value=m.data)

    def _task(self, m):
        self.status.append(m.status)
        self.log('task_status', task_id=m.task_id, status=int(m.status))

    def _attach(self, m):
        self.log('payload_attach', value=m.data)

    def _mass(self, m):
        self.log('payload_mass', value=round(float(m.data), 2))

    def _zone(self, m):
        self.log('zone', value=int(m.data))

    def _estop(self, m):
        self.log('estop_active', value=bool(m.data))

    def _objects(self, m):
        boxes = [o for o in m.objects if o.class_name.lower() == 'box']
        if boxes and self.phase in ('MOVING', 'RECOVERING'):
            d = min(o.distance for o in boxes)
            if not self.box_seen or time.monotonic() - self.box_seen[-1] > 2.0:
                self.log('box_detected', distance=round(float(d), 3),
                         confidence=round(float(max(o.confidence for o in boxes)), 2))
            self.box_seen.append(time.monotonic())

    def _cmd(self, m):
        if self.assign_wall is not None and self.first_cmd_after is None and \
                (abs(m.linear.x) > 0.01 or abs(m.angular.z) > 0.01):
            self.first_cmd_after = time.monotonic()
            latency = (self.first_cmd_after - self.assign_wall) * 1000.0
            self.log('first_cmd_vel', latency_ms=round(latency, 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='amr_01')
    ap.add_argument('--pickup', default='dock_1')
    ap.add_argument('--dropoff', default='dock_a')
    ap.add_argument('--item', default='medium')
    ap.add_argument('--task-id', default='e2e-001')
    ap.add_argument('--out', required=True)
    ap.add_argument('--timeout', type=float, default=5400.0)
    ap.add_argument('--config', default='/ros2_ws/src/amr_behavior/config/behavior.yaml')
    ap.add_argument('--world-gen',
                    default='/ros2_ws/src/amr_simulation/worlds/gen_warehouse_world.py')
    args = ap.parse_args()
    with open(args.config, encoding='utf-8') as f:
        docks = yaml.safe_load(f)['/**']['ros__parameters']['docks']
    faces = marker_faces(args.world_gen)

    rclpy.init()
    out = open(args.out, 'a', encoding='utf-8')
    node = E2E(args.ns, out)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.monotonic()
    while (node.phase != 'IDLE' or node.gt is None) and time.monotonic() - t0 < 600:
        time.sleep(0.5)
    node.log('ready', phase=node.phase)
    if not node.assign.wait_for_service(timeout_sec=120):
        node.log('error', what='assign_task 서비스 없음')
        return
    req = AssignTask.Request()
    task = req.task
    task.task_id = args.task_id
    task.item_type = args.item
    task.priority = 100
    for pose, dock in ((task.pickup_pose, args.pickup), (task.dropoff_pose, args.dropoff)):
        px, py = faces[dock + '_pad']       # 플릿 JSON 처럼 도크 패드 중심 (월드 = map 좌표, C4)
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = float(px), float(py)
        pose.pose.orientation.w = 1.0
    task.header.stamp = node.get_clock().now().to_msg()
    node.assign_wall = time.monotonic()
    fut = node.assign.call_async(req)
    while not fut.done():
        time.sleep(0.01)
    res = fut.result()
    node.log('assign_task', success=res.success, message=res.message,
             call_ms=round((time.monotonic() - node.assign_wall) * 1000.0, 1))
    if not res.success:
        return
    # 도킹 끝 진값: DOCKING → (LOADING|UNLOADING|RECOVERING) 전이 시점 자세 vs 도크 목표 자세
    last_phase = None
    dock_targets = []
    for d in (args.pickup, args.dropoff):
        fx, fy, normal = faces[d]
        st = docks[d]['standoff']
        dock_targets.append((d, fx + st * math.cos(normal), fy + st * math.sin(normal),
                             wrap(normal + math.pi)))
    k = 0
    while time.monotonic() - t0 < args.timeout:
        ph = node.phase
        if last_phase == 'DOCKING' and ph in ('LOADING', 'UNLOADING') and k < len(dock_targets):
            d, tx, ty, tyaw = dock_targets[k]
            k += 1
            with node.lock:
                gx, gy, gyaw = node.gt
            node.log('dock_truth', dock_id=d, pos_mm=round(math.hypot(gx - tx, gy - ty) * 1000, 1),
                     ang_deg=round(abs(math.degrees(wrap(gyaw - tyaw))), 3))
        last_phase = ph
        if node.status and node.status[-1] in (Task.STATUS_COMPLETED, Task.STATUS_FAILED) and \
                ph == 'IDLE':
            break
        time.sleep(0.05)
    node.log('end', phase=node.phase, statuses=[int(s) for s in node.status])
    out.close()
    ex.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
