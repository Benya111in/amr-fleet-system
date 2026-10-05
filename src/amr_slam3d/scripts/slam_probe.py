#!/usr/bin/env python3
"""
매핑 행 재생 측정 노드 — 입력·처리 수(누락 판정), TF 트리, 온라인 궤적, 2D 격자를 모은다.

    slam_probe.py --row rtabmap3d --out /out/rtabmap3d [--idle-exit 30]

rows.py 의 행 정의를 쓴다 (rclpy 외에 amr_slam3d 순수 모듈만 필요 — liosam 이미지에서는 PYTHONPATH 로 소스를 준다).
- 입력: row.input_topic 을 직렬화 그대로 받아 header.stamp 만 읽는다 (큰 점군을 역직렬화하지 않는다).
- 처리: row.processed 토픽 — odom 형은 역직렬화해 lost(공분산 ≥ 9999 = RTAB-Map null odom)를 센다.
- TF: /tf·/tf_static 간선을 모아 tftree.check (부모 하나·뿌리 map). 같은 메시지를 자체 tf2 버퍼에 넣고
  입력 스탬프마다 map → base_footprint 를 조회해 온라인 궤적(TUM, 월드 = map)을 쓴다 — 조회는 그 스탬프가
  가장 최근 입력 스탬프보다 --lookup-delay 만큼 지난 뒤 한다 (map→odom 류 TF 가 늦게 나오므로).
- 부하: 이 노드는 /clock 을 구독하지 않는다(use_sim_time 끔 — 타이머는 벽시계, "지금" = 최근 입력 스탬프).
  TransformListener 도 두지 않는다 (/tf 를 두 번 받지 않게). rclpy spin_once 는 콜백 하나씩 처리하므로
  /clock 500 Hz + /tf 이중 구독이면 뒤처져 KEEP_LAST 큐가 넘친다 [측정: rtabmap3d 첫 재생에서 icp_odometry
  가 1 784 장을 처리했는데(로그 "Odom:") 이 노드는 icp_odom 1 619 개만 받았다]. 그래서 처리 수의 기준은
  replay_row.sh 가 C++ 기록기로 남기는 out_bag 이고 (bag_tools.py match), 이 노드의 수는 참고값이다.
- 지도: row.map_topic 의 마지막 OccupancyGrid → 자기 프레임 그대로(npz) + 월드 정렬 PGM/YAML (grid.to_world).
- 끝: SIGINT/SIGTERM 또는 첫 입력 뒤 --idle-exit 초 동안 입력이 없으면 probe.json 을 쓰고 끝낸다.
"""

import argparse
import json
import os
import signal
import sys
import time

from amr_slam3d import bagstat, grid, rows, tftree
from amr_slam3d.se2 import yaw_from_quat
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

KINDS = {'cloud': PointCloud2, 'scan': LaserScan, 'odom': Odometry}
RELIABLE = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST)
BEST_EFFORT = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
STATIC = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


class Probe(Node):
    """행 하나의 재생 측정."""

    def __init__(self, ns):
        super().__init__('slam_probe',
                         parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.ns = ns
        self.row = rows.row(ns.row)
        self.world, self.base = ns.world_frame, ns.base_frame
        self.input_stamps = []
        self.proc = {t: {'stamps': [], 'lost': 0} for t, _, _ in self.row.processed}
        self.edges = {}                       # (부모, 자식) → 'static' | 'dynamic'
        self.pending = []
        self.traj = []
        self.fail = {}
        self.grid_msg = None
        self.last_input_wall = None
        self.t_start = time.monotonic()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=600.0))
        self.latest_input = None
        kind = KINDS[self.row.input_kind]
        self.create_subscription(kind, self.row.input_topic, self._on_input, RELIABLE, raw=True)
        for topic, k, q in self.row.processed:
            qos = RELIABLE if q == 'reliable' else BEST_EFFORT
            if k == 'odom':
                self.create_subscription(Odometry, topic, self._odom_cb(topic), qos)
            else:
                self.create_subscription(PointCloud2, topic, self._raw_cb(topic), qos, raw=True)
        self.create_subscription(TFMessage, '/tf', self._tf_cb('dynamic'), RELIABLE)
        self.create_subscription(TFMessage, '/tf_static', self._tf_cb('static'), STATIC)
        if self.row.map_topic:
            self.create_subscription(OccupancyGrid, self.row.map_topic, self._on_grid, LATCHED)
        self.create_timer(0.1, self._lookup_pending)

    # ---- 콜백 ----
    def _on_input(self, raw: bytes) -> None:
        stamp = bagstat.cdr_header_stamp(raw)
        self.input_stamps.append(stamp)
        self.latest_input = stamp if self.latest_input is None else max(self.latest_input, stamp)
        self.pending.append(stamp)
        self.last_input_wall = time.monotonic()

    def _odom_cb(self, topic):
        def cb(msg: Odometry) -> None:
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rec = self.proc[topic]
            rec['stamps'].append(st)
            if msg.pose.covariance[0] >= 9999.0:
                rec['lost'] += 1
        return cb

    def _raw_cb(self, topic):
        def cb(raw: bytes) -> None:
            self.proc[topic]['stamps'].append(bagstat.cdr_header_stamp(raw))
        return cb

    def _tf_cb(self, kind):
        def cb(msg: TFMessage) -> None:
            for tr in msg.transforms:
                key = (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))
                self.edges.setdefault(key, kind)
                try:
                    if kind == 'static':
                        self.tf_buffer.set_transform_static(tr, 'probe')
                    else:
                        self.tf_buffer.set_transform(tr, 'probe')
                except Exception:  # noqa: B902 — 잘못된 TF 하나 때문에 측정을 멈추지 않는다
                    self.fail['set_transform'] = self.fail.get('set_transform', 0) + 1
        return cb

    def _on_grid(self, msg: OccupancyGrid) -> None:
        self.grid_msg = msg

    # ---- 궤적 ----
    def _lookup_pending(self) -> None:
        if self.latest_input is None:
            return
        now = self.latest_input
        keep = []
        for st in self.pending:
            if st > now - self.ns.lookup_delay:
                keep.append(st)
                continue
            t = Time(seconds=st)
            try:
                tr = self.tf_buffer.lookup_transform(self.world, self.base, t)
            except Exception as exc:  # noqa: B902 — tf2 예외 종류를 이유로 남긴다
                if st > now - self.ns.lookup_delay - self.ns.lookup_give_up:
                    keep.append(st)
                else:
                    name = type(exc).__name__
                    self.fail[name] = self.fail.get(name, 0) + 1
                continue
            p, q = tr.transform.translation, tr.transform.rotation
            self.traj.append((st, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        self.pending = keep

    def idle(self) -> bool:
        return (self.last_input_wall is not None
                and time.monotonic() - self.last_input_wall > self.ns.idle_exit)

    # ---- 결과 ----
    def flush(self) -> dict:
        # 남은 스탬프는 지연 없이 한 번 더 시도한다 (재생이 끝나 시각이 멈춘 뒤)
        self.ns.lookup_delay, self.ns.lookup_give_up = 0.0, 0.0
        self._lookup_pending()
        out = self.ns.out
        os.makedirs(out, exist_ok=True)
        self.traj.sort()
        np.savez_compressed(os.path.join(out, 'stamps.npz'),
                            input=np.asarray(self.input_stamps),
                            **{t.strip('/').replace('/', '__'): np.asarray(r['stamps'])
                               for t, r in self.proc.items()})
        with open(os.path.join(out, 'traj_online.tum'), 'w', encoding='utf-8') as f:
            for r in self.traj:
                f.write(' '.join(f'{v:.9f}' if i == 0 else f'{v:.6f}' for i, v in enumerate(r))
                        + '\n')
        summary = {
            'row': self.row.name, 'input_topic': self.row.input_topic,
            'input': bagstat.interval_stats(self.input_stamps, 10.0),
            'processed': {}, 'wall_s': time.monotonic() - self.t_start,
            'traj_online': {'n': len(self.traj), 'lookup_fail': self.fail,
                            'world': self.world, 'base': self.base},
        }
        for topic, rec in self.proc.items():
            summary['processed'][topic] = {
                'stats': bagstat.interval_stats(rec['stamps']), 'lost': rec['lost'],
                'match': bagstat.matched(self.input_stamps, rec['stamps'])}
        frames = [self.base] + [c for _, c, _ in self.row.anchors]
        summary['tf'] = tftree.check(self.edges.keys(), frames, root=self.world)
        summary['tf']['edges'] = sorted(f'{p} -> {c} ({k})' for (p, c), k in self.edges.items())
        summary['anchors'] = {}
        for parent, child, _ in self.row.anchors:
            for frame in {parent, child}:
                if frame == self.world:
                    continue
                try:
                    tr = self.tf_buffer.lookup_transform(self.world, frame, Time())
                    p, q = tr.transform.translation, tr.transform.rotation
                    summary['anchors'][frame] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                except Exception as exc:  # noqa: B902
                    summary['anchors'][frame] = f'lookup 실패: {type(exc).__name__}'
        summary['map'] = self._save_grid(out)
        with open(os.path.join(out, 'probe.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f, ensure_ascii=False, indent=1)
        return summary

    def _save_grid(self, out: str) -> dict:
        msg = self.grid_msg
        if msg is None:
            return {'received': False}
        info = msg.info
        o = info.origin
        g = grid.from_msg_fields(info.width, info.height, info.resolution,
                                 (o.position.x, o.position.y,
                                  yaw_from_quat(o.orientation.x, o.orientation.y,
                                                o.orientation.z, o.orientation.w)),
                                 np.asarray(msg.data, dtype=np.int16))
        np.savez_compressed(os.path.join(out, 'map_raw.npz'), data=g.data,
                            resolution=g.resolution, origin=np.asarray(g.origin),
                            frame=msg.header.frame_id)
        res = {'received': True, 'frame': msg.header.frame_id, 'width': info.width,
               'height': info.height, 'resolution': info.resolution,
               'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}
        try:
            tr = self.tf_buffer.lookup_transform(self.world, msg.header.frame_id, Time())
        except Exception as exc:  # noqa: B902
            res['world'] = f'lookup 실패: {type(exc).__name__}'
            return res
        p, q = tr.transform.translation, tr.transform.rotation
        anchor = (p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        gw = grid.to_world(g, anchor)
        pgm, yml = grid.write_map(os.path.join(out, 'map_world'), gw,
                                  comment=f'{self.row.name}: {msg.header.frame_id} → {self.world} '
                                          f'anchor {anchor}')
        res.update({'world': yml, 'anchor': anchor, 'world_origin': list(gw.origin),
                    'world_shape': list(gw.shape),
                    'n_occupied': int(np.count_nonzero(gw.data >= 65)),
                    'n_free': int(np.count_nonzero((gw.data >= 0) & (gw.data <= 19)))})
        return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--row', required=True, choices=sorted(rows.ROWS))
    ap.add_argument('--out', required=True)
    ap.add_argument('--world-frame', default='map')
    ap.add_argument('--base-frame', default=f'{rows.ROBOT}/base_footprint')
    ap.add_argument('--lookup-delay', type=float, default=1.0, help='[s, sim]')
    ap.add_argument('--lookup-give-up', type=float, default=5.0, help='[s, sim]')
    ap.add_argument('--idle-exit', type=float, default=0.0, help='[s, wall] 0 = 신호로만 끝')
    ns, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Probe(ns)
    # 전용 실행기: rclpy.spin_once(node) 는 호출마다 전역 실행기에 노드를 붙였다 떼어 느리다
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop = {'flag': False}

    def on_signal(signum, _frame):
        stop['flag'] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        while rclpy.ok() and not stop['flag']:
            executor.spin_once(timeout_sec=0.1)
            if ns.idle_exit > 0 and node.idle():
                node.get_logger().info('입력 없음 — 종료')
                break
        for _ in range(20):                    # 남은 콜백(지도 등) 비우기
            executor.spin_once(timeout_sec=0.05)
        s = node.flush()
        node.get_logger().info(json.dumps({
            'input': s['input'].get('count'), 'traj': s['traj_online']['n'],
            'tf_valid': s['tf']['valid'], 'map': s['map'].get('received')}, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
