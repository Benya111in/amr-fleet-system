#!/usr/bin/env python3
r"""
3D LiDAR 부하 측정 프로브 (보고서 2단계): 정해진 시간 동안 RTF, 토픽 주기, GPU, loadavg, 프로세스 CPU 를 잰다.

    lidar3d_probe.py --duration 300 --out probe.json \\
        --topic /amr_01/lidar3d/points --topic /amr_01/scan --topic /amr_01/imu/data_raw \\
        --proc 'gazebo' --proc 'parameter_bridge'

- 구독은 raw(직렬화 바이트)로 받아 헤더 스탬프만 읽는다 — 1 MB 점군을 역직렬화하는 비용을 프로브가 더하지 않는다.
  QoS 는 reliable depth 10 (reliable·best-effort 발행자 모두에 붙는다).
  --topic 으로 3D 점군을 구독하는 것이 곧 "브리지 켬" 이다 (전처리·브리지가 lazy — 구독자가 생겨야 렌더한다).
- RTF = Δ(/clock 시뮬레이션 시각) / Δ(벽시계), 측정 창 전체와 10 s 창별 (최소·중앙값).
- 토픽 주기: 헤더 스탬프(시뮬레이션 시각) 기준 hz 와 수신 벽시계 기준 hz 를 함께 낸다.
- GPU (nvidia-smi 전체 — 공유 GPU 라 다른 작업이 섞인다), /proc/loadavg, --proc 패턴에 맞는 프로세스의 CPU 사용
  (코어 수 = CPU 초 / 벽시계 초, 컨테이너 PID 공간 안).
공유 서버에서 잰 값은 "공유 부하 하 예비값" 이다 — 결과에 loadavg 를 같이 남긴다.
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time

from amr_lidar3d.cloud_io import cdr_stamp, interval_stats
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, PointCloud2

TYPES = {'PointCloud2': PointCloud2, 'LaserScan': LaserScan, 'Imu': Imu}


def guess_type(topic: str) -> str:
    if 'points' in topic:
        return 'PointCloud2'
    if 'imu' in topic:
        return 'Imu'
    return 'LaserScan'


def gpu_sample():
    try:
        out = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used',
                              '--format=csv,noheader,nounits'], capture_output=True, text=True,
                             timeout=10, check=False).stdout.strip().splitlines()
        util, mem = (float(v) for v in out[0].split(','))
        return util, mem
    except Exception:   # nvidia-smi 없음·실패 → 기록 안 함
        return None


def proc_cpu(patterns):
    """{패턴: (CPU 초 합, 프로세스 수)} — /proc/<pid>/cmdline 에 패턴이 있는 프로세스 (자기 자신 제외)."""
    tick = os.sysconf('SC_CLK_TCK')
    out = {p: [0.0, 0] for p in patterns}
    me = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
            with open(f'/proc/{pid}/stat') as f:
                fields = f.read().rsplit(')', 1)[1].split()
        except OSError:
            continue
        if 'lidar3d_probe' in cmd:
            continue
        cpu = (int(fields[11]) + int(fields[12])) / tick
        for p in patterns:
            if p in cmd:
                out[p][0] += cpu
                out[p][1] += 1
    return out


class Probe(Node):
    """토픽 수신 시각·스탬프와 /clock 을 모은다."""

    def __init__(self, topics):
        super().__init__('lidar3d_probe')
        self.lock = threading.Lock()
        self.recording = False
        self.recv = {t: [] for t in topics}
        self.stamp = {t: [] for t in topics}
        self.sizes = {t: [] for t in topics}
        self.clock = []      # (벽시계, 시뮬레이션)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.subs = [self.create_subscription(TYPES[guess_type(t)], t, self._cb(t), qos, raw=True)
                     for t in topics]
        self.create_subscription(Clock, '/clock', self._on_clock, QoSProfile(depth=50), raw=True)

    def _cb(self, topic):
        def cb(raw):
            now = time.monotonic()
            if not self.recording:
                return
            with self.lock:
                self.recv[topic].append(now)
                self.stamp[topic].append(cdr_stamp(raw))
                self.sizes[topic].append(len(raw))
        return cb

    def _on_clock(self, raw):
        if self.recording:
            with self.lock:
                self.clock.append((time.monotonic(), cdr_stamp(raw)))


def rtf_windows(clock, window=10.0):
    if len(clock) < 2:
        return []
    out, start = [], 0
    for i in range(1, len(clock)):
        if clock[i][0] - clock[start][0] >= window:
            dw = clock[i][0] - clock[start][0]
            out.append((clock[i][1] - clock[start][1]) / dw)
            start = i
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--duration', type=float, default=300.0, help='측정 창 [s, 벽시계]')
    ap.add_argument('--settle', type=float, default=20.0,
                    help='구독 후 측정 시작 전 대기 [s] (lazy 체인이 켜지는 시간)')
    ap.add_argument('--topic', action='append', default=[])
    ap.add_argument('--proc', action='append', default=[])
    ap.add_argument('--interval', type=float, default=5.0, help='GPU·loadavg 표본 간격 [s]')
    ap.add_argument('--label', default='')
    ap.add_argument('--out', required=True)
    ns = ap.parse_args(argv)

    rclpy.init()
    node = Probe(ns.topic)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    time.sleep(ns.settle)

    gpu, load = [], []
    cpu0 = proc_cpu(ns.proc)
    node.recording = True
    t0 = time.monotonic()
    while time.monotonic() - t0 < ns.duration:
        g = gpu_sample()
        if g:
            gpu.append(g)
        with open('/proc/loadavg') as f:
            load.append(float(f.read().split()[0]))
        time.sleep(ns.interval)
    node.recording = False
    wall = time.monotonic() - t0
    cpu1 = proc_cpu(ns.proc)

    with node.lock:
        clock = list(node.clock)
        result = {
            'label': ns.label, 'duration_wall_s': wall, 'settle_s': ns.settle,
            'started': time.strftime('%Y-%m-%d %H:%M:%S'),
            'rtf': None, 'rtf_10s': {}, 'topics': {}, 'gpu': {}, 'loadavg': {}, 'proc_cores': {},
        }
        if len(clock) >= 2:
            result['rtf'] = (clock[-1][1] - clock[0][1]) / (clock[-1][0] - clock[0][0])
            win = sorted(rtf_windows(clock))
            if win:
                result['rtf_10s'] = {'n': len(win), 'min': win[0], 'median': win[len(win) // 2],
                                     'max': win[-1]}
        for t in ns.topic:
            sim = interval_stats(node.stamp[t])
            wall_s = interval_stats(node.recv[t])
            sizes = node.sizes[t]
            result['topics'][t] = {
                'n': len(node.recv[t]), 'hz_sim': sim.get('hz'), 'hz_wall': wall_s.get('hz'),
                'dt_sim_median': sim.get('dt_median'), 'dt_sim_p95': sim.get('dt_p95'),
                'dt_sim_max': sim.get('dt_max'),
                'bytes_mean': (sum(sizes) / len(sizes)) if sizes else None}
    if gpu:
        util = sorted(u for u, _ in gpu)
        mem = sorted(m for _, m in gpu)
        result['gpu'] = {'n': len(gpu), 'util_median': util[len(util) // 2], 'util_max': util[-1],
                         'mem_median_mib': mem[len(mem) // 2], 'mem_max_mib': mem[-1]}
    if load:
        s = sorted(load)
        result['loadavg'] = {'n': len(s), 'min': s[0], 'median': s[len(s) // 2], 'max': s[-1]}
    for p in ns.proc:
        result['proc_cores'][p] = {'cores': (cpu1[p][0] - cpu0[p][0]) / wall,
                                   'procs': cpu1[p][1]}
    with open(ns.out, 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(json.dumps({'label': ns.label, 'rtf': result['rtf'],
                      'hz_sim': {t: v['hz_sim'] for t, v in result['topics'].items()},
                      'loadavg': result['loadavg'].get('median')}, ensure_ascii=False))
    # spin 스레드가 도는 중에 rclpy.shutdown() 을 부르면 "terminate called without an active exception" 으로
    # 프로세스가 죽는다 (s1_load_L0_a, rc 134 — 결과 파일은 이미 쓴 뒤). 결과를 다 쓴 뒤 바로 끝낸다
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
