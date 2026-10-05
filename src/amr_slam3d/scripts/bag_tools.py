#!/usr/bin/env python3
"""
bag·지도 파일 도구 — 파일럿 bag 메타데이터, 정답 궤적(TUM), 점 지도 월드 변환.

    bag_tools.py stats <bag> --out meta.json        토픽별 메시지 수·header 주기·간격 (sim 시각)
    bag_tools.py tum <bag> --topic /amr_01/ground_truth/odom --out gt.tum
    bag_tools.py cloud <in.ply|in.pcd> <out.ply> --tf x,y,z,qx,qy,qz,qw

stats 는 header 가 첫 필드인 형만 header.stamp 로 주기를 낸다 (CDR 머리만 읽는다 — 점군을 역직렬화하지 않는다).
TFMessage·Clock 같은 형은 bag 기록 시각(벽시계 기준 수신 시각) 통계만 낸다.
"""

import argparse
import json
import os
import sys

from amr_slam3d import bagstat, cloudio
import numpy as np

#: header 가 첫 필드인 형 (CDR 머리로 스탬프를 읽는다)
HEADER_TYPES = {'sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/Imu',
                'sensor_msgs/msg/JointState', 'nav_msgs/msg/Odometry',
                'geometry_msgs/msg/TwistStamped'}
NOMINAL_HZ = {'sensor_msgs/msg/PointCloud2': 10.0, 'sensor_msgs/msg/LaserScan': 10.0}


def storage_id(path: str) -> str:
    """저장 형식 — bag 디렉터리(또는 파일)에 .mcap 파일이 있으면 mcap."""
    if os.path.isdir(path):
        return 'mcap' if any(f.endswith('.mcap') for f in os.listdir(path)) else 'sqlite3'
    return 'mcap' if path.endswith('.mcap') else 'sqlite3'


def _reader(path: str):
    import rosbag2_py
    storage = storage_id(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def cmd_stats(ns) -> int:
    reader, types = _reader(ns.bag)
    stamps = {t: [] for t in types}
    recv = {t: [] for t in types}
    sizes = {t: 0 for t in types}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        recv[topic].append(t_ns * 1e-9)
        sizes[topic] += len(raw)
        if types[topic] in HEADER_TYPES:
            stamps[topic].append(bagstat.cdr_header_stamp(raw))
    out = {'bag': os.path.abspath(ns.bag), 'topics': {}}
    for topic in sorted(types):
        rec = {'type': types[topic], 'count': len(recv[topic]), 'bytes': sizes[topic],
               'recv_wall': bagstat.interval_stats(recv[topic])}
        if stamps[topic]:
            rec['header'] = bagstat.interval_stats(stamps[topic], NOMINAL_HZ.get(types[topic]))
        out['topics'][topic] = rec
    allrecv = [v for vs in recv.values() for v in vs]
    out['recv_wall_span_s'] = (max(allrecv) - min(allrecv)) if allrecv else 0.0
    clock = out['topics'].get('/clock')
    out['note'] = ('header 통계는 시뮬레이션 시각, recv_wall 은 기록 시각(벽시계). '
                   'RTF ≈ header span / recv_wall span (같은 토픽)')
    if clock:
        out['clock_count'] = clock['count']
    with open(ns.out, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    for topic, rec in out['topics'].items():
        h = rec.get('header', {})
        print(f"{topic:40s} {rec['count']:7d}  hz(sim)={h.get('rate_hz', float('nan')):8.3f}  "
              f"gaps={h.get('gaps', '-')}  bursts={h.get('bursts', '-')}  "
              f"MB={rec['bytes'] / 1e6:9.1f}")
    return 0


def cmd_tum(ns) -> int:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader, types = _reader(ns.bag)
    if ns.topic not in types:
        raise SystemExit(f'{ns.topic} 가 bag 에 없다')
    cls = get_message(types[ns.topic])
    n = 0
    with open(ns.out, 'w', encoding='utf-8') as f:
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic != ns.topic:
                continue
            m = deserialize_message(raw, cls)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            f.write(f'{st:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} '
                    f'{q.w:.6f}\n')
            n += 1
    print(f'{ns.out}: {n} 자세 ({ns.topic}, frame = 메시지 header.frame_id)')
    return 0


def cmd_cloud(ns) -> int:
    xyz = (cloudio.read_pcd_xyz(ns.src) if ns.src.endswith('.pcd')
           else cloudio.read_ply_xyz(ns.src))
    vals = [float(v) for v in ns.tf.split(',')]
    if len(vals) != 7:
        raise SystemExit('--tf 는 x,y,z,qx,qy,qz,qw')
    n0 = len(xyz)
    xyz = cloudio.finite(xyz)
    out = cloudio.transform(xyz, vals[:3], vals[3:])
    cloudio.write_ply_xyz(ns.dst, out)
    lo, hi = (out.min(0), out.max(0)) if len(out) else (np.zeros(3), np.zeros(3))
    print(json.dumps({'src': ns.src, 'dst': ns.dst, 'n_in': n0, 'n_out': len(out),
                      'min': lo.round(3).tolist(), 'max': hi.round(3).tolist()},
                     ensure_ascii=False))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('stats')
    s.add_argument('bag')
    s.add_argument('--out', required=True)
    s = sub.add_parser('tum')
    s.add_argument('bag')
    s.add_argument('--topic', required=True)
    s.add_argument('--out', required=True)
    s = sub.add_parser('cloud')
    s.add_argument('src')
    s.add_argument('dst')
    s.add_argument('--tf', required=True)
    ns = ap.parse_args(argv)
    return {'stats': cmd_stats, 'tum': cmd_tum, 'cloud': cmd_cloud}[ns.cmd](ns)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
