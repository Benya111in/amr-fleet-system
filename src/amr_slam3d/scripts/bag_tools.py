#!/usr/bin/env python3
"""
bag·지도 파일 도구 — 파일럿 bag 메타데이터, 정답 궤적(TUM), 점 지도 월드 변환.

    bag_tools.py stats <bag> --out meta.json        토픽별 메시지 수·header 주기·간격 (sim 시각)
    bag_tools.py tum <bag> --topic /amr_01/ground_truth/odom --out gt.tum
    bag_tools.py cloud <in.ply|in.pcd> <out.ply> --tf x,y,z,qx,qy,qz,qw
    bag_tools.py match --in-bag <bag> --in-topic T --out-bag <out_bag> --out-topic T2 --out m.json
    bag_tools.py poses <poses.txt> <out.tum> --tf x,y,z,qx,qy,qz,qw     (SLAM map 프레임 → 월드)
    bag_tools.py assemble <bag> --topic /amr_01/lidar3d/points --poses p.tum --base F --out c.ply

match 는 "처리 스캔 누락 0" 판정이다: 입력 bag 의 header 스탬프마다 출력 bag 에 같은 스탬프가 있는지 본다.
출력 bag 은 C++ 기록기(ros2 bag record)라 Python 측정 노드처럼 뒤처져 버리지 않는다.
assemble 은 자세 파일(월드 프레임, 스탬프 = 스캔 스탬프)의 각 자세에 그 스탬프의 점군을 붙여 월드 점 지도를 만든다
(rtabmap-export --cloud --scan 이 0 점을 내서 [측정 s3_pilot_a rows/rtabmap3d/export_cloud.log] 대신 쓴다).
점군 프레임 → base 변환은 bag 의 /tf_static 으로 계산한다.

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


def _stamps(path: str, topic: str):
    reader, types = _reader(path)
    if topic not in types:
        return None
    import rosbag2_py
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while reader.has_next():
        _, raw, _ = reader.read_next()
        out.append(bagstat.cdr_header_stamp(raw))
    return out


def cmd_match(ns) -> int:
    inp = _stamps(ns.in_bag, ns.in_topic)
    out = _stamps(ns.out_bag, ns.out_topic)
    if inp is None:
        raise SystemExit(f'{ns.in_topic} 가 {ns.in_bag} 에 없다')
    res = bagstat.matched(inp, out or [])
    res.update({'in_bag': ns.in_bag, 'in_topic': ns.in_topic, 'out_bag': ns.out_bag,
                'out_topic': ns.out_topic, 'out_present': out is not None,
                'out_stats': bagstat.interval_stats(out or [])})
    miss = np.asarray(sorted(set(inp) - set(out or [])))
    if len(miss):
        edges = np.arange(np.floor(min(inp)), np.ceil(max(inp)) + 10.0, 10.0)
        hist, _ = np.histogram(miss, bins=edges)
        res['missing_hist_10s'] = {f'{edges[i]:g}': int(h) for i, h in enumerate(hist) if h}
    with open(ns.out, 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: res[k] for k in ('input', 'output', 'matched', 'missing')}))
    return 0


def _parse_tf(text: str):
    vals = [float(v) for v in text.split(',')]
    if len(vals) != 7:
        raise SystemExit('--tf 는 x,y,z,qx,qy,qz,qw')
    return cloudio.pose_matrix(vals[:3], vals[3:])


def cmd_poses(ns) -> int:
    from scipy.spatial.transform import Rotation
    world = _parse_tf(ns.tf)
    arr = cloudio.read_pose_file(ns.src)
    with open(ns.dst, 'w', encoding='utf-8') as f:
        for st, x, y, z, qx, qy, qz, qw in arr:
            m = world @ cloudio.pose_matrix((x, y, z), (qx, qy, qz, qw))
            q = Rotation.from_matrix(m[:3, :3]).as_quat()
            f.write(f'{st:.9f} {m[0, 3]:.6f} {m[1, 3]:.6f} {m[2, 3]:.6f} '
                    f'{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n')
    print(f'{ns.dst}: {len(arr)} 자세 (월드)')
    return 0


def _static_tf(path: str, parent: str, child: str):
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer
    import rosbag2_py
    reader, _ = _reader(path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf_static']))
    buf = Buffer()
    while reader.has_next():
        _, raw, _ = reader.read_next()
        for tr in deserialize_message(raw, TFMessage).transforms:
            buf.set_transform_static(tr, 'bag')
    tr = buf.lookup_transform(parent, child, Time()).transform
    return cloudio.pose_matrix((tr.translation.x, tr.translation.y, tr.translation.z),
                               (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w))


def cmd_assemble(ns) -> int:
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    import rosbag2_py
    poses = cloudio.read_pose_file(ns.poses)
    want = {round(float(p[0]), 6): p for p in poses}
    reader, _ = _reader(ns.bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[ns.topic]))
    base_t_sensor = None
    chunks, used = [], 0
    while reader.has_next():
        _, raw, _ = reader.read_next()
        st = round(bagstat.cdr_header_stamp(raw), 6)
        if st not in want:
            continue
        msg = deserialize_message(raw, PointCloud2)
        if base_t_sensor is None:
            base_t_sensor = _static_tf(ns.bag, ns.base, msg.header.frame_id)
        p = want[st]
        m = cloudio.pose_matrix(p[1:4], p[4:8]) @ base_t_sensor
        off = {f.name: f.offset for f in msg.fields if f.datatype == 7}   # FLOAT32
        arr = np.frombuffer(bytes(msg.data), dtype=np.dtype(
            {'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
             'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step}),
            count=msg.width * msg.height)
        xyz = cloudio.finite(np.column_stack([arr['x'], arr['y'], arr['z']]).astype(float))
        r = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(r >= ns.range_min) & (r <= ns.range_max)]
        chunks.append(cloudio.apply(m, xyz))
        used += 1
    pts = cloudio.voxel_downsample(np.concatenate(chunks) if chunks else np.zeros((0, 3)),
                                   ns.voxel)
    cloudio.write_ply_xyz(ns.out, pts)
    print(json.dumps({'poses': len(poses), 'clouds_used': used, 'points': len(pts),
                      'voxel': ns.voxel, 'out': ns.out}, ensure_ascii=False))
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
    s = sub.add_parser('match')
    s.add_argument('--in-bag', required=True)
    s.add_argument('--in-topic', required=True)
    s.add_argument('--out-bag', required=True)
    s.add_argument('--out-topic', required=True)
    s.add_argument('--out', required=True)
    s = sub.add_parser('poses')
    s.add_argument('src')
    s.add_argument('dst')
    s.add_argument('--tf', required=True)
    s = sub.add_parser('assemble')
    s.add_argument('bag')
    s.add_argument('--topic', required=True)
    s.add_argument('--poses', required=True, help='월드 프레임 TUM (스탬프 = 스캔 스탬프)')
    s.add_argument('--base', default='amr_01/base_footprint')
    s.add_argument('--voxel', type=float, default=0.05)
    s.add_argument('--range-min', type=float, default=0.2)
    s.add_argument('--range-max', type=float, default=30.0)
    s.add_argument('--out', required=True)
    ns = ap.parse_args(argv)
    return {'stats': cmd_stats, 'tum': cmd_tum, 'cloud': cmd_cloud, 'match': cmd_match,
            'poses': cmd_poses, 'assemble': cmd_assemble}[ns.cmd](ns)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
