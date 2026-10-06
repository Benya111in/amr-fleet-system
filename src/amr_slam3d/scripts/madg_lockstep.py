#!/usr/bin/env python3
"""
madgwick 잠금 단계(lockstep) 재계산 → 파생 bag (보너스 2 재생 보강, 리서치 R10).

    madg_lockstep.py --in-bag <원 bag> --out-bag <새 디렉터리> --report derive.json \
        --gain 0.1=/amr_01/imu/data_madgwick_re --gain 0.01=/amr_01/imu/data_madgwick_re_g001

왜: 파일럿 bag 에 기록된 madgwick 출력에는 sim 9.48 s 에 0.14 s 공백이 있다 (입력 imu/data 에는 없다 — rtabmap.md §3).
IMU 를 쓰는 행이 그 스캔을 버렸다. 재생 중에 madgwick 을 다시 돌리면 공백은 사라지지만 madgwick 입력 구독이 best-effort
(깊이 5)라 부하에 따라 재생마다 다른 입력을 잃을 수 있고, liosam 이미지에는 madgwick 이 없다.
그래서 **실제 madgwick 노드**(이 이미지의 imu_filter_madgwick, 우리 config/madgwick.yaml)에 원 bag 의 imu/data 를
한 개씩 넣고, 이득마다 출력이 돌아온 뒤에 다음 것을 넣는다. madgwick 은 입력 header 스탬프 차로 dt 를 내므로
(입력을 잃지 않는 한) 실시간 재계산과 같은 값이다 — 기록된 출력과 공백 이전 구간을 비교해 확인한다 (report).

파생 bag 에는 재생에 쓰는 토픽만 그대로 복사하고(--copy), 재계산 출력을 그 입력 메시지와 **같은 bag 시각**으로 넣는다.
정답 자세(ground_truth/odom)와 기록된 madgwick 은 넣지 않는다 (기록된 것을 쓰는 옛 행은 원 bag 으로 돈다).

오라클(정답 방위) 토픽 (--oracle-topic): `imu/data` 의 방위는 **정답이 아니라 항등** 쿼터니언이다
(imu_filter_node.cpp 147–149행 `orientation.w = 1`, 공분산[0] = −1 = "방위 없음";
이 bag 에서 yaw 가 끝까지 0 [측정]).
정답 방위는 Gazebo 원시 `imu/data_raw` 에만 있다. 그래서 상한 참고 행용 토픽은 `imu/data`(바이어스 보정 각속도·가속도)에
같은 스탬프 `imu/data_raw` 의 방위·방위 공분산만 옮겨 만든다 — 각속도·가속도는 다른 행과 같고 방위만 정답이다.
"""

import argparse
import json
import os
import subprocess
import sys
import time

from amr_slam3d import bagstat, imustat
import numpy as np

COPY = ('/clock', '/tf', '/tf_static', '/amr_01/lidar3d/points', '/amr_01/scan_filtered',
        '/amr_01/imu/data')
IMU_IN = '/amr_01/imu/data'
IMU_RAW = '/amr_01/imu/data_raw'
RECORDED = '/amr_01/imu/data_madgwick'
#: 기록된 madgwick 공백 (rtabmap.md §3, stamps_near.log): 9.48 s 다음이 9.62 s
GAP = (9.48, 9.62)


def _open_reader(path, topics):
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    meta = {t.name: t for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return r, meta


def read_imu(path):
    """원 bag 의 imu/data (bag 시각, 원시 바이트), 기록된 madgwick 원시 바이트, imu/data_raw {스탬프: 원시 바이트}."""
    r, meta = _open_reader(path, [IMU_IN, RECORDED, IMU_RAW])
    imu, rec, raw_by_key = [], [], {}
    while r.has_next():
        topic, raw, t = r.read_next()
        if topic == IMU_IN:
            imu.append((t, bytes(raw)))
        elif topic == RECORDED:
            rec.append((t, bytes(raw)))
        else:
            raw_by_key[bagstat.cdr_header_key(raw)] = bytes(raw)
    return imu, rec, meta, raw_by_key


def oracle_messages(imu, raw_by_key):
    """imu/data 각 메시지에 같은 스탬프 imu/data_raw 의 방위·방위 공분산을 옮긴 원시 바이트 (짝 없으면 오류)."""
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import Imu
    out = []
    for _, raw in imu:
        key = bagstat.cdr_header_key(raw)
        if key not in raw_by_key:
            raise RuntimeError(f'imu/data 스탬프 {key} 에 짝 imu/data_raw 가 없다')
        m = deserialize_message(raw, Imu)
        g = deserialize_message(raw_by_key[key], Imu)
        m.orientation = g.orientation
        m.orientation_covariance = g.orientation_covariance
        out.append(serialize_message(m))
    return out


def start_nodes(gains, params, log_dir):
    """이득마다 madgwick 노드 하나 (네임스페이스 /b2lock_<i>, 이름은 설정 파일 키와 같은 imu_filter_madgwick)."""
    procs = []
    for i, (gain, _) in enumerate(gains):
        cmd = ['ros2', 'run', 'imu_filter_madgwick', 'imu_filter_madgwick_node', '--ros-args',
               '-r', f'__ns:=/b2lock_{i}', '-r', '__node:=imu_filter_madgwick',
               '--params-file', params, '-p', f'gain:={gain}']
        log = open(os.path.join(log_dir, f'madg_node_{i}.log'), 'w', encoding='utf-8')
        procs.append((subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT), log, cmd))
    return procs


def param_get(ns, name):
    out = subprocess.run(['ros2', 'param', 'get', f'/{ns}/imu_filter_madgwick', name],
                         capture_output=True, text=True, timeout=60)
    return (out.stdout + out.stderr).strip()


def lockstep(imu, n_gain, timeout):
    """입력을 하나씩 넣고 모든 이득의 출력(같은 스탬프)을 받은 뒤 다음으로. 출력 원시 바이트 [k][g]."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = Node('b2_madg_lockstep')
    rel = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    pubs = [node.create_publisher(Imu, f'/b2lock_{i}/imu/data_raw', rel) for i in range(n_gain)]
    got = [dict() for _ in range(n_gain)]

    def cb(i):
        def f(raw):
            got[i][bagstat.cdr_header_key(raw)] = bytes(raw)
        return f
    subs = [node.create_subscription(Imu, f'/b2lock_{i}/imu/data', cb(i), rel, raw=True)
            for i in range(n_gain)]
    t0 = time.monotonic()
    while not all(p.get_subscription_count() >= 1 for p in pubs) or \
            not all(node.count_publishers(f'/b2lock_{i}/imu/data') >= 1 for i in range(n_gain)):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - t0 > 60.0:
            raise RuntimeError('madgwick 노드 발견 실패 (60 s)')
    time.sleep(1.0)
    outs, waits = [], []
    t_start = time.monotonic()
    for k, (_, raw) in enumerate(imu):
        key = bagstat.cdr_header_key(raw)
        for p in pubs:
            p.publish(raw)
        t1 = time.monotonic()
        while not all(key in g for g in got):
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - t1 > timeout:
                raise RuntimeError(f'입력 {k} (스탬프 {key}) 의 출력이 {timeout} s 안에 안 왔다 — '
                                   f'받은 이득 {[key in g for g in got]}. 재시도하지 않는다 (입력 유실과 출력 유실을 '
                                   '가를 수 없어 다시 넣으면 필터를 두 번 갱신할 수 있다)')
        outs.append([g.pop(key) for g in got])
        waits.append(time.monotonic() - t1)
    extra = sum(len(g) for g in got)
    for s in subs:
        node.destroy_subscription(s)
    node.destroy_node()
    rclpy.shutdown()
    return outs, {'wall_s': time.monotonic() - t_start, 'wait_ms_max': 1e3 * max(waits),
                  'wait_ms_median': 1e3 * float(np.median(waits)), 'unexpected_outputs': extra}


def write_bag(in_bag, out_bag, copy, gains, outs, rec_meta, oracle_topic='', oracle=()):
    import rosbag2_py
    r, meta = _open_reader(in_bag, copy)
    missing = [t for t in copy if t not in meta]
    if missing:
        raise RuntimeError(f'원 bag 에 없는 복사 토픽 {missing}')
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=out_bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for t in copy:
        w.create_topic(meta[t])
    qos = rec_meta.offered_qos_profiles if rec_meta is not None else ''
    for _, topic in gains:
        w.create_topic(rosbag2_py.TopicMetadata(topic, 'sensor_msgs/msg/Imu', 'cdr', qos))
    if oracle_topic:
        w.create_topic(rosbag2_py.TopicMetadata(oracle_topic, 'sensor_msgs/msg/Imu', 'cdr', qos))
    counts = {t: 0 for t in list(copy) + [t for _, t in gains] + ([oracle_topic] if oracle_topic
                                                                  else [])}
    k = 0
    while r.has_next():
        topic, raw, t = r.read_next()
        w.write(topic, raw, t)
        counts[topic] += 1
        if topic == IMU_IN:
            for g, (_, out_topic) in enumerate(gains):
                w.write(out_topic, outs[k][g], t)
                counts[out_topic] += 1
            if oracle_topic:
                w.write(oracle_topic, oracle[k], t)
                counts[oracle_topic] += 1
            k += 1
    del w
    if k != len(outs):
        raise RuntimeError(f'파생 bag 의 imu/data {k} 개 ≠ 재계산 {len(outs)} 개')
    return counts


def analyse(imu, rec, gains, outs, raw_by_key):
    """정답 방위(imu/data_raw) 대비 roll·pitch 오차와 yaw 표류, 기록된 madgwick 과의 차 (공백 앞·뒤)."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    def q_of(m):
        o = m.orientation
        return [o.x, o.y, o.z, o.w]
    ins = [deserialize_message(raw, Imu) for _, raw in imu]
    t_in = np.array([m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for m in ins])
    q_id = np.array([q_of(m) for m in ins])
    q_gt = np.array([q_of(deserialize_message(raw_by_key[bagstat.cdr_header_key(raw)], Imu))
                     for _, raw in imu])
    wz = np.array([m.angular_velocity.z for m in ins])
    turning = np.abs(wz) > 0.1
    ident = np.tile([0.0, 0.0, 0.0, 1.0], (len(q_id), 1))
    gt_yaw = np.degrees(np.unwrap(imustat.quat_to_rpy(q_gt)[:, 2]))
    rep = {'n_input': len(ins), 'n_turning': int(turning.sum()),
           'turning_rule': '|gyro z| > 0.1 rad/s',
           'reference': 'imu/data_raw orientation (Gazebo 정답, 같은 스탬프)',
           'imu_data_orientation_vs_identity_max_deg':
               float(imustat.quat_angle_deg(q_id, ident).max()),
           'gt_yaw_span_deg': float(np.ptp(gt_yaw))}
    rec_msgs = [deserialize_message(raw, Imu) for _, raw in rec]
    t_rec = np.array([m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for m in rec_msgs])
    q_rec = np.array([q_of(m) for m in rec_msgs]).reshape(-1, 4)
    rep['recorded'] = {'n': len(rec_msgs), 'first': float(t_rec[0]) if len(t_rec) else None,
                       'input_first': float(t_in[0])}
    for g, (gain, topic) in enumerate(gains):
        q = np.array([q_of(deserialize_message(o[g], Imu)) for o in outs])
        e = imustat.rpy_error_deg(q, q_gt)
        # 6 축 madgwick 의 yaw 는 자이로 적분이라 절대 방위가 없다 (첫 출력의 yaw 는 가속도로 못 정한다) →
        # 첫 표본 대비 표류만 본다
        drift = np.degrees(imustat.wrap(np.radians(e[:, 2] - e[0, 2])))
        s = {'topic': topic, 'gain': gain,
             'roll_deg': imustat.stats(e[:, 0]), 'pitch_deg': imustat.stats(e[:, 1]),
             'roll_deg_turning': imustat.stats(e[turning, 0]),
             'pitch_deg_turning': imustat.stats(e[turning, 1]),
             'yaw_offset_first_deg': float(e[0, 2]), 'yaw_drift_deg': imustat.stats(drift),
             'yaw_drift_final_deg': float(drift[-1])}
        ia, ib = imustat.pair_by_stamp(t_rec, t_in)
        if len(ia):
            d = imustat.quat_angle_deg(q_rec[ia], q[ib])
            tt = t_rec[ia]
            pre, post = tt < GAP[0] + 1e-9, tt >= GAP[1] - 1e-9
            edges = [float(t_in[0]), GAP[0], GAP[1] + 1e-6, 15.0, 30.0, 60.0, 120.0, 200.0]
            bins = {}
            for lo, hi in zip(edges[:-1], edges[1:]):
                m = (tt >= lo - 1e-9) & (tt < hi)
                if m.any():
                    bins[f'{lo:.3f}-{hi:.3f}'] = {'n': int(m.sum()), 'max_deg': float(d[m].max()),
                                                  'median_deg': float(np.median(d[m]))}
            last_pre = int(np.nonzero(pre)[0][-1]) if pre.any() else None
            first_post = int(np.nonzero(post)[0][0]) if post.any() else None
            s['vs_recorded'] = {
                'paired': int(len(ia)), 'n_pre': int(pre.sum()), 'n_post': int(post.sum()),
                'max_angle_deg_pre_gap': float(d[pre].max()) if pre.any() else None,
                'max_angle_deg_post_gap': float(d[post].max()) if post.any() else None,
                'angle_deg_last_pre_gap': float(d[last_pre]) if last_pre is not None else None,
                'angle_deg_first_post_gap':
                    float(d[first_post]) if first_post is not None else None,
                'n_exact': int(np.count_nonzero(d == 0.0)), 'by_window': bins}
        rep[f'gain_{gain}'] = s
    return rep


def read_outputs(out_bag, gains, n):
    """파생 bag 에서 이득별 재계산 출력 원시 바이트 [k][g] (imu/data 순서와 같다)."""
    r, _ = _open_reader(out_bag, [t for _, t in gains])
    per = {t: [] for _, t in gains}
    while r.has_next():
        topic, raw, _ = r.read_next()
        per[topic].append(bytes(raw))
    for t, v in per.items():
        if len(v) != n:
            raise RuntimeError(f'{t}: {len(v)} 개 ≠ 입력 {n} 개')
    return [[per[t][k] for _, t in gains] for k in range(n)]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--in-bag', required=True)
    ap.add_argument('--out-bag', required=True)
    ap.add_argument('--report', required=True)
    ap.add_argument('--gain', action='append', required=True, help='이득=출력토픽')
    ap.add_argument('--params', default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                     '..', 'config', 'madgwick.yaml'))
    ap.add_argument('--copy', nargs='+', default=list(COPY))
    ap.add_argument('--timeout', type=float, default=5.0, help='[s] 입력 하나당 출력 대기 상한')
    ap.add_argument('--oracle-topic', default='/amr_01/imu/data_gtorient',
                    help="imu/data + imu/data_raw 정답 방위 합성 토픽 ('' = 만들지 않음)")
    ap.add_argument('--analyse-only', action='store_true',
                    help='재계산하지 않고 이미 만든 --out-bag 의 출력으로 분석만 다시 (report 를 새로 쓴다)')
    ns = ap.parse_args(argv)
    gains = [(g.split('=', 1)[0], g.split('=', 1)[1]) for g in ns.gain]
    if ns.analyse_only:
        imu, rec, _, raw_by_key = read_imu(ns.in_bag)
        rep = {'in_bag': ns.in_bag, 'out_bag': ns.out_bag, 'gains': gains, 'analyse_only': True,
               'analysis': analyse(imu, rec, gains, read_outputs(ns.out_bag, gains, len(imu)),
                                   raw_by_key)}
        with open(ns.report, 'w', encoding='utf-8') as f:
            json.dump(rep, f, ensure_ascii=False, indent=1)
        return 0
    if os.path.exists(ns.out_bag):
        raise SystemExit(f'이미 있다: {ns.out_bag}')
    log_dir = os.path.dirname(os.path.abspath(ns.report))
    report = {'in_bag': ns.in_bag, 'out_bag': ns.out_bag, 'gains': gains,
              'params': os.path.abspath(ns.params), 'copy': ns.copy,
              'started': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    imu, rec, meta, raw_by_key = read_imu(ns.in_bag)
    oracle = oracle_messages(imu, raw_by_key) if ns.oracle_topic else []
    report['oracle_topic'] = ns.oracle_topic
    procs = start_nodes(gains, ns.params, log_dir)
    try:
        time.sleep(3.0)
        report['param_check'] = {f'b2lock_{i}': {'gain': param_get(f'b2lock_{i}', 'gain'),
                                                 'use_mag': param_get(f'b2lock_{i}', 'use_mag'),
                                                 'zeta': param_get(f'b2lock_{i}', 'zeta')}
                                 for i in range(len(gains))}
        outs, report['lockstep'] = lockstep(imu, len(gains), ns.timeout)
    finally:
        for p, log, _ in procs:
            p.send_signal(2)
        for p, log, _ in procs:
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                p.kill()
            log.close()
    report['n_out'] = len(outs)
    report['bag_counts'] = write_bag(ns.in_bag, ns.out_bag, ns.copy, gains, outs,
                                     meta.get(RECORDED), ns.oracle_topic, oracle)
    report['analysis'] = analyse(imu, rec, gains, outs, raw_by_key)
    report['ended'] = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    with open(ns.report, 'w', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=1)
    print(json.dumps({'n_out': report['n_out'], 'lockstep': report['lockstep']}))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
