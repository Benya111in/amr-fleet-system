#!/usr/bin/env python3
"""
3D LiDAR 로봇 1대의 센서 확인 (보고서 1–2단계 [측정]): 점군 형식·주기, 정답 면 거리 오차, IMU 방위 잡음 유무.

    lidar3d_sensor_check.py --robot amr_01 --world-sdf W.sdf --models-dir M --out check.json

전제: Gazebo 에 그 로봇만 있고(동적 행위자를 뺀 월드 — make_static_world.py), 다른 노드가 cmd_vel 을 내지 않는다.

1. 정지 상태로 points_raw(브리지 출력)와 points(전처리 출력)를 각각 모아 필드·오프셋·point_step·height·width·
   is_dense·frame_id·주기(헤더 스탬프 = 시뮬레이션 시각)를 기록한다.
2. 정지 상태에서 IMU 방위(쿼터니언)의 변동과 자이로·가속도의 표준편차·평균을 잰다.
3. 센서 오차: 정지 상태 스캔의 유한 점을 정답 자세(ground_truth/odom)와 장착 외부 파라미터로 월드 좌표로 옮겨
   (a) 가장 가까운 정답 시각체 면까지 거리 |d| 와 (b) 빔 방향 광선을 정답 면과 교차시킨 기대 거리와의 차
   (측정 거리 − 기대 거리, 부호 있음)를 낸다. (b) 는 잡음 σ 와 계통 편차를 가른다 (gz-sim #2743).
4. 제자리 회전(cmd_vel 각속도)을 하며 IMU 방위 yaw 변화와 정답 yaw 변화를 같은 시각에서 비교한다. 방위가 잡음 없는
   정답이면 차이는 보간 오차 수준이고, 자이로 적분 yaw 는 바이어스·잡음만큼 벌어진다.
"""

import argparse
import json
import sys
import threading
import time

from amr_lidar3d import cloud_io as cio
from amr_lidar3d import world_gt as wg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2


def stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Collector(Node):
    """점군·IMU·정답 오도메트리를 모은다."""

    def __init__(self, robot: str):
        super().__init__('lidar3d_sensor_check')
        self.lock = threading.Lock()
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.raw, self.flt, self.imu, self.gt = [], [], [], []
        self.keep_clouds = True
        self.create_subscription(PointCloud2, f'/{robot}/lidar3d/points_raw',
                                 lambda m: self._add(self.raw, m), rel)
        self.create_subscription(PointCloud2, f'/{robot}/lidar3d/points',
                                 lambda m: self._add(self.flt, m), rel)
        self.create_subscription(Imu, f'/{robot}/imu/data_raw',
                                 lambda m: self._add(self.imu, m), QoSProfile(depth=200))
        self.create_subscription(Odometry, f'/{robot}/ground_truth/odom',
                                 lambda m: self._add(self.gt, m), QoSProfile(depth=50))
        self.cmd = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)

    def _add(self, store, msg):
        with self.lock:
            if store is self.raw or store is self.flt:
                if not self.keep_clouds:
                    store.append((stamp(msg), None))
                    return
            store.append((stamp(msg), msg))

    def take(self):
        with self.lock:
            out = (list(self.raw), list(self.flt), list(self.imu), list(self.gt))
            self.raw.clear()
            self.flt.clear()
            self.imu.clear()
            self.gt.clear()
        return out


def describe_cloud(msgs):
    if not msgs:
        return {'n': 0}
    m = msgs[-1][1]
    dense = [bool(x.is_dense) for _, x in msgs if x is not None]
    widths = [int(x.width) * int(x.height) for _, x in msgs if x is not None]
    return {
        'n': len(msgs), 'frame_id': m.header.frame_id, 'fields': cio.cloud_fields(m),
        'point_step': int(m.point_step), 'height': int(m.height), 'width': int(m.width),
        'is_dense_all': all(dense), 'is_dense_any': any(dense),
        'points_per_msg_min': min(widths), 'points_per_msg_max': max(widths),
        'rate': cio.interval_stats([t for t, _ in msgs]),
    }


def ray_box_hits(origin, dirs, boxes, planes):
    """각 단위 방향 광선이 처음 맞는 정답 면까지 거리 (없으면 inf). origin: [3], dirs: [N, 3]."""
    best = np.full(len(dirs), np.inf)
    for b in boxes:
        o = (origin - b.center) @ b.rot
        d = dirs @ b.rot
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (-b.half - o) / d
            t2 = (b.half - o) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        inside = np.all(np.abs(o) <= b.half)
        t = np.where(tmin > 0, tmin, tmax if inside else np.inf)
        hit = (tmax >= np.maximum(tmin, 0.0)) & (t > 0)
        best = np.where(hit & (t < best), t, best)
    for p in planes:
        o = (origin - p.center) @ p.rot
        d = dirs @ p.rot
        with np.errstate(divide='ignore', invalid='ignore'):
            t = -o[2] / d[:, 2]
        q = o[None, :2] + t[:, None] * d[:, :2]
        ok = (t > 0) & np.all(np.abs(q) <= p.half, axis=1)
        best = np.where(ok & (t < best), t, best)
    return best


def gt_pose(gt_msgs):
    """정지 구간 정답 자세 평균 (base_footprint in world) → (R, t, 위치 표준편차)."""
    pos = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
                    for _, m in gt_msgs])
    q = gt_msgs[-1][1].pose.pose.orientation
    return cio.quat_matrix(q.x, q.y, q.z, q.w), pos.mean(axis=0), float(pos.std(axis=0).max())


def sensor_error(raw_msgs, gt_msgs, ext, boxes, planes, max_scans):
    rot, trans, pos_std = gt_pose(gt_msgs)
    r_s, t_s = cio.pose_from_xyz_rpy(ext[:3], ext[3:])
    r_ws, t_ws = rot @ r_s, rot @ t_s + trans          # world ← sensor
    dist_all, resid_all, ranges_all, rings_all = [], [], [], []
    for _, m in raw_msgs[-max_scans:]:
        arr = cio.cloud_array(m)
        p = cio.xyz(arr)
        ok = np.all(np.isfinite(p), axis=1)
        p, ring = p[ok], arr['ring'][ok] if 'ring' in arr.dtype.names else None
        rng = np.linalg.norm(p, axis=1)
        pw = cio.transform(p, r_ws, t_ws)
        dist_all.append(wg.surface_distance(pw, boxes, planes))
        dirs = (p / rng[:, None]) @ r_ws.T
        expect = ray_box_hits(t_ws, dirs, boxes, planes)
        resid_all.append(rng - expect)
        ranges_all.append(rng)
        if ring is not None:
            rings_all.append(ring.astype(int))
    dist = np.concatenate(dist_all)
    resid = np.concatenate(resid_all)
    rng = np.concatenate(ranges_all)
    finite = np.isfinite(resid)
    out = {
        'scans': len(dist_all), 'points': int(dist.size), 'gt_pos_std_m': pos_std,
        'sensor_world_xyz': t_ws.tolist(),
        'surface_distance_abs': wg.summarize(dist),
        'surface_distance_within_0p5m': wg.summarize(dist[dist < 0.5]),
        'outlier_frac_gt_0p5m': float(np.mean(dist >= 0.5)),
        'range_residual': wg.summarize(np.abs(resid[finite])),
        'range_residual_signed_mean': float(np.mean(resid[finite & (np.abs(resid) < 0.5)])),
        'range_residual_signed_median': float(np.median(resid[finite & (np.abs(resid) < 0.5)])),
        'range_residual_std_within_0p5m': float(np.std(resid[finite & (np.abs(resid) < 0.5)])),
        'no_gt_hit_frac': float(np.mean(~finite)),
        'by_range': {},
    }
    for lo, hi in ((0.0, 2.0), (2.0, 5.0), (5.0, 10.0), (10.0, 20.0), (20.0, 31.0)):
        sel = finite & (rng >= lo) & (rng < hi) & (np.abs(resid) < 0.5)
        if sel.any():
            out['by_range'][f'{lo:g}-{hi:g}m'] = {
                'n': int(sel.sum()), 'signed_median': float(np.median(resid[sel])),
                'std': float(np.std(resid[sel])),
                'abs_p95': float(np.percentile(np.abs(resid[sel]), 95))}
    if rings_all:
        ring = np.concatenate(rings_all)
        out['ring_values'] = sorted(int(v) for v in np.unique(ring))
    return out


def imu_static(imu_msgs):
    q = np.array([[m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w]
                  for _, m in imu_msgs])
    g = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
                  for _, m in imu_msgs])
    a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
                  for _, m in imu_msgs])
    return {'n': len(imu_msgs), 'rate': cio.interval_stats([t for t, _ in imu_msgs]),
            'orientation_component_std_max': float(q.std(axis=0).max()),
            'orientation_component_ptp_max': float(np.ptp(q, axis=0).max()),
            'gyro_std': g.std(axis=0).tolist(), 'gyro_mean': g.mean(axis=0).tolist(),
            'accel_std': a.std(axis=0).tolist(), 'accel_mean': a.mean(axis=0).tolist(),
            'orientation_covariance0': float(imu_msgs[-1][1].orientation_covariance[0])}


def imu_rotation(imu_msgs, gt_msgs):
    ti = np.array([t for t, _ in imu_msgs])
    yi = np.array([cio.quat_yaw(m.orientation.x, m.orientation.y, m.orientation.z,
                                m.orientation.w) for _, m in imu_msgs])
    wz = np.array([m.angular_velocity.z for _, m in imu_msgs])
    tg = np.array([t for t, _ in gt_msgs])
    q = [m.pose.pose.orientation for _, m in gt_msgs]
    yg = np.unwrap(np.array([cio.quat_yaw(o.x, o.y, o.z, o.w) for o in q]))
    yi = np.unwrap(yi)
    sel = (ti >= tg[0]) & (ti <= tg[-1])
    ti, yi, wz = ti[sel], yi[sel], wz[sel]
    yg_i = np.interp(ti, tg, yg)
    d_imu = yi - yi[0]
    d_gt = yg_i - yg_i[0]
    gyro_int = np.concatenate([[0.0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(ti))])
    err_orient = cio.wrap(d_imu - d_gt)
    err_gyro = cio.wrap(gyro_int - d_gt)
    return {'n': int(ti.size), 'span_s': float(ti[-1] - ti[0]), 'yaw_turned_rad': float(d_gt[-1]),
            'orientation_minus_gt_abs_max': float(np.abs(err_orient).max()),
            'orientation_minus_gt_abs_median': float(np.median(np.abs(err_orient))),
            'gyro_integral_minus_gt_end': float(err_gyro[-1]),
            'gyro_integral_minus_gt_abs_max': float(np.abs(err_gyro).max())}


def wait_until(pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.2)
    return False


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--robot', default='amr_01')
    ap.add_argument('--world-sdf', required=True)
    ap.add_argument('--models-dir', required=True)
    ap.add_argument('--exclude', action='append', default=['forklift', 'shuttle_amr'])
    ap.add_argument('--extrinsic', default='0.27 0 0.90 0 0 0',
                    help='base_footprint → lidar3d_link x y z r p y '
                         '(lidar3d.yaml + base_link_height)')
    ap.add_argument('--static-s', type=float, default=15.0, help='정지 수집 [s, 벽시계]')
    ap.add_argument('--scans', type=int, default=20, help='오차 계산에 쓸 스캔 수')
    ap.add_argument('--spin-rate', type=float, default=0.5, help='회전 시험 각속도 [rad/s]')
    ap.add_argument('--spin-s', type=float, default=8.0, help='회전 시험 지령 시간 [s, 벽시계]')
    ap.add_argument('--wait-s', type=float, default=180.0)
    ap.add_argument('--out', required=True)
    ns = ap.parse_args(argv)
    ext = [float(v) for v in ns.extrinsic.split()]

    boxes, planes, skipped = wg.load_world(ns.world_sdf, ns.models_dir, ns.exclude)
    rclpy.init()
    node = Collector(ns.robot)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    result = {'robot': ns.robot, 'gt_boxes': len(boxes), 'gt_planes': len(planes),
              'gt_skipped_visuals': len(skipped), 'gt_skipped_examples': skipped[:10]}
    ok = wait_until(lambda: len(node.flt) >= 3 and len(node.raw) >= 3 and len(node.gt) >= 3
                    and len(node.imu) >= 10, ns.wait_s)
    result['ready'] = ok
    if not ok:
        result['error'] = (f'수신 부족: raw {len(node.raw)}, points {len(node.flt)}, '
                           f'imu {len(node.imu)}, gt {len(node.gt)}')
        print(result['error'], file=sys.stderr)
    node.take()
    time.sleep(ns.static_s)
    raw, flt, imu, gt = node.take()
    result['points_raw'] = describe_cloud(raw)
    result['points'] = describe_cloud(flt)
    if raw and gt:
        result['sensor_error'] = sensor_error(raw, gt, ext, boxes, planes, ns.scans)
    if imu:
        result['imu_static'] = imu_static(imu)

    # 회전 시험: 점군은 보관하지 않는다 (메모리)
    node.keep_clouds = False
    twist = Twist()
    twist.angular.z = ns.spin_rate
    t_end = time.monotonic() + ns.spin_s
    while time.monotonic() < t_end:
        node.cmd.publish(twist)
        time.sleep(0.05)
    node.cmd.publish(Twist())
    time.sleep(3.0)
    _, _, imu_rot, gt_rot = node.take()
    if imu_rot and gt_rot:
        result['imu_rotation'] = imu_rotation(imu_rot, gt_rot)
    with open(ns.out, 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    summary = {k: result.get(k) for k in ('ready',)}
    se = result.get('sensor_error', {})
    summary['dist_median'] = se.get('surface_distance_abs', {}).get('median')
    summary['dist_p95'] = se.get('surface_distance_abs', {}).get('p95')
    summary['imu_rot'] = result.get('imu_rotation')
    print(json.dumps(summary, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
