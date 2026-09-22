#!/usr/bin/env python3
r"""
실제 yolo_node 의 Gazebo 실시간 재현율 평가 (보류 구역의 새 자세, 깊이 검증 지면 진실과 IoU 매칭).

로봇을 보류 구역의 새 자세들로 순간이동시키고, 같은 스탬프의 perception/detections_2d 를 깊이 검증 지면 진실
(dataset_capture.py 와 같은 라벨러)과 IoU ≥ 0.5 로 맞춘다.

    # 시뮬레이션(로봇 1 대) + yolo_node 만 (배포 설정 perception.yaml, 기본 가중치) 을 띄운 뒤
    python3 src/amr_perception/scripts/yolo_live_eval.py \
        --world src/amr_simulation/worlds/warehouse.sdf --map maps/warehouse.yaml \
        --robot amr_01 --split test --poses 150 --seed 101 --out /data/live.json

필수 GT (재현율 분모): 채택 라벨 중 가시율 ≥ 0.5, 잘림 ≤ 0.3, 짧은 변 ≥ min_px, 거리 ≤ max_dist. 나머지 채택·가림 라벨은
무시 GT (맞혀도 FP 아님). 매칭되지 않은 box/person/sign 검출은 FP. 구역·자세는 dataset_capture.py 의 SPLIT_REGIONS 와
같은 표본기를 쓰되 다른 seed 로 새 자세를 뽑는다 (수집 데이터와 겹치지 않음).
"""

import argparse
import importlib.util
import json
import math
import os
import random
import subprocess
import sys
import time
from typing import Dict, List

HERE = os.path.dirname(os.path.abspath(__file__))
OUTPUT_CLASSES = ('box', 'person', 'sign')
DIST_BINS = ((0.0, 5.0), (5.0, 10.0), (10.0, 20.0), (20.0, 60.0))


def load_capture_module():
    """같은 디렉토리의 dataset_capture.py (구역·자세 표본기·라벨 규칙 공유)."""
    spec = importlib.util.spec_from_file_location('dataset_capture',
                                                  os.path.join(HERE, 'dataset_capture.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def gt_entries(labels, min_px: float, max_dist: float) -> List[tuple]:
    """라벨 판정 목록 → match_detections 입력 (클래스, bbox, 필수 여부, 거리). 출력 클래스만."""
    out = []
    for lab in labels:
        if lab.class_name not in OUTPUT_CLASSES:
            continue
        side = min(lab.bbox[2] - lab.bbox[0], lab.bbox[3] - lab.bbox[1])
        if lab.kept:
            required = (lab.visible >= 0.5 and lab.truncated <= 0.3 and side >= min_px
                        and lab.distance <= max_dist)
        elif lab.reason in ('occluded', 'truncated', 'small') and lab.pixels > 0:
            required = False
        else:
            continue
        out.append((lab.class_name, lab.bbox, required, lab.distance))
    return out


def tally(stats: Dict[str, dict], gts: List[tuple], dets: List[tuple], iou_thresh: float
          ) -> None:
    """한 프레임의 매칭 결과를 클래스·거리 구간별로 누적한다."""
    from amr_perception.world_objects import match_detections
    gm, dm = match_detections([g[:3] for g in gts], dets, iou_thresh)
    for (cls, _, req, dist), m in zip(gts, gm):
        if not req:
            continue
        s = stats[cls]
        s['gt'] += 1
        s['tp'] += m >= 0
        for lo, hi in DIST_BINS:
            if lo <= dist < hi:
                b = s['bins'].setdefault(f'{lo:g}-{hi:g}m', [0, 0])
                b[0] += 1
                b[1] += m >= 0
    for (cls, _, _), m in zip(dets, dm):
        if cls in stats:
            stats[cls]['fp'] += m < 0


def summarize(stats: Dict[str, dict]) -> Dict[str, dict]:
    out = {}
    for cls, s in stats.items():
        tp, gt, fp = s['tp'], s['gt'], s['fp']
        out[cls] = {'gt': gt, 'tp': tp, 'fp': fp,
                    'recall': round(tp / gt, 4) if gt else None,
                    'precision': round(tp / (tp + fp), 4) if tp + fp else None,
                    'recall_by_distance': {k: {'gt': v[0], 'recall': round(v[1] / v[0], 3)}
                                           for k, v in sorted(s['bins'].items()) if v[0]}}
    return out


def run(args) -> dict:  # pragma: no cover - Gazebo + yolo_node 필요 (perception.md §8.3)
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    import tf2_ros
    from vision_msgs.msg import Detection2DArray

    from amr_perception.object_localizer_node import depth_to_meters
    from amr_perception.transforms import quat_to_matrix, transform_from_msg
    from amr_perception import world_objects as wo

    cap = load_capture_module()
    rng = random.Random(args.seed)
    with open(args.world, 'r', encoding='utf-8') as f:
        sdf = f.read()
    model_dirs = [os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(args.world))),
                               'models')]
    statics, actors = wo.parse_world(sdf, model_dirs)
    dyn = wo.dynamic_templates(sdf, model_dirs, cap.DYNAMIC_MODELS)
    free = cap.FreeSpace.from_yaml(args.map, args.clearance)
    params = wo.LabelParams()

    rclpy.init()
    node = Node('yolo_live_eval', parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)
    r = args.robot
    s = {'rgb_stamps': [], 'depth': {}, 'dets': {}, 'odom': [], 'k': None, 'cam': None}
    veh = {n: [] for n in dyn}
    qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE)

    def key_of(msg) -> int:
        return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

    def keep(d: dict, key, value, n: int) -> None:
        d[key] = value
        while len(d) > n:
            d.pop(min(d))

    def on_rgb(m):
        s['rgb_stamps'].append((time.monotonic(), key_of(m)))
        del s['rgb_stamps'][:-400]

    def on_det(m):
        keep(s['dets'], key_of(m), (time.monotonic(), m), 60)

    node.create_subscription(Image, f'/{r}/camera/image_raw', on_rgb, qos)
    node.create_subscription(Image, f'/{r}/camera/depth/image_raw',
                             lambda m: keep(s['depth'], key_of(m), m, 8), qos)
    node.create_subscription(CameraInfo, f'/{r}/camera/camera_info',
                             lambda m: s.__setitem__('k', m), 1)
    node.create_subscription(Detection2DArray, f'/{r}/{args.detections}', on_det, 50)

    def on_odom(m):
        s['odom'].append((key_of(m), m.pose.pose))
        del s['odom'][:-100]
    node.create_subscription(Odometry, f'/{r}/ground_truth/odom', on_odom, 50)
    for name in dyn:
        def on_veh(m, name=name):
            veh[name].append((key_of(m), m.pose.pose))
            del veh[name][:-100]
        node.create_subscription(Odometry, f'/sim/{name}/odom', on_veh, 50)

    def spin_for(sec: float) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def yaw_of(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def nearest(samples, key):
        if not samples:
            return None
        best = min(samples, key=lambda x: abs(x[0] - key))
        return best if abs(best[0] - key) <= 25_000_000 else None

    t_wait = time.monotonic() + 90.0
    while time.monotonic() < t_wait and (s['k'] is None or s['cam'] is None or not s['dets']):
        spin_for(0.2)
        if s['k'] is not None and s['cam'] is None:
            base = f'{r}/base_footprint' if args.prefixed else 'base_footprint'
            try:
                tf = tf_buffer.lookup_transform(base, s['k'].header.frame_id, Time())
            except tf2_ros.TransformException:
                continue
            c = transform_from_msg(tf.transform)
            s['cam'] = wo.homogeneous(c.rotation, c.translation)
    if s['cam'] is None or not s['dets']:
        raise RuntimeError('카메라 TF 또는 detections_2d 없음 (yolo_node 가 떠 있는가)')
    km = s['k']
    kin = wo.Intrinsics(km.k[0], km.k[4], km.k[2], km.k[5], km.width, km.height)

    stats = {c: {'gt': 0, 'tp': 0, 'fp': 0, 'bins': {}} for c in OUTPUT_CLASSES}
    frames = 0
    missing_det = 0
    examples = []
    t0 = time.monotonic()
    for i in range(args.poses):
        now = node.get_clock().now().nanoseconds * 1e-9
        veh_now = {}
        for name in dyn:
            if veh[name]:
                p = veh[name][-1][1]
                veh_now[name] = (p.position.x, p.position.y, yaw_of(p.orientation))
        targets = [(a.name, *a.pose_at(now)[0][:2], 3.0) for a in actors]
        targets += [(o.name, o.center[0], o.center[1], 1.0) for o in statics
                    if o.class_name == 'sign']
        targets += [(o.name, o.center[0], o.center[1], 0.15) for o in statics
                    if o.class_name == 'box']
        pose = cap.sample_pose(rng, free, args.split, targets, [],
                               cap.vehicle_hazards(veh_now), args.target_frac)
        if pose is None:
            continue
        req = cap.pose_vector_request({r: pose[:3]})
        subprocess.run(['ign', 'service', '-s', f'/world/{args.world_name}/set_pose_vector',
                        '--reqtype', 'ignition.msgs.Pose_V', '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '5000', '--req', req], capture_output=True, text=True,
                       timeout=30, check=False)
        t_min = node.get_clock().now().nanoseconds + int(args.settle * 1e9)
        used = 0
        deadline = time.monotonic() + args.frame_timeout
        while used < args.frames_per_pose and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            ready = sorted(k for k in s['depth'] if k >= t_min and k in s['dets'])
            if not ready:
                continue
            key = ready[0]
            t_min = key + 1
            od = nearest(s['odom'], key)
            if od is None:
                continue
            pose_msg = od[1]
            q = pose_msg.orientation
            dev = math.hypot(pose_msg.position.x - pose[0], pose_msg.position.y - pose[1])
            if dev > 0.03:
                continue
            t_wb = wo.homogeneous(quat_to_matrix((q.x, q.y, q.z, q.w)),
                                  (pose_msg.position.x, pose_msg.position.y,
                                   pose_msg.position.z))
            r_cw, t_cw = wo.world_to_camera(t_wb, s['cam'])
            objs = list(statics) + wo.actor_objects(actors, key * 1e-9, cap.PERSON_BOX)
            for name, tmpl in dyn.items():
                v = nearest(veh[name], key)
                if v is not None:
                    vp = v[1]
                    objs.append(tmpl.at(vp.position.x, vp.position.y, yaw_of(vp.orientation)))
            labels = wo.label_objects(objs, r_cw, t_cw, kin, depth_to_meters(s['depth'][key]),
                                      params)
            gts = gt_entries(labels, args.min_px, args.max_dist)
            det_msg = s['dets'][key][1]
            dets = []
            for d in det_msg.detections:
                cls = d.results[0].hypothesis.class_id if d.results else ''
                cx, cy = d.bbox.center.position.x, d.bbox.center.position.y
                dets.append((cls, (cx - d.bbox.size_x / 2, cy - d.bbox.size_y / 2,
                                   cx + d.bbox.size_x / 2, cy + d.bbox.size_y / 2),
                             d.results[0].hypothesis.score if d.results else 0.0))
            tally(stats, gts, dets, args.iou)
            frames += 1
            used += 1
            if len(examples) < 20 and any(g[2] for g in gts):
                examples.append({'pose': [round(v, 3) for v in pose[:3]], 'stamp': key * 1e-9,
                                 'gt': [(g[0], [round(v) for v in g[1]], g[2], round(g[3], 1))
                                        for g in gts],
                                 'det': [(d[0], [round(v) for v in d[1]], round(d[2], 2))
                                         for d in dets]})
        if used == 0:
            missing_det += 1
        if i % 10 == 0:
            print(f'[{i}] {time.monotonic() - t0:.0f} s, frames {frames}, '
                  + ', '.join(f'{c} {v["tp"]}/{v["gt"]} fp {v["fp"]}' for c, v in stats.items()),
                  flush=True)
    # 처리율: 마지막 검출 60 개가 걸친 스탬프 구간에서 카메라 프레임 대비 검출 발행 비율
    wall = time.monotonic() - t0
    det_keys = set(s['dets'])
    lo, hi = (min(det_keys), max(det_keys)) if det_keys else (0, -1)
    cam_keys = {k for _, k in s['rgb_stamps'] if lo <= k <= hi}
    overlap = len(cam_keys & det_keys)
    node.destroy_node()
    rclpy.shutdown()
    return {'robot': r, 'split': args.split, 'poses': args.poses, 'frames': frames,
            'poses_without_frames': missing_det, 'seed': args.seed, 'min_px': args.min_px,
            'max_dist': args.max_dist, 'iou': args.iou, 'wall_s': round(wall, 1),
            'per_class': summarize(stats),
            'last_window_detection_ratio': round(overlap / max(len(cam_keys), 1), 3)
            if cam_keys else None,
            'examples': examples}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', required=True)
    ap.add_argument('--world-name', default='warehouse')
    ap.add_argument('--map', required=True)
    ap.add_argument('--robot', default='amr_01')
    ap.add_argument('--prefixed', action='store_true', help='TF 프레임 접두사 <robot>/')
    ap.add_argument('--detections', default='perception/detections_2d')
    ap.add_argument('--split', default='test', choices=['train', 'val', 'test'])
    ap.add_argument('--poses', type=int, default=150)
    ap.add_argument('--frames-per-pose', type=int, default=2)
    ap.add_argument('--seed', type=int, default=101)
    ap.add_argument('--clearance', type=float, default=0.55)
    ap.add_argument('--target-frac', type=float, default=0.5)
    ap.add_argument('--settle', type=float, default=0.25)
    ap.add_argument('--frame-timeout', type=float, default=6.0)
    ap.add_argument('--min-px', type=float, default=16.0, help='필수 GT 짧은 변 하한 [px]')
    ap.add_argument('--max-dist', type=float, default=10.0, help='필수 GT 거리 상한 [m] (깊이 범위)')
    ap.add_argument('--iou', type=float, default=0.5)
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)
    res = run(args)
    text = json.dumps(res, indent=1, ensure_ascii=False)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
