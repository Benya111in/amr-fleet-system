#!/usr/bin/env python3
r"""
Gazebo 창고 월드 YOLO 데이터셋 수집기 — 로봇 여러 대를 창고 곳곳으로 순간이동시키며 카메라 영상에 깊이 검증 지면 진실 라벨을 단다.

    # 1) 시뮬레이션: 월드(일시정지) + 로봇 N 대 스폰 + 해제 (카메라 = 배포와 같은 URDF, 전면 0.25 m, RGB 노이즈 켬)
    #    라벨 검증용으로 깊이 카메라 range_max 만 60 m 로 늘린 sensors.yaml 사본을 config_dir 로 준다 (RGB 는 그대로)
    # 2) 수집 (워크스페이스 소싱 후)
    python3 src/amr_perception/scripts/dataset_capture.py --out /data/ds --steps 1600 \
        --world src/amr_simulation/worlds/warehouse.sdf --map maps/warehouse.yaml \
        --robots amr_01,amr_02,amr_03,amr_04,amr_05

방법 (docs/algorithms/perception.md §5)
  - 분할 = 창고 구역 (카메라 위치 기준, 구역 사이 3 m 완충대에는 로봇을 두지 않는다 → 분할 간 근사 중복 프레임 없음)
      test  x ≥ 15            (동측: 출고 도크·대기 구역·동측 통로, 표지판 8 종 판)
      val   x ≤ −14, y ≤ −6   (남서: 충전소)
      train 나머지 − 완충대    (x ≤ 12, 서측 구역은 y ≥ −3)
  - 자세: 지도(maps/warehouse.yaml, map = world) 자유 셀 중 벽·랙까지 clearance 이상인 곳. 절반은 표적(작업자·표지판·
    지게차·셔틀·상자) 쪽을 1.2–8 m 에서 바라보는 자세, 나머지는 무작위 방향. 지게차·셔틀 진행 경로 앞과 다른 로봇 근처는 뺀다.
  - 스텝마다 /world/<w>/set_pose_vector 로 N 대를 한 번에 옮기고, sim settle 초 뒤 같은 스탬프의 RGB·깊이 쌍과
    ground_truth/odom 을 받는다. 명령 자세와 3 cm / 1° 이상 다르면(충돌) 버린다.
  - 라벨: amr_perception.world_objects.label_objects — 정적 물체(상자 include, 인라인 표지판 판), 작업자(actor 궤적,
    이미지 스탬프 시각), 지게차·셔틀(/sim/<이름>/odom), 다른 로봇(ground_truth/odom) 의 3D 박스를 투영하고 깊이로 가림·잘림 판정.
출력: <out>/images/<split>/*.jpg, labels/<split>/*.txt (YOLO), meta/<split>.jsonl (자세·판정 전부),
     data.yaml, stats_<start_step>.json
"""

import argparse
import json
import math
import os
import random
import subprocess
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# 창고 구역 분할 (카메라 위치 x, y). 사각형 [x_min, x_max, y_min, y_max] 의 합집합.
SPLIT_REGIONS: Dict[str, List[Tuple[float, float, float, float]]] = {
    'test': [(15.0, 31.0, -21.0, 21.0)],
    'val': [(-31.0, -14.0, -21.0, -6.0)],
    'train': [(-31.0, 12.0, -3.0, 21.0), (-11.0, 12.0, -21.0, -3.0)],
}
SPLIT_WEIGHTS = {'train': 0.70, 'val': 0.12, 'test': 0.18}
DYNAMIC_MODELS = {'forklift_main': 'forklift', 'shuttle_amr': 'amr'}
# 로봇(amr) 라벨 박스: base_footprint 기준 차체 0.60 × 0.40, 지면 0 ~ 데크 0.33 m (config/robot_params.yaml)
ROBOT_BOX = ((0.0, 0.0, 0.165), (0.60, 0.40, 0.33))
PERSON_BOX = (0.7, 0.7, 1.9)      # 작업자 메시(신장 1.75 m + 안전모, 폭 0.56 m) + 팔 흔듦·자세 오차 여유


def region_of(x: float, y: float, regions=None) -> Optional[str]:
    """카메라 위치 → 분할 이름 (완충대면 None)."""
    for name, rects in (regions or SPLIT_REGIONS).items():
        for x0, x1, y0, y1 in rects:
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
    return None


class FreeSpace:
    """점유 격자(map_server YAML) 의 자유 셀 중 장애물까지 clearance 이상인 셀 표본기."""

    def __init__(self, grid_free: np.ndarray, resolution: float, origin: Sequence[float],
                 clearance: float):
        import cv2
        dist = cv2.distanceTransform(grid_free.astype(np.uint8), cv2.DIST_L2, 5) * resolution
        self.ok = dist >= clearance
        self.res = resolution
        self.origin = (float(origin[0]), float(origin[1]))
        self.h = grid_free.shape[0]
        rows, cols = np.nonzero(self.ok)
        self.cells = np.stack([cols, rows], axis=1)

    @staticmethod
    def from_yaml(path: str, clearance: float) -> 'FreeSpace':
        import cv2
        import yaml
        with open(path, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        img = cv2.imread(os.path.join(os.path.dirname(path), cfg['image']), cv2.IMREAD_GRAYSCALE)
        occ = (255 - img.astype(np.float64)) / 255.0
        if cfg.get('negate', 0):
            occ = 1.0 - occ
        free = occ < float(cfg.get('free_thresh', 0.25))
        return FreeSpace(free, float(cfg['resolution']), cfg['origin'], clearance)

    def is_free(self, x: float, y: float) -> bool:
        c = int((x - self.origin[0]) / self.res)
        r = self.h - 1 - int((y - self.origin[1]) / self.res)
        return 0 <= r < self.ok.shape[0] and 0 <= c < self.ok.shape[1] and bool(self.ok[r, c])

    def sample(self, rng: random.Random) -> Tuple[float, float]:
        c, r = self.cells[rng.randrange(len(self.cells))]
        x = self.origin[0] + (c + rng.random()) * self.res
        y = self.origin[1] + (self.h - 1 - r + rng.random()) * self.res
        return float(x), float(y)


def pose_is_safe(x: float, y: float, placed: Sequence[Tuple[float, float]],
                 hazards: Sequence[Tuple[float, float, float, float, float]],
                 min_sep: float = 1.2) -> bool:
    """
    다른 로봇(placed) 과 min_sep 이상, 움직이는 차량 위험 사각형 밖.

    hazards: (x_min, x_max, y_min, y_max, _) — 지게차·셔틀 현재 위치 앞뒤 진행 경로.
    """
    if any(math.hypot(x - px, y - py) < min_sep for px, py in placed):
        return False
    return not any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1, _ in hazards)


def vehicle_hazards(poses: Dict[str, Tuple[float, float, float]]
                    ) -> List[Tuple[float, float, float, float, float]]:
    """지게차(x 축 왕복, 길이 3.2 m)·셔틀(y 축 왕복) 현재 위치 ± 진행 여유 → 위험 사각형."""
    out = []
    for name, (x, y, _) in poses.items():
        if name.startswith('forklift'):
            out.append((x - 7.0, x + 7.0, y - 1.9, y + 1.9, 0.0))
        else:
            out.append((x - 1.3, x + 1.3, y - 5.0, y + 5.0, 0.0))
    return out


def sample_pose(rng: random.Random, free: FreeSpace, split: str,
                targets: Sequence[Tuple[str, float, float, float]],
                placed: Sequence[Tuple[float, float]], hazards, target_frac: float = 0.5,
                tries: int = 400) -> Optional[Tuple[float, float, float, str]]:
    """
    분할 구역 안의 안전한 자세 (x, y, yaw, 방식). 방식 = 'target:<이름>' 또는 'random'.

    targets: (이름, x, y, 가중치) — 이 방향을 바라보는 자세를 target_frac 확률로 만든다.
    """
    use_target = targets and rng.random() < target_frac
    weights = [t[3] for t in targets] if targets else []
    for _ in range(tries):
        if use_target:
            name, tx, ty, _ = rng.choices(targets, weights=weights)[0]
            r = rng.uniform(1.2, 8.0)
            b = rng.uniform(-math.pi, math.pi)
            x, y = tx - r * math.cos(b), ty - r * math.sin(b)
            yaw = b + rng.gauss(0.0, math.radians(20.0))
            how = f'target:{name}'
        else:
            x, y = free.sample(rng)
            yaw = rng.uniform(-math.pi, math.pi)
            how = 'random'
        if region_of(x, y) != split or not free.is_free(x, y):
            continue
        if not pose_is_safe(x, y, placed, hazards):
            continue
        return x, y, math.atan2(math.sin(yaw), math.cos(yaw)), how
    return None


def pose_vector_request(poses: Dict[str, Tuple[float, float, float]], z: float = 0.005) -> str:
    """순간이동 요청: `ign service` set_pose_vector 의 ignition.msgs.Pose_V 텍스트."""
    items = []
    for name, (x, y, yaw) in poses.items():
        items.append(f'{{name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
                     f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, '
                     f'w: {math.cos(yaw / 2):.6f}}}}}')
    return 'pose: [' + ', '.join(items) + ']'


def yolo_lines(labels, width: int, height: int, classes: Sequence[str]) -> List[str]:
    """채택된 GtLabel → YOLO txt 줄 (cls cx cy w h, 0–1)."""
    out = []
    for lab in labels:
        if not lab.kept:
            continue
        x1, y1, x2, y2 = lab.bbox
        out.append(f'{classes.index(lab.class_name)} {0.5 * (x1 + x2) / width:.6f} '
                   f'{0.5 * (y1 + y2) / height:.6f} {(x2 - x1) / width:.6f} '
                   f'{(y2 - y1) / height:.6f}')
    return out


def write_data_yaml(out: str, classes: Sequence[str]) -> str:
    import yaml
    path = os.path.join(out, 'data.yaml')
    cfg = {'path': os.path.abspath(out), 'train': 'images/train', 'val': 'images/val',
           'test': 'images/test', 'names': {i: n for i, n in enumerate(classes)}}
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return path


def label_record(lab) -> dict:
    return {'cls': lab.class_name, 'name': lab.name,
            'bbox': [round(v, 1) for v in lab.bbox], 'vis': round(lab.visible, 3),
            'trunc': round(lab.truncated, 3), 'dist': round(lab.distance, 2),
            'px': lab.pixels, 'kept': lab.kept, 'reason': lab.reason}


def run(args) -> dict:  # pragma: no cover - Gazebo 필요 (perception.md §5, 실행 기록 §8.3)
    import cv2
    from cv_bridge import CvBridge
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    import tf2_ros

    from amr_perception.object_localizer_node import depth_to_meters
    from amr_perception.synthetic import WorldObject
    from amr_perception.transforms import quat_to_matrix, transform_from_msg
    from amr_perception import world_objects as wo

    rng = random.Random(args.seed)
    with open(args.world, 'r', encoding='utf-8') as f:
        sdf = f.read()
    model_dirs = [os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(args.world))),
                               'models')] + list(args.model_dir)
    statics, actors = wo.parse_world(sdf, model_dirs)
    dyn = wo.dynamic_templates(sdf, model_dirs, DYNAMIC_MODELS)
    free = FreeSpace.from_yaml(args.map, args.clearance)
    classes = wo.DATASET_CLASSES
    params = wo.LabelParams(max_range=args.max_range)
    robots = args.robots.split(',')
    world_name = args.world_name
    print(f'정적 물체 {len(statics)} (sign {sum(o.class_name == "sign" for o in statics)}), '
          f'actor {len(actors)}, 동적 모델 {sorted(dyn)}, 자유 셀 {len(free.cells)}', flush=True)

    rclpy.init()
    node = Node('dataset_capture', parameter_overrides=[Parameter('use_sim_time', value=True)])
    bridge = CvBridge()
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)
    state = {r: {'rgb': {}, 'depth': {}, 'odom': [], 'k': None, 'cam': None} for r in robots}
    veh = {n: [] for n in dyn}
    qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE)

    def stamp_key(msg) -> int:
        return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

    def keep(d: dict, key, value, n: int) -> None:
        d[key] = value
        while len(d) > n:
            d.pop(min(d))

    for r in robots:
        s = state[r]
        node.create_subscription(Image, f'/{r}/camera/image_raw',
                                 lambda m, s=s: keep(s['rgb'], stamp_key(m), m, 6), qos)
        node.create_subscription(Image, f'/{r}/camera/depth/image_raw',
                                 lambda m, s=s: keep(s['depth'], stamp_key(m), m, 4), qos)
        node.create_subscription(CameraInfo, f'/{r}/camera/camera_info',
                                 lambda m, s=s: s.__setitem__('k', m), 1)

        def on_odom(m, s=s):
            s['odom'].append((stamp_key(m), m.pose.pose, m.twist.twist))
            del s['odom'][:-60]
        node.create_subscription(Odometry, f'/{r}/ground_truth/odom', on_odom, 50)
    for name in dyn:
        def on_veh(m, name=name):
            veh[name].append((stamp_key(m), m.pose.pose))
            del veh[name][:-60]
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
        best = min(samples, key=lambda s: abs(s[0] - key))
        return best if abs(best[0] - key) <= 25_000_000 else None

    # 카메라 내부·외부 파라미터 (TF: base_footprint → 광학, 접두사 포함)
    t_wait = time.monotonic() + 60.0
    while time.monotonic() < t_wait and not all(
            state[r]['k'] is not None and state[r]['cam'] is not None for r in robots):
        spin_for(0.2)
        for r in robots:
            if state[r]['cam'] is None and state[r]['k'] is not None:
                pre = f'{r}/' if args.prefixed else ''
                try:
                    tf = tf_buffer.lookup_transform(f'{pre}base_footprint',
                                                    state[r]['k'].header.frame_id, Time())
                except tf2_ros.TransformException:
                    continue
                cam = transform_from_msg(tf.transform)
                state[r]['cam'] = wo.homogeneous(cam.rotation, cam.translation)
    missing = [r for r in robots if state[r]['cam'] is None]
    if missing:
        raise RuntimeError(f'카메라 정보/TF 없음: {missing}')
    km = state[robots[0]]['k']
    kin = wo.Intrinsics(km.k[0], km.k[4], km.k[2], km.k[5], km.width, km.height)
    print(f'K fx={kin.fx:.2f} fy={kin.fy:.2f} cx={kin.cx} cy={kin.cy} {kin.width}x{kin.height}; '
          f'base→optical t={np.round(state[robots[0]]["cam"][:3, 3], 4).tolist()}', flush=True)

    for split in SPLIT_REGIONS:
        os.makedirs(os.path.join(args.out, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(args.out, 'labels', split), exist_ok=True)
    os.makedirs(os.path.join(args.out, 'meta'), exist_ok=True)
    metas = {sp: open(os.path.join(args.out, 'meta', f'{sp}.jsonl'), 'a', encoding='utf-8')
             for sp in SPLIT_REGIONS}
    stats: Dict[str, int] = {}

    def bump(key: str, n: int = 1) -> None:
        stats[key] = stats.get(key, 0) + n

    splits = list(SPLIT_WEIGHTS)
    t_start = time.monotonic()
    for step in range(args.start_step, args.start_step + args.steps):
        now_ns = node.get_clock().now().nanoseconds
        t_now = now_ns * 1e-9
        veh_now = {}
        for name in dyn:
            if veh[name]:
                p = veh[name][-1][1]
                veh_now[name] = (p.position.x, p.position.y, yaw_of(p.orientation))
        hazards = vehicle_hazards(veh_now)
        targets = [(a.name, *a.pose_at(t_now)[0][:2], 3.0) for a in actors]
        targets += [(n, p[0], p[1], 1.5) for n, p in veh_now.items()]
        targets += [(o.name, o.center[0], o.center[1], 1.0) for o in statics
                    if o.class_name == 'sign']
        targets += [(o.name, o.center[0], o.center[1], 0.15) for o in statics
                    if o.class_name == 'box']
        plan = {}
        placed: List[Tuple[float, float]] = []
        for r in robots:
            split = rng.choices(splits, weights=[SPLIT_WEIGHTS[s] for s in splits])[0]
            pose = sample_pose(rng, free, split, targets, placed, hazards, args.target_frac)
            if pose is None:
                continue
            plan[r] = (split, pose)
            placed.append(pose[:2])
        req = pose_vector_request({r: (p[0], p[1], p[2]) for r, (_, p) in plan.items()})
        res = subprocess.run(['ign', 'service', '-s', f'/world/{world_name}/set_pose_vector',
                              '--reqtype', 'ignition.msgs.Pose_V', '--reptype',
                              'ignition.msgs.Boolean', '--timeout', '5000', '--req', req],
                             capture_output=True, text=True, timeout=30, check=False)
        if 'true' not in res.stdout:
            print(f'[{step}] set_pose_vector 실패: {res.stdout.strip()} {res.stderr.strip()}',
                  flush=True)
            bump('teleport_failed')
            spin_for(1.0)
            continue
        t_min = node.get_clock().now().nanoseconds + int(args.settle * 1e9)
        done: Dict[str, bool] = {}
        deadline = time.monotonic() + args.frame_timeout
        while time.monotonic() < deadline and len(done) < len(plan):
            rclpy.spin_once(node, timeout_sec=0.01)
            for r, (split, (px, py, pyaw, how)) in plan.items():
                if r in done:
                    continue
                s = state[r]
                pairs = sorted(k for k in s['depth'] if k >= t_min and k in s['rgb'])
                if not pairs:
                    continue
                key = pairs[0]
                od = nearest(s['odom'], key)
                if od is None:
                    continue
                done[r] = True
                pose, twist = od[1], od[2]
                gx, gy, gyaw = pose.position.x, pose.position.y, yaw_of(pose.orientation)
                dev = math.hypot(gx - px, gy - py)
                dyaw = abs(math.atan2(math.sin(gyaw - pyaw), math.cos(gyaw - pyaw)))
                speed = math.hypot(twist.linear.x, twist.linear.y)
                if dev > 0.03 or dyaw > math.radians(1.0) or speed > 0.05:
                    bump('pose_rejected')
                    continue
                t_img = key * 1e-9
                objs: List[WorldObject] = list(statics)
                objs += wo.actor_objects(actors, t_img, PERSON_BOX)
                for name, tmpl in dyn.items():
                    v = nearest(veh[name], key)
                    if v is not None:
                        vp = v[1]
                        objs.append(tmpl.at(vp.position.x, vp.position.y, yaw_of(vp.orientation)))
                    else:
                        bump(f'no_odom_{name}')
                for other in robots:
                    if other == r:
                        continue
                    o = nearest(state[other]['odom'], key)
                    if o is not None:
                        op = o[1]
                        tmpl = wo.ModelTemplate('amr', np.array(ROBOT_BOX[0]),
                                                np.array(ROBOT_BOX[1]), other)
                        objs.append(tmpl.at(op.position.x, op.position.y, yaw_of(op.orientation)))
                q = pose.orientation
                t_wb = wo.homogeneous(quat_to_matrix((q.x, q.y, q.z, q.w)),
                                      (gx, gy, pose.position.z))
                r_cw, t_cw = wo.world_to_camera(t_wb, s['cam'])
                depth = depth_to_meters(s['depth'][key])
                labels = wo.label_objects(objs, r_cw, t_cw, kin, depth, params)
                lines = yolo_lines(labels, kin.width, kin.height, classes)
                if not lines and rng.random() > args.keep_empty:
                    bump(f'{split}_empty_dropped')
                    continue
                stem = f'{r}_{step:05d}'
                bgr = bridge.imgmsg_to_cv2(s['rgb'][key], desired_encoding='bgr8')
                cv2.imwrite(os.path.join(args.out, 'images', split, stem + '.jpg'), bgr,
                            [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                with open(os.path.join(args.out, 'labels', split, stem + '.txt'), 'w',
                          encoding='utf-8') as f:
                    f.write(''.join(ln + '\n' for ln in lines))
                metas[split].write(json.dumps({
                    'image': f'images/{split}/{stem}.jpg', 'robot': r, 'step': step,
                    'stamp': round(t_img, 3), 'pose': [round(gx, 3), round(gy, 3),
                                                       round(gyaw, 4)],
                    'how': how, 'labels': [label_record(lab) for lab in labels]},
                    ensure_ascii=False) + '\n')
                bump(f'{split}_images')
                if not lines:
                    bump(f'{split}_empty')
                for lab in labels:
                    bump(f'{split}_{lab.class_name}' if lab.kept
                         else f'{split}_{lab.class_name}_rejected_{lab.reason}')
        for r in plan:
            if r not in done:
                bump('frame_timeout')
        if step % 20 == 0:
            el = time.monotonic() - t_start
            print(f'[{step}] {el:.0f} s, sim {node.get_clock().now().nanoseconds * 1e-9:.1f} s, '
                  + ', '.join(f'{k}={v}' for k, v in sorted(stats.items())
                              if k.endswith('_images') or '_rejected' not in k), flush=True)
            for fh in metas.values():
                fh.flush()
    for fh in metas.values():
        fh.close()
    node.destroy_node()
    rclpy.shutdown()
    write_data_yaml(args.out, classes)
    return stats


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', required=True, help='월드 SDF (include·인라인 표지판·actor)')
    ap.add_argument('--world-name', default='warehouse', help='Gazebo <world name>')
    ap.add_argument('--model-dir', action='append', default=[], help='model:// 검색 경로 추가')
    ap.add_argument('--map', required=True, help='점유 격자 YAML (map 프레임 = 월드)')
    ap.add_argument('--robots', default='amr_01,amr_02,amr_03,amr_04,amr_05')
    ap.add_argument('--prefixed', action='store_true', default=True,
                    help='TF 프레임 접두사 <robot>/ (다중 로봇 스폰)')
    ap.add_argument('--no-prefix', dest='prefixed', action='store_false')
    ap.add_argument('--steps', type=int, default=1000, help='순간이동 횟수 (스텝당 로봇 수만큼 이미지)')
    ap.add_argument('--start-step', type=int, default=0, help='이어 받기: 파일 이름 번호 시작')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--clearance', type=float, default=0.55, help='[m] 벽·랙까지 최소 거리')
    ap.add_argument('--target-frac', type=float, default=0.5, help='표적을 바라보는 자세 비율')
    ap.add_argument('--settle', type=float, default=0.25, help='[s, sim] 순간이동 뒤 기다림')
    ap.add_argument('--frame-timeout', type=float, default=8.0, help='[s, wall]')
    ap.add_argument('--max-range', type=float, default=60.0, help='[m] 라벨 거리 상한')
    ap.add_argument('--keep-empty', type=float, default=0.3, help='라벨 없는 프레임을 남길 확률')
    ap.add_argument('--jpeg-quality', type=int, default=95)
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    stats = run(args)
    with open(os.path.join(args.out, f'stats_{args.start_step}.json'), 'w',
              encoding='utf-8') as f:
        json.dump(stats, f, indent=1, sort_keys=True)
    print(json.dumps(stats, indent=1, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
