#!/usr/bin/env python3
r"""
YOLO 미세조정 데이터셋 생성기 (box / person / sign, YOLO txt 형식).

두 가지 원천
  synthetic  Gazebo 없이 cv2 도형 장면을 그린다 (파이프라인 검증·폴백).
      generate_dataset.py synthetic --out /tmp/ds --train 60 --val 20 --seed 0
  gazebo     실행 중인 시뮬레이션의 카메라 영상에 지면 진실 라벨을 단다. 물체 3D 박스는 월드 SDF
             (박스 모델 include + 작업자 actor 궤적, amr_perception.world_objects) 에서, 카메라 자세는
             ground_truth/odom(world → base_footprint) ∘ TF(base_footprint → optical) 로
             구해 8 꼭짓점을 투영하고, 깊이 이미지로 가림(occlusion) 비율을 검사해 많이 가려진 물체는 뺀다.
      ros2 run amr_perception generate_dataset.py gazebo --out /tmp/ds_gz \
          --world src/amr_simulation/worlds/warehouse.sdf --namespace /amr_01 --frames 2000
출력: <out>/images/{train,val}/*.jpg, <out>/labels/{train,val}/*.txt, <out>/data.yaml (+ stats.json)
"""

import argparse
import json
import math
import os
import sys

import numpy as np

from amr_perception.synthetic import (CLASSES, generate_synthetic_dataset, Label, occlusion_ratio,
                                      project_object, write_data_yaml, write_sample, box_corners)


def run_synthetic(args) -> dict:
    stats = generate_synthetic_dataset(args.out, args.train, args.val, args.seed)
    return stats


def run_gazebo(args) -> dict:  # pragma: no cover - 시뮬레이션 필요 (docs/algorithms/perception.md §5)
    import rclpy
    from cv_bridge import CvBridge
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    import tf2_ros

    from amr_perception.object_localizer_node import depth_to_meters
    from amr_perception.transforms import Transform, transform_from_msg
    from amr_perception.world_objects import actor_objects, parse_world

    with open(args.world, 'r', encoding='utf-8') as f:
        sdf = f.read()
    model_dirs = [os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(args.world))),
                               'models')] + list(args.model_dir)
    statics, actors = parse_world(sdf, model_dirs)
    ns = args.namespace.rstrip('/')

    class Collector(Node):
        def __init__(self):
            super().__init__('dataset_collector')
            self.bridge = CvBridge()
            self.k = None
            self.gt = None
            self.cam_tf = None
            self.count = 0
            self.saved = 0
            self.stats = {'train_images': 0, 'val_images': 0}
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.create_subscription(CameraInfo, f'{ns}/camera/camera_info', self.on_info,
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, f'{ns}/ground_truth/odom', self.on_gt, 10)
            img = Subscriber(self, Image, f'{ns}/camera/image_raw',
                             qos_profile=qos_profile_sensor_data)
            dep = Subscriber(self, Image, f'{ns}/camera/depth/image_raw',
                             qos_profile=qos_profile_sensor_data)
            self.sync = ApproximateTimeSynchronizer([img, dep], 10, 0.02)
            self.sync.registerCallback(self.on_frame)

        def on_info(self, msg):
            self.k = np.asarray(msg.k).reshape(3, 3)

        def on_gt(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.gt = Transform.from_quat((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

        def on_frame(self, img_msg, depth_msg):
            self.count += 1
            if self.k is None or self.gt is None or self.count % args.every:
                return
            if self.cam_tf is None:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        args.base_frame, img_msg.header.frame_id, Time())
                except tf2_ros.TransformException:
                    return
                self.cam_tf = transform_from_msg(tf.transform)
            world_cam = self.gt.compose(self.cam_tf).inverse()
            t = Time.from_msg(img_msg.header.stamp).nanoseconds * 1e-9
            bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            depth = depth_to_meters(depth_msg)
            h, w = bgr.shape[:2]
            labels = []
            for obj in statics + actor_objects(actors, t):
                if np.linalg.norm(obj.center[:2] - self.gt.translation[:2]) > args.max_range:
                    continue
                corners = np.array([world_cam.apply(c) for c in box_corners(obj)])
                proj = project_object(corners, self.k[0, 0], self.k[1, 1], self.k[0, 2],
                                      self.k[1, 2], w, h)
                if proj is None or proj.truncated > args.max_truncation:
                    continue
                x1, y1, x2, y2 = proj.bbox
                if min(x2 - x1, y2 - y1) < args.min_px:
                    continue
                if depth is not None and occlusion_ratio(
                        depth, proj.bbox, proj.depth - 0.5 * float(np.max(obj.size)),
                        args.occlusion_tolerance) > args.max_occlusion:
                    continue
                labels.append(Label(CLASSES.index(obj.class_name), proj.bbox))
            if not labels and not args.keep_empty:
                return
            split = 'val' if (self.saved % max(int(round(1.0 / args.val_fraction)), 2)) == 0 \
                else 'train'
            write_sample(args.out, split, self.saved, bgr, labels)
            self.stats[f'{split}_images'] += 1
            for lab in labels:
                key = f'{split}_{CLASSES[lab.class_id]}'
                self.stats[key] = self.stats.get(key, 0) + 1
            self.saved += 1

    rclpy.init()
    node = Collector()
    try:
        while rclpy.ok() and node.saved < args.frames:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    write_data_yaml(args.out)
    stats = dict(node.stats)
    node.destroy_node()
    rclpy.shutdown()
    return stats


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='source', required=True)
    s = sub.add_parser('synthetic', help='cv2 합성 장면')
    s.add_argument('--out', required=True)
    s.add_argument('--train', type=int, default=60)
    s.add_argument('--val', type=int, default=20)
    s.add_argument('--seed', type=int, default=0)
    g = sub.add_parser('gazebo', help='실행 중인 Gazebo 카메라 + 지면 진실')
    g.add_argument('--out', required=True)
    g.add_argument('--world', required=True, help='월드 SDF (박스 include + actor 궤적)')
    g.add_argument('--model-dir', action='append', default=[], help='model:// 검색 경로 추가')
    g.add_argument('--namespace', default='/amr_01')
    g.add_argument('--base-frame', default='base_footprint',
                   help='ground_truth/odom 의 child 프레임 (접두사 포함)')
    g.add_argument('--frames', type=int, default=2000, help='저장할 이미지 수')
    g.add_argument('--every', type=int, default=5, help='N 프레임마다 1 장')
    g.add_argument('--val-fraction', type=float, default=0.2)
    g.add_argument('--max-range', type=float, default=12.0, help='[m] 이보다 먼 물체는 무시')
    g.add_argument('--min-px', type=float, default=10.0, help='라벨 최소 변 [px]')
    g.add_argument('--max-truncation', type=float, default=0.5)
    g.add_argument('--max-occlusion', type=float, default=0.6)
    g.add_argument('--occlusion-tolerance', type=float, default=0.3, help='[m]')
    g.add_argument('--keep-empty', action='store_true', help='라벨 없는 프레임도 저장 (배경)')
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    stats = run_synthetic(args) if args.source == 'synthetic' else run_gazebo(args)
    with open(os.path.join(args.out, 'stats.json'), 'w', encoding='utf-8') as f:
        json.dump(stats, f, indent=1)
    print(json.dumps(stats, indent=1))
    return 0 if math.isfinite(sum(stats.values())) else 1


if __name__ == '__main__':
    sys.exit(main())
