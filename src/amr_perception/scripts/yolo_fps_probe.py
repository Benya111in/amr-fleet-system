#!/usr/bin/env python3
"""
yolo_node 종단 처리율(FPS)·지연 측정기.

능동 (기본): 640×480 이미지를 --rate Hz 로 camera/image_raw 에 발행하고 perception/detections_2d 를 센다.
    ros2 run amr_perception yolo_fps_probe.py --rate 60 --duration 30 \
        --images src/amr_perception/models/heldout/images
    이미지는 --images 디렉토리의 jpg (Gazebo 보류 프레임) 또는 합성 장면(SyntheticSceneRenderer) --variety 장을 돌려 쓴다.
    FPS = 받은 detections_2d 수 / 측정 시간. 입력이 처리 용량보다 빠르면 yolo_node 의 depth-1 best-effort 구독이
    오래된 프레임을 버리므로 출력 FPS = 용량이다 (30 Hz 입력만으로는 30 을 넘는 여유를 볼 수 없다 — 60 Hz 도 잰다).
수동 (--passive): 발행하지 않고 실제 카메라(Gazebo 브리지) camera/image_raw 와 detections_2d 를 함께 받아
    카메라 도착률, 검출 발행률, 같은 스탬프 쌍의 벽시계 지연(카메라 수신 → 검출 수신)을 잰다 (브리지 → yolo_node 종단).
지연 = 수신 시각 − 이미지 스탬프 (능동: 같은 벽시계), 수동: 같은 스탬프의 카메라 수신 → 검출 수신.
"""

import argparse
import glob
import json
import os
import sys
import time


def load_frames(images: str, variety: int):
    """--images 의 jpg (정렬, 최대 variety 장) 또는 합성 장면."""
    import cv2
    if images:
        paths = sorted(glob.glob(os.path.join(images, '*.jpg')))[:variety]
        if not paths:
            raise FileNotFoundError(f'{images} 에 jpg 없음')
        return [cv2.imread(p) for p in paths]
    from amr_perception.synthetic import SyntheticSceneRenderer
    renderer = SyntheticSceneRenderer(seed=11)
    return [renderer.render()[0] for _ in range(variety)]


def percentile(values, q: float) -> float:
    import numpy as np
    return round(float(np.nanpercentile(values, q)), 1) if len(values) else float('nan')


def main(argv=None) -> int:  # pragma: no cover - ROS 통합 시험 (docs/algorithms/perception.md §8.2)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rate', type=float, default=30.0, help='[Hz] 입력 이미지 주기 (능동)')
    ap.add_argument('--duration', type=float, default=30.0, help='[s] 측정 시간')
    ap.add_argument('--warmup', type=float, default=5.0, help='[s] 측정 전 예열')
    ap.add_argument('--variety', type=int, default=20, help='돌려 쓸 이미지 수')
    ap.add_argument('--images', default='', help='jpg 디렉토리 (비우면 합성 장면)')
    ap.add_argument('--passive', action='store_true', help='실제 카메라 → yolo_node 종단 측정')
    ap.add_argument('--image-topic', default='camera/image_raw')
    ap.add_argument('--detections-topic', default='perception/detections_2d')
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)

    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray

    rclpy.init()
    node = Node('yolo_fps_probe')
    recv = []          # (수신 monotonic, 지연 s, 검출 수, 스탬프 키)
    cam = {}           # 수동: 스탬프 키 → 수신 monotonic
    sent = [0]
    measuring = [False]

    def key_of(msg) -> int:
        return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

    def on_det(msg):
        if not measuring[0]:
            return
        now_m = time.monotonic()
        if args.passive:
            lat = now_m - cam[key_of(msg)] if key_of(msg) in cam else float('nan')
        else:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            lat = node.get_clock().now().nanoseconds * 1e-9 - stamp
        recv.append((now_m, lat, len(msg.detections), key_of(msg)))

    node.create_subscription(Detection2DArray, args.detections_topic, on_det, 50)
    if args.passive:
        def on_cam(msg):
            if measuring[0]:
                cam[key_of(msg)] = time.monotonic()
        node.create_subscription(Image, args.image_topic, on_cam, qos_profile_sensor_data)
    else:
        bridge = CvBridge()
        msgs = [bridge.cv2_to_imgmsg(f, encoding='bgr8')
                for f in load_frames(args.images, args.variety)]
        pub = node.create_publisher(Image, args.image_topic, qos_profile_sensor_data)

        def tick():
            msg = msgs[sent[0] % len(msgs)]
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_optical_frame'
            pub.publish(msg)
            if measuring[0]:
                sent[0] += 1

        node.create_timer(1.0 / args.rate, tick)
    t_end = time.monotonic() + args.warmup
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.002)
    measuring[0] = True
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.002)
    elapsed = time.monotonic() - t0
    node.destroy_node()
    rclpy.shutdown()

    lat = np.array([r[1] for r in recv]) * 1e3
    result = {'mode': 'passive' if args.passive else 'active', 'duration_s': round(elapsed, 2),
              'detections_msgs': len(recv), 'output_fps': round(len(recv) / elapsed, 2),
              'latency_ms_p50': percentile(lat, 50), 'latency_ms_p95': percentile(lat, 95),
              'mean_detections': round(float(np.mean([r[2] for r in recv])), 2) if recv else 0.0}
    if args.passive:
        matched = sum(1 for r in recv if r[3] in cam)
        result.update({'camera_msgs': len(cam), 'camera_fps': round(len(cam) / elapsed, 2),
                       'processed_fraction': round(matched / max(len(cam), 1), 3)})
    else:
        result.update({'input_rate_hz': args.rate, 'images_sent': sent[0],
                       'input_fps_actual': round(sent[0] / elapsed, 2),
                       'images': args.images or 'synthetic'})
    text = json.dumps(result, indent=1)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
