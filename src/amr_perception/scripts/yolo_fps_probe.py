#!/usr/bin/env python3
"""
yolo_node 종단 처리율(FPS)·지연 측정기 — 640×480 이미지를 rate Hz 로 발행하고 detections_2d 를 센다.

    ros2 run amr_perception yolo_fps_probe.py --rate 30 --duration 30 --out /tmp/fps_gpu.json

이미지는 합성 장면(SyntheticSceneRenderer) 중 --variety 장을 돌려 쓴다 (추론 부하가 실제와 비슷하게
검출 수가 바뀐다). FPS = 받은 detections_2d 수 / 측정 시간, 지연 = 수신 시각 − 이미지 스탬프(같은 시계).
입력보다 느리면 yolo_node 의 depth-1 구독이 오래된 프레임을 버리므로 FPS < rate 가 된다.
"""

import argparse
import json
import sys
import time


def main(argv=None) -> int:  # pragma: no cover - ROS 통합 시험 (docs/algorithms/perception.md §6)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rate', type=float, default=30.0, help='[Hz] 입력 이미지 주기')
    ap.add_argument('--duration', type=float, default=30.0, help='[s] 측정 시간')
    ap.add_argument('--warmup', type=float, default=5.0, help='[s] 측정 전 예열')
    ap.add_argument('--variety', type=int, default=20, help='돌려 쓸 합성 이미지 수')
    ap.add_argument('--out', default='')
    args = ap.parse_args(argv)

    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray

    from amr_perception.synthetic import SyntheticSceneRenderer

    renderer = SyntheticSceneRenderer(seed=11)
    frames = [renderer.render()[0] for _ in range(args.variety)]
    rclpy.init()
    node = Node('yolo_fps_probe')
    bridge = CvBridge()
    pub = node.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
    recv = []          # (수신 wall, 지연 s, 검출 수)
    sent = [0]
    measuring = [False]

    def on_det(msg):
        if not measuring[0]:
            return
        now = node.get_clock().now()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        recv.append((time.monotonic(), now.nanoseconds * 1e-9 - stamp, len(msg.detections)))

    node.create_subscription(Detection2DArray, 'perception/detections_2d', on_det, 50)

    def tick():
        msg = bridge.cv2_to_imgmsg(frames[sent[0] % len(frames)], encoding='bgr8')
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        pub.publish(msg)
        if measuring[0]:
            sent[0] += 1

    node.create_timer(1.0 / args.rate, tick)
    t_end = time.monotonic() + args.warmup
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.005)
    measuring[0] = True
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.002)
    elapsed = time.monotonic() - t0
    node.destroy_node()
    rclpy.shutdown()

    lat = np.array([r[1] for r in recv]) * 1e3 if recv else np.array([np.nan])
    result = {
        'input_rate_hz': args.rate, 'duration_s': round(elapsed, 2), 'images_sent': sent[0],
        'detections_msgs': len(recv),
        'input_fps_actual': round(sent[0] / elapsed, 2),
        'output_fps': round(len(recv) / elapsed, 2),
        'latency_ms_p50': round(float(np.nanpercentile(lat, 50)), 1),
        'latency_ms_p95': round(float(np.nanpercentile(lat, 95)), 1),
        'mean_detections': round(float(np.mean([r[2] for r in recv])), 2) if recv else 0.0,
    }
    text = json.dumps(result, indent=1)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text)
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
