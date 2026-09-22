#!/usr/bin/env python3
r"""
YOLOv8 미세조정 (ultralytics) — generate_dataset.py 가 만든 data.yaml 로 학습하고 val 분할 mAP 를 보고한다.

    train.py --data /tmp/ds/data.yaml --model yolov8n.pt --epochs 60 --imgsz 640 --device 0 \
             --out src/amr_perception/models/yolov8n_warehouse.pt
결과: <project>/<name>/weights/best.pt (→ --out 으로 복사), metrics.json (mAP50, mAP50-95, 클래스별 AP50)
yolo_node 는 weights:=yolov8n_warehouse.pt 로 바꾸면 되고, config/classes.yaml 의 model_class_map 이
box/person/sign 을 그대로 통과시킨다.
"""

import argparse
import json
import os
import shutil
import sys
import time


def resolve_model(name: str) -> str:
    """사전학습 가중치: 경로가 있으면 그대로, 없으면 패키지 models/ 와 $ROS_WS 에서 찾는다."""
    from amr_perception.yolo_backend import resolve_weights
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(os.path.join(get_package_share_directory('amr_perception'), 'models'))
    except Exception:  # noqa: BLE001 - 설치 전 실행
        pass
    return resolve_weights(name, dirs)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', required=True, help='data.yaml')
    ap.add_argument('--model', default='yolov8n.pt', help='시작 가중치 (COCO 사전학습)')
    ap.add_argument('--epochs', type=int, default=60)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--batch', type=int, default=16)
    ap.add_argument('--device', default='0', help="'0' (GPU 0) 또는 'cpu'")
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--project', default='/tmp/amr_yolo_runs')
    ap.add_argument('--name', default='warehouse')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', default='', help='best.pt 복사 경로 (비우면 복사 안 함)')
    ap.add_argument('--amp', action='store_true', help='혼합 정밀도 (기본 끔: AMP 점검이 네트워크를 쓴다)')
    args = ap.parse_args(argv)

    from ultralytics import YOLO

    model = YOLO(resolve_model(args.model))
    t0 = time.time()
    model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
                device=args.device, workers=args.workers, project=args.project, name=args.name,
                exist_ok=True, seed=args.seed, deterministic=True, amp=args.amp, plots=False,
                verbose=False)
    train_s = time.time() - t0
    run_dir = os.path.join(args.project, args.name)
    best = os.path.join(run_dir, 'weights', 'best.pt')
    trained = YOLO(best if os.path.exists(best) else os.path.join(run_dir, 'weights', 'last.pt'))
    val = trained.val(data=args.data, split='val', imgsz=args.imgsz, device=args.device,
                      batch=args.batch, plots=False, verbose=False, project=args.project,
                      name=f'{args.name}_val', exist_ok=True)   # 현재 디렉토리에 runs/ 를 만들지 않는다
    names = trained.names
    per_class = {}
    try:
        for i, c in enumerate(val.box.ap_class_index):
            per_class[names[int(c)]] = float(val.box.ap50[i])
    except (AttributeError, IndexError):
        pass
    metrics = {
        'data': os.path.abspath(args.data), 'model': args.model, 'epochs': args.epochs,
        'imgsz': args.imgsz, 'device': args.device, 'train_seconds': round(train_s, 1),
        'mAP50': float(val.box.map50), 'mAP50_95': float(val.box.map),
        'precision': float(val.box.mp), 'recall': float(val.box.mr), 'ap50_per_class': per_class,
        'weights': best,
    }
    with open(os.path.join(run_dir, 'metrics.json'), 'w', encoding='utf-8') as f:
        json.dump(metrics, f, indent=1)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        shutil.copy(metrics['weights'], args.out)
        metrics['copied_to'] = os.path.abspath(args.out)
    print(json.dumps(metrics, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
