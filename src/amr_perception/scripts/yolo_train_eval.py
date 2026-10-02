#!/usr/bin/env python3
r"""
YOLOv8 미세조정 + 보류 구역 평가 (dataset_capture.py 데이터셋, 분할 = 창고 구역).

    # 학습 (GPU) → best.pt 를 models/ 로 복사 (옵티마이저 제거·FP16, ultralytics strip)
    yolo_train_eval.py train --data /data/ds/data.yaml --model yolov8n.pt --epochs 80 \
        --device 0 --project /data/runs --name v8n \
        --out src/amr_perception/models/yolov8n_warehouse.pt
    # 평가만: val·test 분할의 클래스별 P / R / mAP50 / mAP50-95 (ultralytics val) + 배포 경로 동작점 재현율
    yolo_train_eval.py eval --data /data/ds/data.yaml --device 0 \
        --weights src/amr_perception/models/yolov8n_warehouse.pt
    # 회귀 시험용 보류 프레임 뽑기 (test 분할에서 클래스별로 고르게)
    yolo_train_eval.py heldout --data /data/ds/data.yaml --per-class 4 \
        --out src/amr_perception/models/heldout

동작점(operating point) 평가는 yolo_node 와 같은 코드 경로(YoloDetector: 레터박스 → 융합 모델 → NMS, GPU FP16 640 /
CPU FP32 320)와 배포 임계값(conf 0.35)으로 모든 클래스를 돌려 IoU ≥ 0.5 탐욕 매칭으로 정밀도·재현율을 센다.
"""

import argparse
import glob
import json
import os
import shutil
import sys
import time
from typing import Dict, List, Sequence


def read_yaml(path: str) -> dict:
    import yaml
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def split_dirs(data_yaml: str, split: str):
    """data.yaml → (이미지 디렉토리, 라벨 디렉토리, 클래스 이름 목록)."""
    cfg = read_yaml(data_yaml)
    root = cfg.get('path') or os.path.dirname(os.path.abspath(data_yaml))
    img_dir = os.path.join(root, cfg[split])
    names = cfg['names']
    names = [names[i] for i in sorted(names)] if isinstance(names, dict) else list(names)
    return img_dir, img_dir.replace(os.sep + 'images', os.sep + 'labels'), names


def read_labels(path: str, width: int, height: int) -> List[tuple]:
    """YOLO txt → [(class_id, (x1, y1, x2, y2) px)]."""
    out = []
    if not os.path.exists(path):
        return out
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            parts = line.split()
            if len(parts) != 5:
                continue
            c, cx, cy, w, h = int(parts[0]), *map(float, parts[1:])
            out.append((c, ((cx - w / 2) * width, (cy - h / 2) * height,
                            (cx + w / 2) * width, (cy + h / 2) * height)))
    return out


def count_labels(data_yaml: str) -> Dict[str, Dict[str, int]]:
    """분할별 이미지 수·클래스별 라벨 수."""
    cfg = read_yaml(data_yaml)
    out = {}
    for split in ('train', 'val', 'test'):
        if split not in cfg:
            continue
        img_dir, lbl_dir, names = split_dirs(data_yaml, split)
        counts = {n: 0 for n in names}
        imgs = sorted(glob.glob(os.path.join(img_dir, '*.jpg')))
        empty = 0
        for p in imgs:
            labs = read_labels(os.path.join(lbl_dir, os.path.splitext(os.path.basename(p))[0]
                                            + '.txt'), 1, 1)
            empty += not labs
            for c, _ in labs:
                counts[names[c]] += 1
        out[split] = dict(images=len(imgs), empty=empty, **counts)
    return out


def min_side_px(box) -> float:
    return min(box[2] - box[0], box[3] - box[1])


def operating_point(weights: str, data_yaml: str, split: str, device: str, conf: float,
                    imgsz: int, min_px: float = 12.0, iou_thresh: float = 0.5) -> dict:
    """
    배포 경로(YoloDetector) 동작점 재현율·정밀도. GT 짧은 변 < min_px 는 무시 GT (맞혀도 FP 아님).

    반환: 클래스별 {gt, tp, fp, recall, precision} + 추론 시간 중앙값.
    """
    import cv2
    import numpy as np

    from amr_perception.world_objects import match_detections
    from amr_perception.yolo_backend import YoloDetector

    img_dir, lbl_dir, names = split_dirs(data_yaml, split)
    det = YoloDetector(weights, device=device, imgsz=imgsz, cpu_imgsz=imgsz, conf=conf)
    det.warmup(runs=2)
    stats = {n: {'gt': 0, 'tp': 0, 'fp': 0} for n in names}
    times = []
    for p in sorted(glob.glob(os.path.join(img_dir, '*.jpg'))):
        img = cv2.imread(p)
        h, w = img.shape[:2]
        gts = [(names[c], b, min_side_px(b) >= min_px)
               for c, b in read_labels(os.path.join(
                   lbl_dir, os.path.splitext(os.path.basename(p))[0] + '.txt'), w, h)]
        t0 = time.perf_counter()
        dets = det.infer(img)
        times.append(time.perf_counter() - t0)
        dl = [(d.class_name, (d.cx - d.width / 2, d.cy - d.height / 2, d.cx + d.width / 2,
                              d.cy + d.height / 2), d.score) for d in dets]
        gm, dm = match_detections(gts, dl, iou_thresh)
        for (cls, _, req), m in zip(gts, gm):
            if req:
                stats[cls]['gt'] += 1
                stats[cls]['tp'] += m >= 0
        for (cls, _, _), m in zip(dl, dm):
            if cls in stats:
                stats[cls]['fp'] += m < 0
    for s in stats.values():
        s['recall'] = round(s['tp'] / s['gt'], 4) if s['gt'] else None
        s['precision'] = round(s['tp'] / (s['tp'] + s['fp']), 4) if (s['tp'] + s['fp']) else None
    return {'split': split, 'device': det.device, 'imgsz': det.imgsz, 'conf': conf,
            'min_gt_px': min_px, 'per_class': stats,
            'infer_ms_p50': round(float(np.median(times)) * 1e3, 2) if times else None}


def ultralytics_val(weights: str, data_yaml: str, split: str, imgsz: int, device: str,
                    project: str) -> dict:
    """Ultralytics val → 전체·클래스별 P / R / mAP50 / mAP50-95."""
    from ultralytics import YOLO
    model = YOLO(weights)
    r = model.val(data=data_yaml, split=split, imgsz=imgsz, device=device, batch=16,
                  plots=False, verbose=False, project=project, name=f'val_{split}_{imgsz}',
                  exist_ok=True)
    names = model.names
    per = {}
    for i, c in enumerate(r.box.ap_class_index):
        p, rec, ap50, ap = r.box.class_result(i)
        per[names[int(c)]] = {'P': round(float(p), 4), 'R': round(float(rec), 4),
                              'mAP50': round(float(ap50), 4), 'mAP50_95': round(float(ap), 4)}
    return {'split': split, 'imgsz': imgsz, 'P': round(float(r.box.mp), 4),
            'R': round(float(r.box.mr), 4), 'mAP50': round(float(r.box.map50), 4),
            'mAP50_95': round(float(r.box.map), 4), 'per_class': per}


def evaluate(weights: str, data_yaml: str, device: str, project: str,
             splits: Sequence[str] = ('val', 'test'), conf: float = 0.35) -> dict:
    out = {'weights': os.path.abspath(weights), 'data': os.path.abspath(data_yaml),
           'counts': count_labels(data_yaml), 'val': [], 'operating_point': []}
    gpu = device not in ('cpu', '')
    for split in splits:
        for imgsz in (640, 320):
            out['val'].append(ultralytics_val(weights, data_yaml, split, imgsz,
                                              device if gpu else 'cpu', project))
        if gpu:
            out['operating_point'].append(operating_point(weights, data_yaml, split, 'cuda', conf,
                                                          640))
        out['operating_point'].append(operating_point(weights, data_yaml, split, 'cpu', conf,
                                                      320))
    return out


def cmd_train(args) -> dict:
    from ultralytics import YOLO
    model = YOLO(args.model)
    t0 = time.time()
    model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
                device=args.device, workers=args.workers, project=args.project, name=args.name,
                exist_ok=True, seed=args.seed, deterministic=False, amp=args.amp, plots=False,
                patience=args.patience, cos_lr=True, close_mosaic=10, verbose=False,
                cache=args.cache or False)
    train_s = time.time() - t0
    run_dir = os.path.join(args.project, args.name)
    best = os.path.join(run_dir, 'weights', 'best.pt')
    metrics = {'model': args.model, 'epochs': args.epochs, 'imgsz': args.imgsz,
               'batch': args.batch, 'train_seconds': round(train_s, 1), 'best': best}
    metrics.update(evaluate(best, args.data, args.device, args.project))
    with open(os.path.join(run_dir, 'metrics.json'), 'w', encoding='utf-8') as f:
        json.dump(metrics, f, indent=1, ensure_ascii=False)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        shutil.copy(best, args.out)
        metrics['copied_to'] = os.path.abspath(args.out)
    return metrics


def cmd_heldout(args) -> dict:
    """Test 분할에서 클래스별로 per_class 장씩 (라벨 많은 순, 서로 다른 스텝) 골라 이미지·라벨을 복사한다."""
    img_dir, lbl_dir, names = split_dirs(args.data, 'test')
    chosen: List[str] = []
    for ci in range(min(args.classes, len(names))):
        scored = []
        for p in sorted(glob.glob(os.path.join(img_dir, '*.jpg'))):
            stem = os.path.splitext(os.path.basename(p))[0]
            labs = read_labels(os.path.join(lbl_dir, stem + '.txt'), 640, 480)
            big = [b for c, b in labs if c == ci and min_side_px(b) >= args.min_px]
            if big and p not in chosen:
                scored.append((len(big), stem.split('_')[-1], p))
        scored.sort(reverse=True)
        steps_used = set()
        n = 0
        for _, step, p in scored:
            if step in steps_used:
                continue
            chosen.append(p)
            steps_used.add(step)
            n += 1
            if n >= args.per_class:
                break
    os.makedirs(os.path.join(args.out, 'images'), exist_ok=True)
    os.makedirs(os.path.join(args.out, 'labels'), exist_ok=True)
    for p in chosen:
        stem = os.path.splitext(os.path.basename(p))[0]
        shutil.copy(p, os.path.join(args.out, 'images', stem + '.jpg'))
        shutil.copy(os.path.join(lbl_dir, stem + '.txt'),
                    os.path.join(args.out, 'labels', stem + '.txt'))
    return {'heldout': args.out, 'images': [os.path.basename(p) for p in chosen]}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    t = sub.add_parser('train')
    t.add_argument('--data', required=True)
    t.add_argument('--model', default='yolov8n.pt', help='시작 가중치 (COCO 사전학습)')
    t.add_argument('--epochs', type=int, default=80)
    t.add_argument('--imgsz', type=int, default=640)
    t.add_argument('--batch', type=int, default=32)
    t.add_argument('--device', default='0')
    t.add_argument('--workers', type=int, default=8)
    t.add_argument('--patience', type=int, default=25)
    t.add_argument('--project', default='/tmp/amr_yolo_runs')
    t.add_argument('--name', default='warehouse')
    t.add_argument('--seed', type=int, default=0)
    t.add_argument('--amp', action='store_true', help='혼합 정밀도 (AMP 점검이 네트워크를 쓸 수 있다)')
    t.add_argument('--cache', default='', help="'ram' 이면 디코딩한 이미지를 메모리에 (CPU 부하 절감)")
    t.add_argument('--out', default='', help='best.pt 복사 경로')
    e = sub.add_parser('eval')
    e.add_argument('--data', required=True)
    e.add_argument('--weights', required=True)
    e.add_argument('--device', default='0')
    e.add_argument('--project', default='/tmp/amr_yolo_runs')
    e.add_argument('--conf', type=float, default=0.35)
    e.add_argument('--out', default='', help='지표 JSON 경로')
    h = sub.add_parser('heldout')
    h.add_argument('--data', required=True)
    h.add_argument('--out', required=True)
    h.add_argument('--per-class', type=int, default=4)
    h.add_argument('--classes', type=int, default=3, help='앞에서부터 이 수의 클래스를 덮는다')
    h.add_argument('--min-px', type=float, default=16.0)
    args = ap.parse_args(argv)
    if args.cmd == 'train':
        res = cmd_train(args)
    elif args.cmd == 'eval':
        res = evaluate(args.weights, args.data, args.device, args.project, conf=args.conf)
        if args.out:
            with open(args.out, 'w', encoding='utf-8') as f:
                json.dump(res, f, indent=1, ensure_ascii=False)
    else:
        res = cmd_heldout(args)
    print(json.dumps(res, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
