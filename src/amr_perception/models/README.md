# amr_perception 모델

| 파일 | 내용 |
| --- | --- |
| `yolov8n_warehouse.pt` | YOLOv8n 미세조정 가중치 (6.2 MB, ultralytics 8.4.155). `yolo_node` 기본 `weights` |
| `heldout/` | 보류 구역(test, x ≥ 15 m) 프레임 12 장 + YOLO 라벨 — `test/test_yolo_heldout.py` 회귀 시험 |

## 출처

- 기반: ultralytics 공식 배포 `yolov8n.pt` (COCO 사전학습, AGPL-3.0 — ultralytics 라이선스를 따른다).
- 데이터: 이 저장소의 Gazebo 창고 월드(`amr_simulation`, 카메라 전면 0.25 m·RGB 노이즈 켬)에서
  `scripts/dataset_capture.py` 로 수집한 5671 장, 지면 진실 투영 라벨, 창고 **구역**으로 train / val / test 분할.
- 학습: `scripts/yolo_train_eval.py train` — 80 epoch, 640 px, batch 32, RTX 5090 (780 s). 학습 클래스
  box / person / sign / forklift / amr, 출력은 `config/classes.yaml` 이 box / person / sign 만 통과.
- 결과: 보류 test 구역 mAP50 0.935, mAP50-95 0.727 — 전체 표는 `docs/algorithms/perception.md` §5, §8.3.

## 재생성

```bash
# 1) 시뮬레이션 (월드 + 로봇 5 대) 을 띄운 뒤, 워크스페이스를 소싱하고 수집 — 방법은 dataset_capture.py 머리말
python3 src/amr_perception/scripts/dataset_capture.py --out /data/ds --steps 1600 \
    --world src/amr_simulation/worlds/warehouse.sdf --map maps/warehouse.yaml \
    --robots amr_01,amr_02,amr_03,amr_04,amr_05
# 2) 학습 → 이 디렉터리로 복사 (옵티마이저 제거)
python3 src/amr_perception/scripts/yolo_train_eval.py train --data /data/ds/data.yaml --model yolov8n.pt \
    --epochs 80 --device 0 --project /data/runs --name v8n --out src/amr_perception/models/yolov8n_warehouse.pt
# 3) 평가, 보류 프레임 갱신
python3 src/amr_perception/scripts/yolo_train_eval.py eval --data /data/ds/data.yaml --device 0 \
    --weights src/amr_perception/models/yolov8n_warehouse.pt
python3 src/amr_perception/scripts/yolo_train_eval.py heldout --data /data/ds/data.yaml --per-class 4 \
    --out src/amr_perception/models/heldout
```

월드(물체 형상·배치)나 카메라 설정이 바뀌면 다시 만든다. 가중치가 없으면 `yolo_node` 는 COCO `yolov8n.pt` 로 내려가지만
이 월드의 상자·표지판을 거의 찾지 못한다.
