"""
보너스 2 (3D LiDAR SLAM) 평가 도구 — rclpy 비의존 순수 모듈.

- se2      : 정렬 없는 SE(2) ATE (+ 보조 정렬 origin / se2 Umeyama), 거리 RPE, 시간 창 RTE, 명세 구간별 오차
- gridmap  : 같은 격자 공간의 2D 지도 지표 (ADNN₀, Chamfer, 정밀도·재현율·F1@τ, 공통 격자 IoU)
- cloud    : 3D 점군 지도 지표 (F-score@5·10 cm, AC/COM, Chamfer 두 규약, 유령 점 비율, SA/DA/AA)
- results  : 결과 CSV 스키마 (긴 형식, 한 행 = 지표 하나)

정답(GT)은 src/amr_simulation/worlds/gen_warehouse_world.py --gt-dir 가 만든다
(docs/research/slam3d/evaluation.md).
"""
