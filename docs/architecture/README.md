# 시스템 아키텍처

> 명세 10장 요구 산출물. 컴포넌트 다이어그램과 시퀀스 다이어그램을 포함해야 한다.

## 작성 예정 항목

- [ ] 전체 컴포넌트 다이어그램 (패키지 간 의존 관계)
- [ ] ROS2 노드 그래프 (토픽/서비스/액션 연결)
- [ ] TF 트리 구조도 (`map → odom → base_link → 센서`)
- [ ] 시퀀스 다이어그램
  - [ ] 작업 요청 → 할당 → 주행 → 도킹 → 완료
  - [ ] 동적 장애물 감지 → 추적 → TTC 계산 → 회피 → 복귀
  - [ ] 교착 탐지 → 해소 전략 적용
- [ ] 다중 로봇 네임스페이스 설계

## TF 트리 (설계 기준)

`config/sensors.yaml` 의 extrinsic 값과 반드시 일치해야 한다.

```
map
└── odom                       (ekf_filter_node_map 발행: map → odom. AMCL 은 tf_broadcast: false)
    └── base_footprint         (ekf_filter_node_odom 발행: odom → base_footprint. URDF 루트 링크)
        └── base_link          (robot_state_publisher, URDF 고정 조인트)
            ├── lidar_link            (x: 0.15, y: 0, z: 0.20)
            ├── camera_link           (x: 0.18, y: 0, z: 0.25)
            │   ├── camera_optical_frame
            │   └── camera_depth_optical_frame
            ├── imu_link              (x: 0,    y: 0, z: 0.10)
            ├── left_wheel_link
            └── right_wheel_link
```

다중 로봇(명세 4.9): `map` 은 공유하고 그 아래 프레임은 `amr_01/odom`, `amr_01/base_footprint` 처럼
네임스페이스 접두사를 붙인다 (런치에서 주입, `config/ekf.yaml` 머리말 참고). 자식 프레임이 로봇마다 다르므로 충돌이 없다.

검증: `ros2 run tf2_tools view_frames`
