# 시스템 아키텍처

> 명세 10장 요구 산출물. 컴포넌트 다이어그램과 시퀀스 다이어그램을 포함해야 한다.

## 문서 목록

설계 단계 문서다. 구현하면서 인터페이스/수치가 바뀌면 해당 문서를 같이 갱신한다.

- [x] 전체 컴포넌트 다이어그램 (패키지 간 의존 관계) — [components.md](components.md) §2
- [x] ROS2 노드 그래프 (토픽/서비스/액션 연결) + 노드별 인터페이스 표(API 문서 골격) — [components.md](components.md) §4–5
- [x] TF 트리 구조도 — 아래 + [multi_robot.md](multi_robot.md) §2 (다중 로봇 접두사)
- [x] 시퀀스 다이어그램 — [sequences.md](sequences.md)
  - [x] 작업 요청 → 할당 → 주행 → 도킹 → 완료 (복구 분기 포함)
  - [x] 동적 장애물 감지 → 추적 → TTC 계산 → 회피 → 복귀
  - [x] 교착 탐지 → 해소 전략 적용 (우선순위 양보 / 대체 경로)
- [x] 다중 로봇 네임스페이스 설계 — [multi_robot.md](multi_robot.md)
- [x] 센서 캘리브레이션 절차 (명세 4.1) — [sensor_calibration.md](sensor_calibration.md)
- [ ] 구현 후 갱신: 실제 `ros2 node list` / `view_frames` 결과로 다이어그램 검증

## TF 트리 (구현 기준)

`config/sensors.yaml` 의 extrinsic 값과 반드시 일치해야 한다 (xacro 가 그 파일을 직접 읽는다 — `src/amr_description/urdf/`).
`base_link` 는 차체 박스 중심(지면 +0.18 m)이라 괄호 안 값에 0.18 을 더한 것이 지면 높이다.

```
map
└── odom                       (ekf_filter_node_map 발행: map → odom. AMCL 은 tf_broadcast: false)
    └── base_footprint         (ekf_filter_node_odom 발행: odom → base_footprint. URDF 루트 링크, 지면)
        └── base_link          (robot_state_publisher, URDF 고정 조인트. z = 0.18)
            ├── lidar_link            (x: 0.15, y: 0, z: 0.02)   스캔 평면 지면 +0.20 m, 차체 안 슬롯
            ├── camera_link           (x: 0.29, y: 0, z: 0.07)   지면 +0.25 m, 전면 창(차체 전면 1 cm 안쪽)
            │   ├── camera_optical_frame
            │   └── camera_depth_optical_frame
            ├── imu_link              (x: 0,    y: 0, z: 0.10)   지면 +0.28 m
            ├── cargo_link            (x: 0,    y: 0, z: 0.15)   적재 데크(차체 상면, 지면 +0.33 m)
            ├── front_caster_link / rear_caster_link
            ├── left_wheel_link
            └── right_wheel_link
```

센서 배치 결정 (`config/sensors.yaml` 주석): LiDAR·카메라를 데크 아래에 두어 어떤 크기의 적재물도 센서를 가리지 않고,
0.20 m 를 넘는 바닥 물체(중형 박스 0.30, 대형 0.40, 다른 로봇)는 LiDAR 에 잡힌다. 명세 8장 TF 예시(lidar z 0.2,
camera z 0.25)는 지면 기준 높이로 맞췄다. 차체 안 LiDAR 가 자기 차체를 보지 않게 하는 방법(Gazebo 가시성 비트)은
[sensor_calibration.md](sensor_calibration.md) §1.1 참고.

다중 로봇(명세 4.9): `map` 은 공유하고 그 아래 프레임은 `amr_01/odom`, `amr_01/base_footprint` 처럼
네임스페이스 접두사를 붙인다 (런치에서 주입, `config/ekf.yaml` 머리말 참고). 자식 프레임이 로봇마다 다르므로 충돌이 없다.

검증: `ros2 run tf2_tools view_frames`
