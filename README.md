# 지능형 물류 AMR 시스템

Gazebo Fortress 기반 스마트 물류센터 시뮬레이션에서 동적 환경에 대응하는
자율주행 AMR(Autonomous Mobile Robot) 시스템과 다중 로봇 Fleet Management 를 구현한다.

> 현재 상태: **환경 세팅 완료 / 기능 구현 착수 전**
> 아래 "구현 현황"의 체크박스가 실제 진행도다.

---

## 1. 요구 환경

| 항목 | 요구 | 이 서버 실측 |
| --- | --- | --- |
| OS (호스트) | 무관 (컨테이너 내부가 Ubuntu 22.04) | Rocky Linux 8.10 |
| Docker | Engine + compose v2 | 26.1.3 / v2.27.0 |
| GPU | YOLOv8 추론, Gazebo 렌더링 | RTX 5090 32GB (driver 580.142) |
| NVIDIA Container Toolkit | 컨테이너 GPU 패스스루 | 1.20.0 |
| CPU / RAM | 5대 동시 운용 | Ryzen 9 9950X 32스레드 / 123GB |

호스트 준비가 안 된 새 머신이라면:

```bash
sudo ./scripts/setup_host.sh
```

이 서버는 이미 적용되어 있으므로 재실행할 필요가 없다.

---

## 2. 빠른 시작

```bash
# 1) 이미지 빌드 (최초 1회, 수십 분 소요)
docker compose build dev

# 2) 개발 컨테이너 기동 및 접속
docker compose up -d dev
docker compose exec dev bash

# 3) 컨테이너 안에서 워크스페이스 빌드
./scripts/build.sh          # == colcon build --symlink-install

# 4) 테스트 + 커버리지
./scripts/test.sh
```

`src/` 는 호스트와 컨테이너가 공유(bind mount)하므로, 호스트 에디터로 수정한 코드가
컨테이너에 즉시 반영된다. 재빌드는 `colcon build` 만 다시 돌리면 된다.

### 헤드리스 환경 주의

이 서버에는 접근 가능한 `DISPLAY` 가 없다. Gazebo/RViz2 GUI 는 기본적으로 뜨지 않는다.
시뮬레이션 **연산과 센서 렌더링은 GPU EGL 로 headless 동작**하므로 개발에는 지장이 없다.
화면 확인이 필요하면 다음 중 하나를 쓴다.

- rosbag2 로 기록 후 Foxglove Studio 로 재생 (명세 9장 모니터링 요구사항과 겹침)
- 웹 대시보드(`amr_dashboard`)로 상태 확인
- 호스트에 `x11vnc` / `Xvfb` 를 올려 가상 디스플레이 연결

---

## 3. 패키지 구조

모든 소스는 `src/` 아래 기능별 ROS2 패키지로 분리된다 (명세 7장 제약).

| 패키지 | 책임 | 관련 명세 |
| --- | --- | --- |
| `amr_msgs` | 커스텀 msg/srv/action 정의 | 6장 |
| `amr_description` | URDF/xacro 로봇 모델, TF 트리 | 1장 |
| `amr_simulation` | 물류센터 월드(60x40m), 동적 장애물 | 1장 |
| `amr_bringup` | 통합 런치, 파라미터 조립 | 10장 |
| `amr_localization` | 오도메트리, SLAM, AMCL, EKF 퓨전 | 2·3장 |
| `amr_navigation` | A*/DWA 직접 구현, Costmap, 경로 추종 | 4·5장 |
| `amr_perception` | YOLOv8 인식, 2D→3D 변환, 객체 추적 | 6·7장 |
| `amr_behavior` | BehaviorTree 작업 수행, 도킹 | 8장 |
| `amr_fleet` | 작업 할당, Traffic/교착 관리 | 9장 |
| `amr_dashboard` | 웹 모니터링, KPI 시각화 | 9장 |

```
.
├── docker-compose.yml       # 명세 4장 요구 산출물
├── docker/
│   ├── Dockerfile           # ros:humble 베이스
│   └── entrypoint.sh
├── src/                     # ROS2 패키지 (위 표)
├── config/                  # 전역 파라미터 YAML
├── maps/                    # SLAM 산출 맵
├── docs/
│   ├── architecture/        # 컴포넌트/시퀀스 다이어그램
│   ├── algorithms/          # DWA, EKF, SLAM 튜닝 근거
│   └── reports/             # 성능 측정 리포트
├── scripts/                 # 빌드/테스트/셋업
├── tests/integration/       # 통합 테스트 시나리오 (10개 이상)
└── logs/                    # 런타임 로그, 커버리지 산출물
```

---

## 4. 브랜치 전략

명세 4장에 따라 Git Flow(Main/Develop/Feature)와 Conventional Commits 를 사용한다.

```bash
./scripts/setup_gitflow.sh      # develop 브랜치 + 커밋 템플릿 등록
```

- `main` — 릴리스만 병합, 직접 커밋 금지
- `develop` — 통합 브랜치
- `feature/*` — 기능 단위 작업

커밋 형식: `feat(navigation): A* 글로벌 플래너 직접 구현`

---

## 5. 구현 현황

환경 세팅만 완료된 상태이며, 아래는 전부 미착수다.

### 인프라
- [x] Docker + compose 환경
- [x] GPU 패스스루 (nvidia-container-toolkit)
- [x] ROS2 Humble + Gazebo Fortress 이미지
- [x] 패키지 스켈레톤, Git Flow

### 기능 (명세 4장)
- [ ] 1. 시뮬레이션 환경 및 로봇 모델링 (60x40m 월드, URDF, 센서 노이즈)
- [ ] 2. 로봇 기초 (키네마틱스, 오도메트리, TF 트리)
- [ ] 3. 위치 추정 (SLAM, AMCL, EKF 퓨전)
- [ ] 4. 경로 계획 (A*/DWA 직접 구현, Costmap)
- [ ] 5. 모션 제어 (Pure Pursuit, PID, 속도 프로파일)
- [ ] 6. AI 인지 (YOLOv8, Pinhole 2D→3D)
- [ ] 7. 동적 환경 대응 (추적, TTC, 회피, 안전)
- [ ] 8. 작업 수행 (BehaviorTree, 도킹)
- [ ] 9. Fleet Management (5대, 할당, 교착 해소, 대시보드)
- [ ] 10. 통합·테스트·문서화 (커버리지 70%)

권장 구현 순서는 명세 4장 하단 "구현 순서 권장" 참고.

---

## 6. 문서

- [시스템 아키텍처](docs/architecture/) — 컴포넌트/시퀀스 다이어그램
- [핵심 알고리즘](docs/algorithms/) — DWA, EKF, SLAM 파라미터 튜닝 근거
- [성능 리포트](docs/reports/) — 위치 추정 RMSE, CTE, 응답 시간, 커버리지
