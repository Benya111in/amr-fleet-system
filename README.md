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
# 0) 저장소 클론 (SSH 키가 있으면 git@github.com:Benya111in/amr-fleet-system.git)
git clone https://github.com/Benya111in/amr-fleet-system.git
cd amr-fleet-system

# 1) YOLOv8 가중치 내려받기 (대용량이라 git 에 없음 — 최초 1회.
#    호스트에 wget 이 없으면 3) 이미지 빌드 뒤 `docker compose run --rm dev ./scripts/download_models.sh` 로
#    컨테이너 안에서 실행해도 된다. src/ 가 공유되므로 결과는 같다)
./scripts/download_models.sh

# 2) 사람별 환경변수 — 서버를 여러 명이 같이 쓰므로 프로젝트 이름/ROS 도메인/Gazebo 파티션을
#    사람마다 다르게 준다 (.env 는 git 에 올라가지 않는다). 예시값(amr_hong / 42)은 그대로도
#    동작하지만 두 사람이 같은 값을 쓰면 서로의 컨테이너·볼륨을 재생성하고 토픽이 섞인다 — 꼭 바꾼다
cp .env.example .env && vi .env      # docker compose config 로 값 검증 가능

# 3) 이미지 빌드 (최초 1회, 수십 분 소요) — build: 는 dev 서비스에만 있어 한 번만 빌드되고
#    나머지 서비스는 그 이미지(amr-fleet-system:latest)를 그대로 쓴다
docker compose build

# 4) 기동: builder 가 colcon build 를 수행하고 dev 컨테이너가 뜬다
docker compose up -d && docker compose wait builder   # builder 종료(exit 0)까지 대기. 진행 상황: docker compose logs -f builder
docker compose exec dev bash          # 셸 접속 — 대화형 bash 만 ROS 와 install/ 오버레이를 자동 소싱한다
                                      # (/etc/bash.bashrc 경유. builder 종료 전에 연 셸은 다시 연다)
./scripts/verify_env.sh               # 환경 검증 (37개 항목, 전부 통과해야 한다)

#    셸을 열지 않고 명령 하나만 돌릴 때: exec 는 엔트리포인트를 거치지 않으므로
#    `exec dev bash -c '...'` / `exec dev ros2 ...` 에는 ROS 환경이 없다. 아래 둘 중 하나를 쓴다
docker compose exec dev bash -ic 'ros2 topic list'   # -i: 대화형 bash 로 강제 → bashrc 가 소싱
docker compose run --rm dev ros2 topic list          # 새 컨테이너 — 엔트리포인트가 소싱

# 5) 소스 수정 후 재빌드 (컨테이너 안) — 또는 호스트에서 docker compose up builder
./scripts/build.sh              # colcon build --symlink-install + 경고 플래그(-Wall -Wextra -Wpedantic)
WERROR=1 ./scripts/build.sh     # -Werror 추가: 경고를 에러로 승격 (명세 7장 "경고 0" 검증용)

# 6) 테스트 + 커버리지 (컨테이너 안)
./scripts/test.sh

# 7) 전체 시스템 기동 (런치 파일이 갖춰진 뒤): builder 성공 후 simulation → localization →
#    navigation → fleet → dashboard 순으로 뜬다 (perception 은 simulation 뒤)
docker compose --profile run up -d
```

`src/` 는 호스트와 컨테이너가 공유(bind mount)하므로, 호스트 에디터로 수정한 코드가
컨테이너에 즉시 반영된다. colcon 산출물(build/ install/ log/)은 이름 있는 볼륨
(`<프로젝트>_ros2_ws_build` 등)에 있어 dev 와 run 프로필 서비스가 한 워크스페이스를 공유하고,
컨테이너를 재생성해도 빌드가 사라지지 않는다. 빌드까지 지우려면 `docker compose down -v`.

`docker/Dockerfile` 이 바뀐 커밋을 pull 했다면 이미지를 다시 만들고 컨테이너를 재생성한다
(기존 컨테이너는 옛 이미지로 계속 돈다). 볼륨의 빌드는 유지되므로 builder 가 증분 빌드만 한다.

```bash
docker compose build && docker compose up -d
```

**마이그레이션 — 컨테이너 이름이 `amr_dev` 로 고정되어 있던 시절부터 쓰던 프로젝트라면** 이 변경
(`container_name` 제거 + `ros2_ws_*` 볼륨) 뒤의 최초 `docker compose up -d` 가 서비스 설정 변경을 감지해
옛 컨테이너 `amr_dev` 를 멈추고 지운 뒤 `<프로젝트>-dev-1` 을 새로 만들고, 비어 있는 새 볼륨을
`build/ install/ log/` 에 마운트한다. 옛 컨테이너 안에만 있던 것(컨테이너 레이어의 빌드 산출물,
홈 디렉토리 설정, 임시 파일 등)은 사라지므로 **먼저 꺼내 둔다**:
`docker cp amr_dev:<컨테이너 안 경로> <호스트 경로>`. bind mount 인 `src/ config/ maps/ logs/` 는
호스트에 그대로 있다. 이후 builder 가 새 볼륨에 한 번 전체 빌드를 한다. `.env` 로 `COMPOSE_PROJECT_NAME`
을 새로 줬다면 옛 프로젝트의 컨테이너는 건드리지 않고 남으므로 따로 지운다:
`docker compose -p <옛 프로젝트 이름> down`.

`network_mode: host` 라서 같은 `ROS_DOMAIN_ID` 를 쓰는 LAN 의 다른 호스트와 DDS 트래픽이 오간다.
격리가 필요하면 `.env` 에 `ROS_LOCALHOST_ONLY=1` 을 준다 (컨테이너끼리는 계속 통신된다).

### 테스트와 커버리지

`./scripts/test.sh` 는 `colcon test` → `colcon test-result --verbose` → `colcon coveragepy-result`
순으로 돌고, 테스트가 하나라도 실패하면 0 이 아닌 코드로 끝난다.
린터(flake8 / pep257 / xmllint / lint_cmake / cpplint / cppcheck / uncrustify)도 `colcon test` 의
일부라서 스타일 위반은 테스트 실패로 잡힌다. 파일별 저작권 헤더 검사(ament_copyright)만 생략한다
(라이선스는 각 `package.xml` 에 선언).

- 요약 줄(예: `Summary: 72 tests, 0 errors, 0 failures, 0 skipped`)에서 failures/errors 가 0 이어야 한다.
  실패한 테스트는 그 위에 `- <패키지>.<린터/테스트> ...` 와 실패 메시지로 나열된다.
- 커버리지는 패키지마다 따로 측정된다 (`package.xml` 에 `<test_depend>python3-pytest-cov</test_depend>`
  가 있는 패키지). 터미널에 `Starting >>> <패키지>` 아래 그 패키지의 `coverage report` 가 모듈(파일)
  단위로 나오고, 마지막에 전체 합산 표가 나온다. `test/` 아래 테스트 파일은 집계에서 빼므로
  `Cover` 열이 곧 모듈 커버리지다. 명세 4.10 목표: 주요 모듈 70% 이상.
  `No .coverage files found for package '...'` 경고는 그 패키지에 측정된 파이썬 테스트가 아직
  없다는 뜻이다 (리소스 전용 패키지 `amr_msgs`/`amr_description`/`amr_simulation`/`amr_bringup` 은 정상).
- HTML: `logs/coverage/htmlcov/index.html` (전체 합산).
  패키지별 HTML 은 `build/<패키지>/pytest_cov/<패키지>_pytest/coverage.html/`.
- `xmllint` 는 `package.xml` 스키마를 download.ros.org 에서 받아 검증하므로 네트워크가 없으면
  xmllint 만 실패한다.
- 특정 패키지만: `./scripts/test.sh --packages-select amr_fleet` (인자는 `colcon test` 로 전달).
  이전 실행의 결과 파일은 실행 시작 시 지우므로(`colcon test-result --delete-yes`) 요약과 종료 코드는
  이번에 돌린 패키지만 반영한다. 커버리지 합산은 `build/` 에 남은 `.coverage` 를 모두 읽으므로
  다른 패키지의 지난 측정값이 함께 나올 수 있다.

### 헤드리스 환경 주의

이 서버에는 접근 가능한 `DISPLAY` 가 없다. Gazebo/RViz2 GUI 는 기본적으로 뜨지 않는다.
시뮬레이션 **연산과 센서 렌더링은 GPU EGL 로 headless 동작**하므로 개발에는 지장이 없다.
화면 확인이 필요하면 다음 중 하나를 쓴다.

- Foxglove Studio 를 실시간으로 연결: 컨테이너에서 `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
  (기본 포트 8765, 이미지에 `ros-humble-foxglove-bridge` 포함) → 노트북의 Foxglove 에서
  `ws://<서버>:8765` 접속. 토픽/TF/마커/카메라를 RViz2 없이 본다 (명세 4.9 모니터링 요구사항과 겹침)
- rosbag2 로 기록 후 Foxglove Studio 로 재생
- 웹 대시보드(`amr_dashboard`)로 상태 확인
- 호스트에 `x11vnc` / `Xvfb` 를 올려 가상 디스플레이 연결 (RViz2/Groot 화면이 꼭 필요할 때)

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
원격은 GitHub [`Benya111in/amr-fleet-system`](https://github.com/Benya111in/amr-fleet-system) 이고,
`main`/`develop` 에는 ruleset 이 걸려 있어 직접 push·force-push·삭제가 막혀 있다.

| 브랜치 | 역할 | 들어오는 길 |
| --- | --- | --- |
| `main` | 릴리스 | `develop` → `main` PR, 승인 2명 |
| `develop` | 통합 | `feature/*` → `develop` PR, 승인 1명 |
| `feature/*` | 기능 단위 작업 | 직접 커밋 |

```bash
./scripts/setup_gitflow.sh              # 로컬 develop 브랜치 + 커밋 템플릿(.gitmessage) 등록
git checkout develop && git pull
git checkout -b feature/<이름>          # 작업 → push → develop 대상 PR
```

커밋 형식: `feat(navigation): A* 글로벌 플래너 직접 구현`

---

## 5. 구현 현황

환경 세팅만 완료된 상태이며, 기능은 전부 미착수다.

### 인프라
- [x] Docker + compose 환경
- [x] GPU 패스스루 (nvidia-container-toolkit)
- [x] ROS2 Humble + Gazebo Fortress 이미지
- [x] 패키지 스켈레톤
- [x] GitHub 원격 + Git Flow 브랜치 보호(ruleset), 빌드/테스트/커버리지 스크립트

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
