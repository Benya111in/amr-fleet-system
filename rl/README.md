# rl/ — 보너스 1 강화학습 내비게이션

ROS 패키지가 아니다. `rl/COLCON_IGNORE` 때문에 `colcon list`·`colcon build` 가 이 디렉터리를 보지 않는다.
배포용 C++ 컨트롤러 플러그인만 ROS 패키지로 `src/amr_rl_controller/` 에 있다.

## 디렉터리와 소유자 (1차 물결, `/home/CAPYI_AI/COORDINATION.md` §2)

| 경로 | 내용 | 소유 | 브랜치 |
|---|---|---|---|
| `rl/contract/` | 관측·행동·보상·비용·에피소드 **계약** (규범 문서 + numpy 기준 구현 + golden 벡터 + ONNX 입출력) | r0 | `feature/rl-contract` |
| `rl/isaac/` | Isaac Lab 2.3.2 학습 환경 (같은 계약) | r1 | `feature/rl-isaac-env` |
| `rl/sim2d/` | GPU 배치 2D 레이캐스트 시뮬레이터·학습 | r2 | `feature/rl-sim2d` |
| `rl/eval/` | 평가·통계 (Gazebo 짝 평가, rliable, SRCC) | r4 | `feature/rl-eval` |
| `src/amr_rl_controller/` | ONNX Runtime C++ `nav2_core::Controller` 플러그인 | r3 | `feature/rl-controller` |
| `rl/README.md`, `rl/COLCON_IGNORE` | 이 파일 | r0 | `feature/rl-contract` |

## 계약을 쓰는 방법

1. 규범: [`contract/contract.md`](contract/contract.md) — 모든 수치에 출처(파일:행 또는 보고서 절).
2. ONNX: [`contract/onnx_io.md`](contract/onnx_io.md) — 비교군 B·C 의 입력·출력 이름, 모양, dtype, 정규화 위치.
3. 기준 구현: `contract/amr_rl_contract/` (numpy 만). 정확성 우선이고 벡터화하지 않았다.
4. Golden: `contract/golden/*.json` (31 사례, 계약 v1.1.0, `gen_golden.py` 로 재생성). r1(torch)·r2(torch)·r3(C++)는
   **자기 구현이 golden 과 허용오차(`tolerance` 블록) 안에서 일치하는지** 자기 경로에 시험을 둔다.
   `contract/tests/test_golden.py` 의 `test_decision_replay`·`test_step_replay` 가 그 시험의 견본이다.
5. 계약을 바꿔야 하면 직접 고치지 말고 r0 에 요청한다 (REPORT "다른 소유자에게 요청").

```bash
# 시험 (계약 기준 구현)
docker run --rm --name amr-ai-r0-contract-test -v <worktree>:/ws -w /ws/rl/contract amr-ai/rl:0.3 \
  bash -lc 'python3 -m pytest -q tests'
```
