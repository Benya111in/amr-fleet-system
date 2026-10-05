# 정책 ONNX 입출력 계약 v1.1.0 (입출력 자체는 v1.0.0 과 같다)

r1(Isaac)·r2(2D)가 내보내는 정책과 r3(`src/amr_rl_controller`, ONNX Runtime C++ 1.23.2 `/opt/onnxruntime`)가
읽는 정책의 인터페이스. 관측·행동의 의미는 [`contract.md`](contract.md) §4–§5 가 정한다.
고정물 `fixture_{b,c}.onnx` 는 **이 계약을 지키는 가짜 정책**(시드 고정 난수 가중치, 학습 안 함)이고
`onnx/fixtures/fixture_expected.json` 은 golden decision 17 사례(`d*.json`)의 obs·mask 에 대한 onnxruntime(CPU) 출력이다 —
r3 는 학습된 정책을 기다리지 않고 C++ 에서 같은 출력이 나오는지 시험할 수 있다.
저장소 `.gitignore:43` 이 `*.onnx` 를 무시하므로 **모델 파일은 커밋하지 않는다**. `amr-ai/rl:0.3` 안에서
`python3 rl/contract/onnx/make_fixtures.py [출력 디렉터리]` 로 만든다 (기본 `rl/contract/onnx/fixtures/`, torch 2.11 CPU
에서 결정적 — `tests/test_onnx_fixtures.py` 가 다시 만든 모델의 출력이 커밋된 JSON 과 1e-5 안에서 같음을 확인한다).

## 공통

| 항목 | 값 |
|---|---|
| opset | **17** (ORT 1.23.2 지원 범위 안) |
| dtype | 입력·출력 모두 float32, 단 C 의 `action` 은 int64 |
| 배치 축 | 0 번 축 동적 (`batch`), 플러그인은 1 로 부른다 |
| 관측 | `obs` [batch, 291] — `amr_rl_contract.observation.build_observation` 의 출력 그대로 (계약 정규화 포함) |
| 학습 정규화 | running mean/std 는 **모델 안**: x = clip((obs − μ)/√(σ² + 1e-8), −10, 10). 플러그인은 계약 관측만 만든다 |
| 메타데이터 (`metadata_props`) | 필수: `contract_version`(=1.1.0), `arm`(B·C), `obs_dim`(291), `dt`(0.1), `normalization`(inside). 권장: `algo`, `seed`, `git_commit`, `train_steps`, `simulator`(isaac·sim2d) |
| 내보내기 | `amr-ai/rl:0.3` 에는 `onnxscript` 가 없어 torch 2.11 의 기본(dynamo) 내보내기가 안 된다 → `torch.onnx.export(..., dynamo=False, opset_version=17)` (2026-10-06 이 컨테이너에서 확인) |
| 버전 불일치 | 플러그인은 `contract_version` 의 주 버전이 다르거나 `obs_dim` 이 다르면 `configure()` 에서 실패해야 한다 |

## 비교군 B (연속 가속도)

| 방향 | 이름 | 모양 | 의미 |
|---|---|---|---|
| 입력 | `obs` | [batch, 291] float32 | 계약 관측 |
| 출력 | `action` | [batch, 2] float32 | **결정론적 행동** tanh(μ) ∈ [−1, 1]². 플러그인은 `integrate_b` 로 (a_v, a_ω) = (u₀·1.0, u₁·2.0) → ray 클램프 |
| 출력 | `mu` | [batch, 2] float32 | tanh 전 평균 (확률적·저온 평가용) |
| 출력 | `log_std` | [batch, 2] float32 | 가우시안 log σ (tanh 전). 확률적 평가: u = tanh(μ + σ·ε), 저온 = σ × 0.3 (「이론」 167행) |

## 비교군 C (후보 선택)

| 방향 | 이름 | 모양 | 의미 |
|---|---|---|---|
| 입력 | `obs` | [batch, 291] float32 | 계약 관측 |
| 입력 | `mask` | [batch, 100] float32 | `compute_mask` 의 결과, 1.0 = 유효 / 0.0 = 무효. 인덱스 99(폴백)는 항상 1.0 |
| 출력 | `logits` | [batch, 100] float32 | **마스크를 모델 안에서 적용한** logit: 무효 셀 = 원 logit − 1e9 (−inf 를 쓰지 않는다 — softmax 가 NaN 이 되지 않게) |
| 출력 | `action` | [batch] int64 | argmax(`logits`), 동률이면 작은 인덱스 (ONNX ArgMax `select_last_index=0`) |

- 마스크를 **입력으로** 받는 이유: 정책 관측(섹터 LiDAR)이 마스크 입력(전체 스캔)보다 거칠면 가용성이 정책 관점에서
  확률적이 되고, 그때 정책·critic 은 마스크를 입력으로 받아야 한다 (contract.md §5.2, 「이론」 67행).
- 플러그인 의무: `mask[action] == 1` 을 다시 확인하고 아니면 폴백(99)을 쓴다; 폴백 선택 빈도를 진단 토픽으로 낸다.
- chattering 방지 히스테리시스는 플러그인이 `logits[prev] += β` 후 argmax 로 (기본 β = 0, contract.md §5.2).

## 선택: 순환 정책 (GRU)

| 방향 | 이름 | 모양 | 의미 |
|---|---|---|---|
| 입력 | `h_in` | [layers, batch, hidden] float32 | 직전 은닉 상태. 새 FollowPath 목표·플러그인 `activate()`·`reset()` 에서 0 |
| 출력 | `h_out` | [layers, batch, hidden] float32 | 다음 결정에 넣는다 (결정 주기 0.1 s 마다 한 번 — 20 Hz 틱마다가 아니다) |

`layers`·`hidden` 은 메타데이터 `rnn_layers`·`rnn_hidden` 으로 알린다. 학습의 BPTT 경계와 배포의 0 초기화 시점이
같아야 한다.

## 플러그인 쪽 처리 순서 (r3, 결정 주기 0.1 s)

1. 스캔·트랙·경로·odom·safety 상태로 `State` 를 만든다 (contract.md §4 의 선별·외삽·투영 규칙 그대로).
2. 관측 291 을 만든다 (golden `d*` 와 1e-5 안에서 일치해야 한다).
3. C 면 마스크 100 을 만든다 (golden 과 정확히 일치).
4. ORT 실행 → B: `action` → `integrate_b`; C: `action` → 확인 → `integrate_c`.
5. 다음 결정까지 가속도를 붙잡고 20 Hz 로 v_cmd + a·0.05 를 발행; 실행 명령 가속도를 `prev_actions` 에 넣는다.
6. 목표 성공 조건(xy ≤ 0.10, |v| ≤ 0.05)에서는 정책 대신 제자리 yaw 정렬 (contract.md §8).
