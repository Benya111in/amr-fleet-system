# A2-3 학습 정책 추론 지연 재측정 (C++ ORT, 20 Hz 예산 대비)

- 시각: 2026-10-06 18:59 KST · 에이전트: b1-plugin · 단계: A2
- 코드: `a7da8a2` (브랜치 `feature/b1-plugin`)
- 관련 실행 id: (끝나면 채운다 — runs/b1-plugin.jsonl)

## 질문

1차 물결 r3 는 **학습되지 않은 크기 대표 모델**(무작위 가중치 MLP256 등)로 지연을 쟀다
(`/home/CAPYI_AI/logs/agents/r3-controller/latency_bench_2.log`: 한 결정 p99 ≤ 0.22 ms). 실제 학습 정책
(Isaac v1.2 PPO, actor MLP 512-256-128 ELU, 관측 정규화 포함)은 더 크다. 플러그인의 한 결정
(관측 291 + ORT 추론, 비교군 B 는 마스크 없음)이 20 Hz 제어 주기 예산 50 ms 와 플러그인 시간 초과 상한
(`rl.max_inference_ms` 25 ms) 안에 드는가?

## 가설과 예측

- 망이 r3 의 MLP256 보다 약 4 배 크다(첫 층 291×512). 예측: 추론 p99 < 1 ms, 한 결정 p99 < 1 ms (예산의 2 % 미만).
  **기각 조건**: p99 > 5 ms (예산의 10 %) 이면 원인을 찾는다 (스레드·부하·그래프 최적화).
- 공유 서버 부하에 따라 최댓값(꼬리)은 수 ms 까지 튈 수 있다 (r3 실측 최댓값 7.43 ms, loadavg ≈ 100) — 꼬리는 따로 적는다.

## 설정

- 도구: `latency_bench` (r3, `src/amr_rl_controller/tools/latency_bench.cpp`) — golden d01–d17 상태 순환, 그리고
  합성 최악 장면(720 빔 전부 유효, 트랙 8, 경로 600 점). 5000 회 + 워밍업 200, intra_op 1 (플러그인 기본).
- 모델: v12_s0·s1·s2 `policy_final.onnx`, 모드 B. 참고로 intra_op 4 도 한 번.
- 컨테이너 `amr-ai/rl:0.3`, 시작·끝 loadavg 기록 (공유 부하 하 예비값 여부 표기). 같은 측정을 두 번 (시간 간격을 두고).
- 플러그인 안 실측 `cycle_ms`·`infer_ms` 는 A2-4 Gazebo 스모크의 `rl_controller/stats` 에서 함께 보고한다.
- 판정 대상: 한 결정(total) p50/p99/max, 추론(infer) p50/p99/max, p99/50 ms 비율.

## 결과

(실행 뒤 채운다)

## 해석

(실행 뒤 채운다)

## 다음

(실행 뒤 채운다)
