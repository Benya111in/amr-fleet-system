"""
P4 사전 등록 계획 — 스윕 셀·주 비교·판정 규칙을 실행 전에 코드로 고정한다 (노트 P4-3 · P4-4).

이 파일을 바꾸면 노트에 이유를 적는다. report.py 가 이 계획대로 표를 만든다.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

RATE = 1.0 / 45.0            # 부하 보정 P4-2 (보정 시드 9001–9010) 판정값
HORIZON_S = 1200.0
T_FAULT_S = 300.0

# ---------------------------------------------------------------------- A/A (P4-3)
AA_ARMS = ['A1_nearest_central', 'A6_cbaa_vote', 'A9_hybrid_vote']
AA_CELLS: List[Tuple[float, str]] = [(0.05, 'F0'), (0.2, 'F0'), (0.2, 'F1')]
AA_METRICS = ['completed', 'latency_rmean', 'latency_p50', 'wait_s', 'completed_after_fault']

# ---------------------------------------------------------------------- 본 스윕 (P4-4)
MAIN_ARMS = ['A1_nearest_central', 'A2_hungarian_central', 'A3_ssi_central', 'A4_hungarian_vote',
             'A5_cbaa_central', 'A6_cbaa_vote', 'A7_cbaa_opt_vote', 'A8_cbaa_event_vote',
             'A9_hybrid_vote']
MAIN_LOSSES = [0.0, 0.05, 0.2, 0.4]
MAIN_FAULTS = ['F0', 'F1', 'F5', 'F6']
DIAG_ARMS = ['X0_hungarian_none']          # F0 만, 진단(교통 무시 기준선)
SEEDS = '1-30'

STRICT_ARMS = ['A5_cbaa_central', 'A6_cbaa_vote', 'A8_cbaa_event_vote', 'A9_hybrid_vote']
SAFE_ARMS = MAIN_ARMS                      # 동시 점유 0 이 요구되는 비교군 (X0 제외)

# 주 비교 (Holm 보정 가족). (id, A, B, 손실, 고장, 지표, 판정 종류, 예측)
PRIMARY: List[Dict] = [
    {'id': 'P1', 'a': 'A1_nearest_central', 'b': 'A9_hybrid_vote', 'loss': 0.2, 'fault': 'F1',
     'metric': 'completed', 'kind': 'superiority',
     'pred': '서버 충돌 뒤 현행은 새 할당·구역 허가가 멈추고 제안 설계는 이어 간다: B − A ≥ +10 건'},
    {'id': 'P2', 'a': 'A4_hungarian_vote', 'b': 'A6_cbaa_vote', 'loss': 0.2, 'fault': 'F0',
     'metric': 'latency_rmean', 'kind': 'equivalence',
     'pred': '서버가 살아 있을 때 할당 방식(중앙 Hungarian 대 CBAA 엄격)의 지연 차는 ±10 s 안'},
    {'id': 'P3', 'a': 'A2_hungarian_central', 'b': 'A4_hungarian_vote', 'loss': 0.2, 'fault': 'F0',
     'metric': 'latency_rmean', 'kind': 'equivalence',
     'pred': '교통(중앙 토큰 대 과반 투표)의 지연 차는 ±10 s 안, 부호는 투표가 느린 쪽(+)'},
    {'id': 'P4', 'a': 'A1_nearest_central', 'b': 'A9_hybrid_vote', 'loss': 0.05, 'fault': 'F0',
     'metric': 'latency_rmean', 'kind': 'equivalence',
     'pred': '무장애·명세 통신에서 제안 설계의 평상시 비용(지연)은 현행 대비 ±10 s 안'},
]
EQUIV_MARGIN = {'latency_rmean': 10.0, 'latency_p50': 10.0}
SUPERIORITY_MIN_K = 25                     # P1: 쌍 30 중 같은 방향(B > A) 최소 수

# 판정 규칙 (report.py 가 그대로 적용)
#  superiority : Holm p < 0.05 이고 CI 하한 > 0 이고 k+ ≥ SUPERIORITY_MIN_K → "우월", 아니면 "판정 불가"
#  equivalence : 95 % CI 가 ±margin 안 → "동등(±margin)", Holm p < 0.05 이고 CI 가 0 을 빼면 → "차이",
#                둘 다면 "차이 있으나 margin 안", 아무것도 아니면 "판정 불가"
#  A/A 바닥    : 탐색 표에서 |평균 쌍차| < 같은 지표·셀의 A/A MDE(n=30) 이면 "바닥 아래" 표시

# 탐색 비교 (p 를 주장하지 않는다 — 효과크기·CI·k/n 만)
EXPLORE_VS = 'A1_nearest_central'          # 각 비교군 대 현행, 셀마다
EXPLORE_METRICS = ['completed', 'latency_rmean', 'completed_after_fault', 'wait_s', 'msgs_sent']
FACTORIAL = {'CC': 'A2_hungarian_central', 'CV': 'A4_hungarian_vote',
             'DC': 'A5_cbaa_central', 'DV': 'A6_cbaa_vote'}

__all__ = ['AA_ARMS', 'AA_CELLS', 'AA_METRICS', 'DIAG_ARMS', 'EQUIV_MARGIN',
           'EXPLORE_METRICS', 'EXPLORE_VS', 'FACTORIAL', 'HORIZON_S', 'MAIN_ARMS', 'MAIN_FAULTS',
           'MAIN_LOSSES', 'PRIMARY', 'RATE', 'SAFE_ARMS', 'SEEDS', 'STRICT_ARMS',
           'SUPERIORITY_MIN_K', 'T_FAULT_S']
