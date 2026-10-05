"""
계약 기준 구현(rl/contract/amr_rl_contract) 을 import 경로에 넣는다.

상수는 복사하지 않고 계약의 constants.py 를 그대로 쓴다 (COORDINATION §2.1 단일 진실 공급원).
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
SIM2D_DIR = os.path.dirname(_HERE)
RL_DIR = os.path.dirname(SIM2D_DIR)
REPO_DIR = os.path.dirname(RL_DIR)
CONTRACT_DIR = os.path.join(RL_DIR, 'contract')
GOLDEN_DIR = os.path.join(CONTRACT_DIR, 'golden')
if CONTRACT_DIR not in sys.path:
    sys.path.insert(0, CONTRACT_DIR)

from amr_rl_contract import constants as C  # noqa: E402,F401
