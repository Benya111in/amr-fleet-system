"""Pytest 설정 — rl/isaac 와 rl/contract 를 import 경로에 넣는다."""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC = os.path.dirname(HERE)
CONTRACT = os.path.join(os.path.dirname(ISAAC), 'contract')
for p in (ISAAC, CONTRACT):
    if p not in sys.path:
        sys.path.insert(0, p)
