"""Pytest 설정 — rl/sim2d 와 rl/contract 를 import 경로에 넣는다."""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM2D = os.path.dirname(HERE)
CONTRACT = os.path.join(os.path.dirname(SIM2D), 'contract')
for p in (SIM2D, CONTRACT, os.path.join(CONTRACT, 'golden')):
    if p not in sys.path:
        sys.path.insert(0, p)
